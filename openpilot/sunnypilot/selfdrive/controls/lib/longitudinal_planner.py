"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.sunnypilot.selfdrive.car.cruise_button_control.controller import CruiseButtonController
from openpilot.sunnypilot.selfdrive.car.cruise_button_control.mpc import MS_TO_MPH, MpcConfig
from openpilot.sunnypilot.selfdrive.car.cruise_button_control.plant import PlantParams

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
EventNameSP = custom.OnroadEventSP.EventName

# The cruise buttons cannot brake -- the only deceleration available is coasting.
# Warn when the plan needs more than that, rather than quietly closing on a lead.
# The margin keeps borderline cases quiet, and the plan must stay unachievable for
# DECEL_WARN_FRAMES (20Hz) so a momentary dip in the trajectory does not chime.
DECEL_AUTHORITY_MARGIN = 0.15   # m/s^2
DECEL_WARN_FRAMES = 10          # 0.5s at the 20Hz model rate
DECEL_WARN_MIN_SPEED = 5.0      # m/s, no point warning at a crawl
# Only the near-term plan matters. Taking the minimum over the full 2.5s made a
# single dip at the far end raise the alert while the car was tracking fine and
# the planner was not even asking to slow -- which is what made the warning look
# unrelated to what the car was doing.
DECEL_WARN_HORIZON = 1.5        # s

# Anticipatory coasting from the end-to-end model.
#
# modelV2.action.desiredAcceleration is published every frame regardless of
# experimental mode -- the ACC planner only ignores it because is_e2e is false
# without openpilot longitudinal. The model is trained on human driving, so it
# lifts off for things the ACC MPC cannot see at all: roundabouts, junctions,
# lights. Measured over 26 curvature events on this car it asked to slow before
# the event in 21 of them, with a median lead of 9.9s.
#
# It routinely asks for far more than coasting can deliver (median -0.72 m/s^2
# against -0.19 available, 3.4x), so its request is clamped to what the plant can
# actually do and bounded in how far below the ACC target it may pull. Without
# that bound a red light would drag the setpoint to its floor, the car would
# crawl, and every junction would raise the decel alert.
E2E_COAST_MAX_DROP = 5.0 * CV.MPH_TO_MS   # most the model may pull below the ACC target
E2E_COAST_MIN_SPEED = 8.0                 # m/s, below this the buttons are near their floor


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.

    # Cruise button planner. It lives here rather than in selfdrived because the
    # solve costs ~32ms on device and selfdrived runs at 100Hz -- running it there
    # blew the 10ms budget and raised "system lagging" alerts on the road. plannerd
    # ticks at the 20Hz model rate, and the speed trajectory it plans against is
    # produced right here.
    self.cruise_button_mpc = None
    if Params().get_bool("CruiseButtonMpc"):
      self.cruise_button_mpc = CruiseButtonController(PlantParams(), MpcConfig())
    self.cruise_button = SendButtonState.none
    self._cb_t = 0.0
    self._decel_short_frames = 0
    self.e2e_coast_enabled = Params().get_bool("CruiseButtonE2eCoast")
    self.e2e_coast_active = False

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def update_cruise_button(self, sm: messaging.SubMaster) -> None:
    """Plan the next cruise button press against the unrounded speed trajectory."""
    if self.cruise_button_mpc is None:
      return

    cfg = self.cruise_button_mpc.cfg
    cs = sm['carState']
    cc = sm['carControl']

    # v_desired_trajectory lives on the parent LongitudinalPlanner (this class is
    # its base). It is one cycle old here because the MPC solves it after this runs
    # -- 50ms of staleness against a 0.8s plant deadtime, which is immaterial.
    speeds = getattr(self, 'v_desired_trajectory', None)
    if speeds is None or len(speeds) < 2:
      self.cruise_button = SendButtonState.none
      return

    # The trajectory is non-uniform and spans 2.5s; resample onto the planner grid
    # and hold the final speed past the end of the plan rather than extrapolating.
    plan_t = np.array(ModelConstants.T_IDXS[:len(speeds)], dtype=np.float64)
    grid = np.arange(cfg.n_steps) * cfg.dt
    v_des = np.interp(grid, plan_t, np.asarray(speeds, dtype=np.float64))

    # v_desired_trajectory is NOT bounded by the set speed. Upstream moved the
    # cruise limit out of the MPC (commaai/openpilot#38367): the trajectory is now
    # the lead-following solution alone, and the set speed is enforced separately
    # as an acceleration limit via get_cruise_accel(). Chasing the raw trajectory
    # therefore drives the setpoint straight past the set speed -- measured at
    # +6.8mph mean and +14mph peak over a drive, which is what made the car creep
    # up and then demand more braking than coasting could give.
    v_cruise_kph = min(float(cs.vCruise), V_CRUISE_MAX)
    if v_cruise_kph < V_CRUISE_UNSET:
      v_des = np.minimum(v_des, v_cruise_kph * CV.KPH_TO_MS)

    # Fold in the sunnypilot target (curve slowdown, map, speed limit assist).
    # update_targets() already reduces these to a single minimum, but it was only
    # reaching the ACC accel command -- which does nothing on a car where openpilot
    # has no throttle. Routing it here is what makes those features reach the
    # buttons at all.
    sp_target = float(getattr(self, 'output_v_target', 0.0) or 0.0)
    if sp_target > 1.0:
      v_des = np.minimum(v_des, sp_target)

    # Anticipatory coasting from the e2e model, clamped to coast authority and
    # bounded relative to the ACC target.
    v_des = self.apply_e2e_coast(sm, v_des, float(cs.vEgo), cfg)

    self._cb_t += cfg.dt
    ready = bool(cc.enabled and not cc.cruiseControl.override and
                 not cc.cruiseControl.cancel and not cc.cruiseControl.resume)
    driver_pressing = any(b.pressed for b in cs.buttonEvents)

    st = self.cruise_button_mpc.update(self._cb_t, float(cs.vEgo), float(cs.aEgo),
                                       float(cs.cruiseState.speedCluster * MS_TO_MPH),
                                       v_des, ready=ready, driver_pressing=driver_pressing)
    if st.action > 0:
      self.cruise_button = SendButtonState.increase
    elif st.action < 0:
      self.cruise_button = SendButtonState.decrease
    else:
      self.cruise_button = SendButtonState.none

    # After the decision, so the alert reflects what was just commanded.
    self.update_decel_authority(ready, float(cs.vEgo), float(cs.aEgo), st.action)

  def apply_e2e_coast(self, sm: messaging.SubMaster, v_des: np.ndarray, v_ego: float, cfg) -> np.ndarray:
    """
    Let the end-to-end model pull the speed target down, within what coasting can
    actually deliver.

    The model sees things the ACC planner cannot -- it asks to slow for junctions
    and roundabouts with no lead car involved. Its raw request is far beyond coast
    authority, so integrate the *clamped* request instead: the result is a target
    the buttons can actually track, which starts falling as soon as the model sees
    the event rather than when the geometry finally bends.
    """
    if not getattr(self, 'e2e_coast_enabled', False) or v_ego < E2E_COAST_MIN_SPEED:
      self.e2e_coast_active = False
      return v_des

    a_req = float(sm['modelV2'].action.desiredAcceleration)
    if not np.isfinite(a_req) or a_req >= 0.0:
      self.e2e_coast_active = False
      return v_des

    # Only what the plant can do. Asking for the model's raw -1.2 m/s^2 would build
    # a target the car can never reach, which is how the setpoint ends up parked at
    # its floor while the decel alert sounds continuously.
    a_coast = max(a_req, float(self.cruise_button_mpc.p.a_min_at(v_ego)))

    t = np.arange(cfg.n_steps) * cfg.dt
    v_e2e = v_ego + a_coast * t

    # Never pull more than E2E_COAST_MAX_DROP below what ACC already wants, so a
    # request to stop degrades into an early lift rather than a crawl.
    v_e2e = np.maximum(v_e2e, v_des - E2E_COAST_MAX_DROP)

    out = np.minimum(v_des, v_e2e)
    self.e2e_coast_active = bool(out[-1] < v_des[-1] - 0.05)
    return out

  def update_decel_authority(self, ready: bool, v_ego: float, a_ego: float, action: int) -> None:
    """
    Warn when the buttons are already doing all they can and the car is still not
    slowing as fast as the plan needs.

    The planner solves as though it can brake; driven only through the cruise
    buttons it cannot. Rather than warning on any steep-looking point in the plan,
    require all three of:

      * the near-term plan needs more deceleration than coasting can provide,
      * the planner is not asking to go faster (so it is already doing its best),
      * the car is measurably not decelerating as hard as the plan needs.

    The third condition is what ties the alert to reality: without it the warning
    fired on predictions the car went on to satisfy anyway.
    """
    a_plan = getattr(self, 'a_desired_trajectory', None)
    if not ready or a_plan is None or len(a_plan) == 0 or v_ego < DECEL_WARN_MIN_SPEED:
      self._decel_short_frames = 0
      return

    n = int(np.searchsorted(ModelConstants.T_IDXS[:len(a_plan)], DECEL_WARN_HORIZON, side="right"))
    required = float(np.min(a_plan[:max(n, 1)]))
    available = float(self.cruise_button_mpc.p.a_min_at(v_ego))

    beyond_authority = required < available - DECEL_AUTHORITY_MARGIN
    doing_our_best = action <= 0
    falling_short = a_ego > required + DECEL_AUTHORITY_MARGIN

    if beyond_authority and doing_our_best and falling_short:
      self._decel_short_frames += 1
    else:
      self._decel_short_frames = 0

    if self._decel_short_frames >= DECEL_WARN_FRAMES:
      self.events_sp.add(EventNameSP.insufficientDecelAuthority)

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)
    self.update_cruise_button(sm)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()
    longitudinalPlanSP.cruiseButton = self.cruise_button

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
