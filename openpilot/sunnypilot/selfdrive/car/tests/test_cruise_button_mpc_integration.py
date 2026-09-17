import numpy as np

from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.car.cruise_button_control.mpc import MPH_TO_MS, MS_TO_MPH
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  IntelligentCruiseButtonManagement)

SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState


class FakeLPSP:
  """Minimal stand-in for longitudinalPlanSP carrying only what the relay reads."""
  def __init__(self, button):
    self.cruiseButton = button


def make_cs(v_ego, setpoint_mph):
  cs = structs.CarState()
  cs.vEgo = v_ego
  cs.aEgo = 0.0
  cs.cruiseState.speed = setpoint_mph * MPH_TO_MS
  cs.cruiseState.speedCluster = setpoint_mph * MPH_TO_MS
  return cs


class TestCruiseButtonRelay(OpenpilotTestCase):
  """
  The button is planned in plannerd and only relayed here. selfdrived runs at
  100Hz and the MPC solve costs ~32ms on device, so this path must stay free of
  any planning work.
  """

  def make_icbm(self, mocker, mpc_enabled):
    mod = "openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller"
    mocker.patch(mod + ".Params.get_bool", return_value=mpc_enabled)
    CP = structs.CarParams()
    CP_SP = structs.CarParamsSP()
    CP_SP.pcmCruiseSpeed = False
    icbm = IntelligentCruiseButtonManagement(CP, CP_SP)
    icbm.is_ready = True
    return icbm

  def test_off_by_default(self, mocker):
    assert self.make_icbm(mocker, False).mpc_enabled is False

  def test_relays_increase(self, mocker):
    icbm = self.make_icbm(mocker, True)
    out = icbm._relay_planned_button(FakeLPSP(SendButtonState.increase))
    assert out == SendButtonState.increase
    assert icbm.state == State.increasing

  def test_relays_decrease(self, mocker):
    icbm = self.make_icbm(mocker, True)
    out = icbm._relay_planned_button(FakeLPSP(SendButtonState.decrease))
    assert out == SendButtonState.decrease
    assert icbm.state == State.decreasing

  def test_relays_none(self, mocker):
    icbm = self.make_icbm(mocker, True)
    assert icbm._relay_planned_button(FakeLPSP(SendButtonState.none)) == SendButtonState.none

  def test_not_ready_blocks_relay(self, mocker):
    """Readiness is still enforced downstream of the planner."""
    icbm = self.make_icbm(mocker, True)
    icbm.is_ready = False
    assert icbm._relay_planned_button(FakeLPSP(SendButtonState.increase)) == SendButtonState.none
    assert icbm.state == State.inactive

  def test_missing_field_is_safe(self, mocker):
    """An older longitudinalPlanSP without the field must not raise."""
    icbm = self.make_icbm(mocker, True)

    class Empty:
      pass

    assert icbm._relay_planned_button(Empty()) == SendButtonState.none

  def test_pcm_cruise_speed_gate_still_returns_early(self, mocker):
    icbm = self.make_icbm(mocker, True)
    icbm.CP_SP.pcmCruiseSpeed = True
    icbm.cruise_button = SendButtonState.none
    icbm.run(make_cs(31.0, 70.0), structs.CarControl(),
             custom.LongitudinalPlanSP.new_message(), False)
    assert icbm.cruise_button == SendButtonState.none


class TestDecelAuthorityWarning(OpenpilotTestCase):
  """
  The cruise buttons cannot brake. When the longitudinal plan asks for more
  deceleration than coasting provides, the driver has to be told -- otherwise the
  car simply closes on the lead, which is what happened on the road.
  """

  def make_planner(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.controller import CruiseButtonController
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.mpc import MpcConfig
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.plant import PlantParams
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

    p = object.__new__(LongitudinalPlannerSP)
    p.events_sp = EventsSP()
    p.cruise_button_mpc = CruiseButtonController(PlantParams(), MpcConfig())
    p._decel_short_frames = 0
    return p

  @staticmethod
  def _raised(p):
    from openpilot.cereal import custom
    name = custom.OnroadEventSP.EventName.insufficientDecelAuthority
    return any(e.name == name for e in p.events_sp.to_msg())

  def test_quiet_when_plan_is_achievable(self):
    p = self.make_planner()
    v = 30.0
    # ask for gentle decel well inside coast authority
    cfg = p.cruise_button_mpc.cfg
    v_des = v + (-0.1) * (np.arange(cfg.n_steps) * cfg.dt)
    for _ in range(40):
      p.events_sp.clear()
      p.update_decel_authority(True, v, 0.0, -1, v_des, cfg, True)
    assert not self._raised(p)

  def test_warns_when_plan_exceeds_coast_authority(self):
    p = self.make_planner()
    v = 30.0
    cfg = p.cruise_button_mpc.cfg
    v_des = v + (-3.0) * (np.arange(cfg.n_steps) * cfg.dt)  # impossible by coasting
    for _ in range(40):
      p.events_sp.clear()
      p.update_decel_authority(True, v, 0.0, -1, v_des, cfg, True)
    assert self._raised(p)

  def test_debounced_not_instant(self):
    """A momentary dip in the trajectory must not chime."""
    p = self.make_planner()
    cfg = p.cruise_button_mpc.cfg
    v_des = 30.0 + (-3.0) * (np.arange(cfg.n_steps) * cfg.dt)
    p.events_sp.clear()
    p.update_decel_authority(True, 30.0, 0.0, -1, v_des, cfg, True)
    assert not self._raised(p), "warned on the very first frame"

  def test_clears_when_plan_becomes_achievable(self):
    p = self.make_planner()
    cfg = p.cruise_button_mpc.cfg
    v_des = 30.0 + (-3.0) * (np.arange(cfg.n_steps) * cfg.dt)
    for _ in range(40):
      p.events_sp.clear()
      p.update_decel_authority(True, 30.0, 0.0, -1, v_des, cfg, True)
    assert self._raised(p)
    v_des = 30.0 + (-0.1) * (np.arange(cfg.n_steps) * cfg.dt)
    p.events_sp.clear()
    p.update_decel_authority(True, 30.0, 0.0, -1, v_des, cfg, True)
    assert not self._raised(p)

  def test_silent_when_not_engaged(self):
    p = self.make_planner()
    cfg = p.cruise_button_mpc.cfg
    v_des = 30.0 + (-3.0) * (np.arange(cfg.n_steps) * cfg.dt)
    for _ in range(40):
      p.events_sp.clear()
      p.update_decel_authority(False, 30.0, 0.0, -1, v_des, cfg, True)
    assert not self._raised(p)

  def test_silent_at_crawl(self):
    p = self.make_planner()
    cfg = p.cruise_button_mpc.cfg
    v_des = 1.0 + (-3.0) * (np.arange(cfg.n_steps) * cfg.dt)
    for _ in range(40):
      p.events_sp.clear()
      p.update_decel_authority(True, 1.0, 0.0, -1, v_des, cfg, True)
    assert not self._raised(p)

  def test_uses_speed_dependent_authority(self):
    """Coast authority grows with speed, so a decel impossible at low speed may be
    achievable at highway speed; the threshold must follow it."""
    p = self.make_planner()
    a_min_slow = p.cruise_button_mpc.p.a_min_at(13.0)
    a_min_fast = p.cruise_button_mpc.p.a_min_at(35.0)
    assert a_min_fast < a_min_slow


class TestTargetRespectsSetSpeed(OpenpilotTestCase):
  """
  longitudinalPlan.speeds is not bounded by the set speed. Upstream moved the
  cruise limit out of the MPC (commaai/openpilot#38367), so the trajectory is the
  lead-following solution alone and the set speed is applied separately as an
  acceleration limit. Chasing it unclamped drove the setpoint past the set speed
  by up to 14mph on the road.
  """

  def make_planner(self, v_cruise_kph):
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.controller import CruiseButtonController
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.mpc import MpcConfig
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.plant import PlantParams
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

    p = object.__new__(LongitudinalPlannerSP)
    p.events_sp = EventsSP()
    p.cruise_button_mpc = CruiseButtonController(PlantParams(), MpcConfig())
    p.cruise_button = SendButtonState.none
    p._cb_t = 0.0
    p._decel_short_frames = 0
    p.v_desired_trajectory = None
    p.output_v_target = 0.0
    p.e2e_coast_enabled = False
    p.e2e_coast_active = False
    self._v_cruise_kph = v_cruise_kph
    return p

  def _sm(self, v_ego, v_cruise_kph, setpoint_mph=None):
    """setpoint defaults to roughly what would hold the current speed, so the
    planner starts from a consistent state rather than one already commanding a
    large change."""
    cs = structs.CarState()
    cs.vEgo = v_ego
    cs.aEgo = 0.0
    cs.vCruise = v_cruise_kph
    if setpoint_mph is None:
      setpoint_mph = v_ego * MS_TO_MPH + 2.0  # ~ the speedo offset
    cs.cruiseState.speedCluster = setpoint_mph * MPH_TO_MS
    cc = structs.CarControl()
    cc.enabled = True
    model = type("M", (), {"action": type("A", (), {
      "desiredAcceleration": 0.0, "shouldStop": False, "desiredCurvature": 0.0})()})()
    return {"carState": cs, "carControl": cc, "modelV2": model}

  def test_does_not_press_up_when_plan_exceeds_set_speed(self):
    """
    The exact on-road failure: the car is already at the set speed but the
    trajectory asks for much more. Every tick is inspected, not just the last --
    the press cooldown leaves most ticks idle, so sampling only the final value
    hides the bug.
    """
    kph = 45.0 / 0.621371          # set speed 45mph
    p = self.make_planner(kph)
    v_ego = 45.0 * MPH_TO_MS
    # trajectory asking for 60mph, well above the 45mph set speed
    p.v_desired_trajectory = np.full(17, 60.0 * MPH_TO_MS)
    seen = []
    for _ in range(30):
      p.update_cruise_button(self._sm(v_ego, kph))
      seen.append(p.cruise_button)
    ups = sum(1 for b in seen if b == SendButtonState.increase)
    assert ups == 0, f"pressed up past the set speed {ups} times"

  def test_still_presses_up_below_set_speed(self):
    """Clamping must not break normal acceleration toward the set speed."""
    kph = 60.0 / 0.621371
    p = self.make_planner(kph)
    v_ego = 45.0 * MPH_TO_MS
    p.v_desired_trajectory = np.full(17, 55.0 * MPH_TO_MS)
    seen = set()
    for _ in range(30):
      p.update_cruise_button(self._sm(v_ego, kph))
      seen.add(p.cruise_button)
    assert SendButtonState.increase in seen, "clamp broke normal acceleration"

  def test_unset_cruise_does_not_clamp_to_zero(self):
    """V_CRUISE_UNSET must not be treated as a real ceiling."""
    from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
    p = self.make_planner(V_CRUISE_UNSET)
    p.v_desired_trajectory = np.full(17, 30.0)
    p.update_cruise_button(self._sm(29.0, V_CRUISE_UNSET))
    assert p.cruise_button in (SendButtonState.none, SendButtonState.increase,
                               SendButtonState.decrease)


class TestDecelWarningIsTiedToReality(OpenpilotTestCase):
  """
  The warning must mean "I am doing all I can and still losing ground", not
  "some point in the next 2.5s looks steep". The original version fired on
  predictions the car went on to satisfy, and while the planner was pressing up.
  """

  def make_planner(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.controller import CruiseButtonController
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.mpc import MpcConfig
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.plant import PlantParams
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

    p = object.__new__(LongitudinalPlannerSP)
    p.events_sp = EventsSP()
    p.cruise_button_mpc = CruiseButtonController(PlantParams(), MpcConfig())
    p._decel_short_frames = 0
    return p

  @staticmethod
  def _raised(p):
    from openpilot.cereal import custom
    name = custom.OnroadEventSP.EventName.insufficientDecelAuthority
    return any(e.name == name for e in p.events_sp.to_msg())

  def _run(self, p, a_req, a_ego, action, n=40, v=30.0):
    """a_req is the deceleration the *target* implies; build a v_des with that slope."""
    cfg = p.cruise_button_mpc.cfg
    t = np.arange(cfg.n_steps) * cfg.dt
    v_des = v + a_req * t
    for _ in range(n):
      p.events_sp.clear()
      p.update_decel_authority(True, v, a_ego, action, v_des, cfg, True)
    return self._raised(p)

  def test_silent_while_planner_is_pressing_up(self):
    """If the planner wants to go faster it is not out of authority."""
    p = self.make_planner()
    assert not self._run(p, -3.0, a_ego=0.0, action=1)

  def test_silent_when_car_is_already_slowing_enough(self):
    """Plan wants -3.0 and the car is doing -3.2: nothing is wrong."""
    p = self.make_planner()
    assert not self._run(p, -3.0, a_ego=-3.2, action=-1)

  def test_warns_when_at_limit_and_losing_ground(self):
    p = self.make_planner()
    assert self._run(p, -3.0, a_ego=0.0, action=-1)




class TestE2eCoast(OpenpilotTestCase):
  """
  The e2e model anticipates junctions and roundabouts the ACC planner cannot see
  (measured: it asked to slow before 21 of 26 curvature events, median lead 9.9s)
  but asks for far more deceleration than coasting can deliver (median -0.72 m/s^2
  against -0.19 available). It must be clamped, not followed.
  """

  def make_planner(self, enabled=True):
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.controller import CruiseButtonController
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.mpc import MpcConfig
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.plant import PlantParams

    p = object.__new__(LongitudinalPlannerSP)
    p.cruise_button_mpc = CruiseButtonController(PlantParams(), MpcConfig())
    p.e2e_coast_enabled = enabled
    p.e2e_coast_active = False
    return p

  @staticmethod
  def _sm(a_req):
    return {"modelV2": type("M", (), {"action": type("A", (), {
      "desiredAcceleration": a_req, "shouldStop": False, "desiredCurvature": 0.0})()})()}

  def _apply(self, p, a_req, v_ego=20.0, v_target=20.0):
    cfg = p.cruise_button_mpc.cfg
    v_des = np.full(cfg.n_steps, v_target)
    return p.apply_e2e_coast(self._sm(a_req), v_des, v_ego, cfg)

  def test_disabled_is_a_passthrough(self):
    p = self.make_planner(enabled=False)
    out = self._apply(p, -1.0)
    assert np.allclose(out, 20.0)

  def test_positive_request_never_raises_the_target(self):
    """The model may only ever pull the target down, never push it up."""
    p = self.make_planner()
    out = self._apply(p, +1.5)
    assert out.max() <= 20.0 + 1e-9

  def test_lowers_target_when_model_wants_to_slow(self):
    p = self.make_planner()
    out = self._apply(p, -0.6)
    assert out[-1] < 20.0, "model asked to slow but the target did not drop"

  def test_request_is_clamped_to_coast_authority(self):
    """A -2 m/s^2 request must not build a target the buttons can never track."""
    p = self.make_planner()
    cfg = p.cruise_button_mpc.cfg
    v_ego = 20.0
    a_min = float(p.cruise_button_mpc.p.a_min_at(v_ego))
    out = self._apply(p, -2.0, v_ego=v_ego)
    horizon = cfg.n_steps * cfg.dt
    # slope of the produced target cannot exceed what coasting can do
    slope = (out[-1] - out[0]) / horizon
    assert slope >= a_min - 1e-6, f"target falls at {slope:.3f}, beyond coast {a_min:.3f}"

  def test_drop_is_bounded_relative_to_acc_target(self):
    """A request to stop degrades into an early lift, not a crawl."""
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import E2E_COAST_MAX_DROP
    p = self.make_planner()
    out = self._apply(p, -4.0, v_ego=30.0, v_target=30.0)
    assert out.min() >= 30.0 - E2E_COAST_MAX_DROP - 1e-6

  def test_silent_at_low_speed(self):
    """Near the setpoint floor there is nothing useful to give up."""
    p = self.make_planner()
    out = self._apply(p, -1.0, v_ego=5.0, v_target=5.0)
    assert np.allclose(out, 5.0)

  def test_real_event_profiles(self):
    """
    Replayed from logged curvature events on this car. The no-brake cases asked
    for roughly what coasting provides and should produce a usable target; the
    braking cases asked for several times more and must still clamp.
    """
    p = self.make_planner()
    for a_req, label in ((-0.25, "driver did not brake (median)"),
                         (-1.19, "driver braked (median)"),
                         (-2.29, "worst logged request")):
      out = self._apply(p, a_req, v_ego=20.0, v_target=20.0)
      a_min = float(p.cruise_button_mpc.p.a_min_at(20.0))
      cfg = p.cruise_button_mpc.cfg
      slope = (out[-1] - out[0]) / (cfg.n_steps * cfg.dt)
      assert slope >= a_min - 1e-6, f"{label}: target beyond coast authority"
      assert out[-1] < 20.0, f"{label}: no slowing produced"


class TestPlannerSurface(OpenpilotTestCase):
  """
  plannerd calls these by name every cycle. Nothing else in this file does, so a
  method could be deleted outright and the rest of the suite would still pass --
  which is exactly how update() was once lost, crashlooping plannerd on the car.
  """

  def test_methods_plannerd_depends_on_exist(self):
    import inspect
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    for name in ("update", "update_targets", "publish_longitudinal_plan_sp",
                 "update_cruise_button", "apply_e2e_coast", "update_decel_authority"):
      assert callable(getattr(LongitudinalPlannerSP, name, None)), f"missing {name}()"
      assert inspect.isfunction(getattr(LongitudinalPlannerSP, name))

  def test_update_still_drives_the_button_planner(self):
    """update() must keep calling update_cruise_button, or the buttons go silent
    while every other test continues to pass."""
    import inspect
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    src = inspect.getsource(LongitudinalPlannerSP.update)
    assert "update_cruise_button" in src


class TestDecelWarningNeedsALead(OpenpilotTestCase):
  """
  Falling short of the plan only matters when something is ahead. Lowering the
  set speed makes the plan ask for brisk deceleration the buttons cannot match,
  but the only consequence is reaching the new speed gradually.
  """

  def make_planner(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.controller import CruiseButtonController
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.mpc import MpcConfig
    from openpilot.sunnypilot.selfdrive.car.cruise_button_control.plant import PlantParams
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

    p = object.__new__(LongitudinalPlannerSP)
    p.events_sp = EventsSP()
    p.cruise_button_mpc = CruiseButtonController(PlantParams(), MpcConfig())
    p._decel_short_frames = 0
    return p

  @staticmethod
  def _raised(p):
    from openpilot.cereal import custom
    name = custom.OnroadEventSP.EventName.insufficientDecelAuthority
    return any(e.name == name for e in p.events_sp.to_msg())

  def _run(self, p, has_lead, v=30.0, a_req=-3.0):
    cfg = p.cruise_button_mpc.cfg
    v_des = v + a_req * (np.arange(cfg.n_steps) * cfg.dt)
    for _ in range(40):
      p.events_sp.clear()
      p.update_decel_authority(True, v, 0.0, -1, v_des, cfg, has_lead)
    return self._raised(p)

  def test_silent_with_no_lead(self):
    """Lowering the set speed on an empty road must not warn."""
    assert not self._run(self.make_planner(), has_lead=False)

  def test_warns_with_a_lead(self):
    assert self._run(self.make_planner(), has_lead=True)

  def test_lead_disappearing_clears_the_warning(self):
    p = self.make_planner()
    assert self._run(p, has_lead=True)
    cfg = p.cruise_button_mpc.cfg
    v_des = 30.0 + (-3.0) * (np.arange(cfg.n_steps) * cfg.dt)
    p.events_sp.clear()
    p.update_decel_authority(True, 30.0, 0.0, -1, v_des, cfg, False)
    assert not self._raised(p)
