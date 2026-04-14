"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

# Desired following gap expressed as a time headway.
# desired_dist = max(v_ego * T_GAP, MIN_FOLLOW_DIST)
T_GAP = 2.0          # seconds
MIN_FOLLOW_DIST = 8.0  # meters — floor so we still maintain gap at very low speeds

# PD controller gains.
# P: speed adjustment per meter of distance error.
KP = 0.1   # (m/s) / m
# D: speed adjustment based on relative velocity (vRel = d(dRel)/dt).
#    Negative vRel means the gap is closing — D term reacts before the distance
#    error grows large.
KD = 0.3   # dimensionless (vRel is already in m/s)

# Asymmetric speed adjustment limits.
# Closing the gap (lead far): small cap to avoid aggressive acceleration toward the lead.
# Opening the gap (lead close): larger cap to shed speed quickly when following too closely.
MAX_SPEED_ADJ_CLOSE =  0.5  # m/s — max increase above lead speed to close a distant gap
MAX_SPEED_ADJ_OPEN  =  2.8  # m/s (~10 kph) — max decrease below lead speed to open a close gap


class LeadFollowController:
  """ICBM source: PD controller that adjusts vTarget around the lead car's speed.

  Computes a desired following distance (time-gap from ego speed), then uses a
  PD controller on the distance error to offset the lead's absolute speed:

    speed_adj = clip(KP * (dRel - desired_dist) + KD * vRel, -MAX_SPEED_ADJ_OPEN, +MAX_SPEED_ADJ_CLOSE)
    output_v_target = max(v_lead + speed_adj, 0)

  The D term uses vRel (= d(dRel)/dt) directly, so the controller reacts to a
  closing or opening gap before the distance error grows large.

  No I term: ICBM itself acts as an integrator (continuously pressing buttons
  until the set speed reaches v_target), so adding I here would double-integrate.

  The positive cap is intentionally small — the controller mostly acts as a speed
  limiter that follows the lead down, with only a gentle nudge to recover distance
  when the lead pulls ahead.

  When no lead is present output_v_target is V_CRUISE_UNSET (no constraint).
  """

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.enabled = self.params.get_bool("ICBMLeadFollow")
    self.is_enabled = False
    self.is_active = False
    self.output_v_target: float = V_CRUISE_UNSET
    self.output_a_target: float = 0.0

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("ICBMLeadFollow")

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float) -> None:
    self._update_params()

    self.is_enabled = self.enabled and long_enabled

    lead = sm['radarState'].leadOne
    self.is_active = self.is_enabled and lead.status and not long_override

    if self.is_active:
      desired_dist = max(v_ego * T_GAP, MIN_FOLLOW_DIST)
      dist_error = lead.dRel - desired_dist
      speed_adj = max(-MAX_SPEED_ADJ_OPEN, min(MAX_SPEED_ADJ_CLOSE, KP * dist_error + KD * lead.vRel))
      self.output_v_target = max(lead.vLead + speed_adj, 0.0)
    else:
      self.output_v_target = V_CRUISE_UNSET

    self.frame += 1
