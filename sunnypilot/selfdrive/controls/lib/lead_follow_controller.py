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
MIN_FOLLOW_DIST = 8.0  # metres — floor so we still maintain gap at very low speeds

# Proportional gain: how many m/s to adjust per metre of distance error.
# e.g. 20 m too far → +2 m/s above lead speed to close the gap.
DIST_GAIN = 0.1      # (m/s) / m

# Maximum speed adjustment in either direction from the lead's absolute speed.
# Keeps corrections gentle — ≈10 kph band.
MAX_SPEED_ADJ = 2.8  # m/s  (~10 kph / 6 mph)


class LeadFollowController:
  """ICBM source: adjust vTarget around the lead car's speed based on gap error.

  Computes a desired following distance (time-gap from ego speed), then offsets
  the lead's absolute speed proportionally to the distance error:

    speed_adj = clip(DIST_GAIN * (dRel - desired_dist), -MAX_SPEED_ADJ, +MAX_SPEED_ADJ)
    output_v_target = max(v_lead + speed_adj, 0)

  - Lead too close  → negative adj → target below lead speed → ICBM slows set speed
  - Lead too far    → positive adj → target above lead speed → ICBM raises set speed
  - Lead at ideal distance → adj ≈ 0 → target matches lead speed

  The min() in update_targets ensures this never raises the cruise set speed above
  the user's chosen limit, so it only speeds up to close a gap when there is room
  within the existing set speed.

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
      speed_adj = max(-MAX_SPEED_ADJ, min(MAX_SPEED_ADJ, DIST_GAIN * dist_error))
      self.output_v_target = max(lead.vLead + speed_adj, 0.0)
    else:
      self.output_v_target = V_CRUISE_UNSET

    self.frame += 1
