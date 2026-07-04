"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD


class ModelLongitudinalController:
  """ICBM source: drives cruise set-speed from the MPC planned speed trajectory.

  When enabled (ICBMModelLong param), replaces the lead-follow P-controller.
  Uses the previous cycle's MPC v_desired_trajectory[-1] (~1.6 s ahead) as the
  cruise target. The MPC already accounts for the model's intent, speed limits,
  and safety constraints, and naturally recovers to the user's set speed when
  the road clears — no manual filtering or time-horizon tuning required.

  Falls back to modelV2.action.desiredAcceleration integrated over 2 s if no
  trajectory is available (e.g. first frame).
  """

  FALLBACK_HORIZON = 2.0  # seconds, used only when no MPC trajectory is available

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.enabled = self.params.get_bool("ICBMModelLong")
    self.is_enabled = False
    self.is_active = False
    self.output_v_target: float = V_CRUISE_UNSET
    self.output_a_target: float = 0.0

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("ICBMModelLong")

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float,
             prev_speeds: np.ndarray | None = None) -> None:
    self.frame += 1
    self._update_params()

    self.is_enabled = self.enabled and long_enabled
    self.is_active = self.is_enabled and not long_override

    if self.is_active:
      if prev_speeds is not None and len(prev_speeds) > 0:
        # MPC planned speed ~1.6 s ahead — smooth, constrained, recovers naturally
        self.output_v_target = max(0.0, float(prev_speeds[-1]))
        self.output_a_target = 0.0
      else:
        # Fallback: integrate model's desired acceleration (first frame only)
        a_model = sm['modelV2'].action.desiredAcceleration
        self.output_v_target = max(0.0, v_ego + a_model * self.FALLBACK_HORIZON)
        self.output_a_target = a_model
    else:
      self.output_v_target = V_CRUISE_UNSET
      self.output_a_target = 0.0
