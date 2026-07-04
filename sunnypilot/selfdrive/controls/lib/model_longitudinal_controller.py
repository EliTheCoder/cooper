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

# How far ahead (seconds) to project desiredAcceleration into a cruise target speed.
# Larger = more aggressive response: deeper cuts on braking, higher bumps on acceleration.
T_HORIZON = 2.0

# Low-pass filter time constant (seconds). Smooths rapid fluctuations in desiredAcceleration
# so ICBM doesn't button-hunt on every model frame.
FILTER_TAU = 1.5


class ModelLongitudinalController:
  """ICBM source: drives cruise set-speed from the longitudinal model's desiredAcceleration.

  When enabled (ICBMModelLong param), replaces the lead-follow P-controller. The vision
  model already accounts for lead cars, traffic, and curvature — this just translates its
  acceleration intent into a cruise target speed for ICBM.

    v_target = max(0, v_ego + filtered_accel * T_HORIZON)

  When the model is coasting (accel ≈ 0), v_target ≈ v_ego and ICBM does nothing.
  When the model wants to slow down, v_target < v_ego and ICBM decreases the set speed.
  The user's cruise set speed is always the upper bound (it's part of the min() in update_targets).
  """

  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.enabled = self.params.get_bool("ICBMModelLong")
    self.is_enabled = False
    self.is_active = False
    self.output_v_target: float = V_CRUISE_UNSET
    self.output_a_target: float = 0.0
    self._a_filtered: float = 0.0
    self._was_active: bool = False

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("ICBMModelLong")

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float) -> None:
    self.frame += 1
    self._update_params()

    self.is_enabled = self.enabled and long_enabled
    self.is_active = self.is_enabled and not long_override

    if self.is_active:
      a_model = sm['modelV2'].action.desiredAcceleration

      # Seed the filter from the live model value on first activation to avoid a step jump
      if not self._was_active:
        self._a_filtered = a_model

      alpha = DT_MDL / (DT_MDL + FILTER_TAU)
      self._a_filtered += alpha * (a_model - self._a_filtered)

      self.output_a_target = self._a_filtered
      self.output_v_target = max(0.0, v_ego + self._a_filtered * T_HORIZON)
    else:
      self._a_filtered = 0.0
      self.output_v_target = V_CRUISE_UNSET
      self.output_a_target = 0.0

    self._was_active = self.is_active
