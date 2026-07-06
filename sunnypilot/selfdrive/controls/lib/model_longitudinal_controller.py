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


class ModelLongitudinalController:
  """ICBM source: adjusts cruise set-speed using the model's deceleration intent.

  When enabled (ICBMModelLong param), replaces the lead-follow P-controller.

  When the model is neutral or wants to accelerate (desiredAcceleration >= 0),
  outputs the car's actual reported cruise set-speed so ICBM holds steady and
  lets native cruise handle any acceleration to target.

  When the model wants to slow (desiredAcceleration < 0, e.g. lead car ahead or
  curve), outputs v_ego + a_model * DECEL_HORIZON — a projected lower speed —
  so ICBM presses decrease to bring cruise set-speed down.

  This avoids the feedback loop that arises from using v_desired_trajectory:
  that trajectory is set by the MPC with MLC's own output as v_cruise, so any
  value below v_cruise spirals the target down to v_ego each cycle.
  """

  DECEL_HORIZON = 5.0  # seconds to project deceleration intent forward

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
             prev_speeds=None) -> None:
    self.frame += 1
    self._update_params()

    self.is_enabled = self.enabled and long_enabled
    self.is_active = self.is_enabled and not long_override

    if self.is_active:
      a_model = sm['modelV2'].action.desiredAcceleration
      cruise_speed = sm['carState'].cruiseState.speed  # m/s, from car's CAN (e.g. LVR12 for non-SCC)

      if cruise_speed > 0 and a_model >= 0:
        # Model neutral or accelerating: hold at current cruise set-speed.
        # Native cruise handles any acceleration to the set target.
        self.output_v_target = cruise_speed
      else:
        # Model wants to slow down (or no cruise reference): project forward.
        # ICBM sees a target below current cruise and presses decrease.
        self.output_v_target = max(0.0, v_ego + a_model * self.DECEL_HORIZON)

      self.output_a_target = 0.0
    else:
      self.output_v_target = V_CRUISE_UNSET
      self.output_a_target = 0.0
