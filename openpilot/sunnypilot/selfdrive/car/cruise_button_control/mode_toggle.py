"""Hold CANCEL with cruise off to flip CooperCruise between e2e and standard long.

The car has no spare button openpilot can read -- the LKAS switch is wired into
the camera and never reaches the bus -- but CANCEL with cruise already off does
nothing on this car, and Hyundai with stock cruise never raises buttonCancel.

The press has to start with cruise disengaged. Cancelling an engaged cruise and
keeping the thumb on the button does not count, so the ordinary use of CANCEL can
never flip the mode by accident.
"""
from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.common.realtime import DT_CTRL

ButtonType = structs.CarState.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName

E2E_PARAM = "CruiseButtonE2eCoast"
HOLD_TIME = 1.0  # s


class CancelHoldToggle:
  def __init__(self, hold_time: float = HOLD_TIME, dt: float = DT_CTRL):
    self.hold_frames = max(1, int(round(hold_time / dt)))
    self.held_frames = 0
    self.armed = False
    self.fired = False

  def update(self, button_events, cruise_engaged: bool) -> bool:
    """Advance one control frame. True on the single frame the hold completes."""
    for be in button_events:
      if be.type != ButtonType.cancel:
        continue
      if be.pressed:
        self.armed = not cruise_engaged
        self.held_frames = 0
        self.fired = False
      else:
        self.armed = False

    if cruise_engaged:
      self.armed = False
    if not self.armed or self.fired:
      return False

    self.held_frames += 1
    if self.held_frames >= self.hold_frames:
      self.fired = True
      return True
    return False


class CooperModeToggle:
  """Owns the hold detector, the param flip and the alert for selfdrived."""

  def __init__(self, params):
    self.params = params
    self.enabled = params.get_bool("CruiseButtonMpc")
    self.detector = CancelHoldToggle()

  def update(self, CS: structs.CarState, events_sp) -> None:
    if not self.enabled:
      return
    if not self.detector.update(CS.buttonEvents, CS.cruiseState.enabled):
      return

    e2e = not self.params.get_bool(E2E_PARAM)
    self.params.put_bool(E2E_PARAM, e2e)
    events_sp.add(EventNameSP.cooperCruiseE2e if e2e else EventNameSP.cooperCruiseStandard)
