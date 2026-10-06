"""Hold CANCEL with cruise off to flip CooperCruise e2e/standard."""
from types import SimpleNamespace

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.car.cruise_button_control.mode_toggle import (
  ButtonType, CancelHoldToggle, CooperModeToggle, E2E_PARAM, EventNameSP,
)

HOLD = 100  # frames at 100Hz


def press(btn=ButtonType.cancel):
  return [SimpleNamespace(type=btn, pressed=True)]


def release(btn=ButtonType.cancel):
  return [SimpleNamespace(type=btn, pressed=False)]


def run(det, frames, events=(), cruise=False):
  """Feed `events` on the first frame, then `frames - 1` quiet frames. Returns fire count."""
  fired = int(det.update(list(events), cruise))
  for _ in range(frames - 1):
    fired += det.update([], cruise)
  return fired


class TestCancelHoldToggle(OpenpilotTestCase):
  def test_fires_once_at_one_second(self):
    det = CancelHoldToggle()
    self.assertEqual(det.hold_frames, HOLD)
    self.assertEqual(run(det, HOLD - 1, press()), 0)
    self.assertTrue(det.update([], False))
    # keep holding: no repeat
    self.assertEqual(run(det, 500), 0)

  def test_short_press_does_nothing(self):
    det = CancelHoldToggle()
    self.assertEqual(run(det, 50, press()), 0)
    self.assertEqual(run(det, 200, release()), 0)

  def test_each_hold_fires_again(self):
    det = CancelHoldToggle()
    self.assertEqual(run(det, HOLD, press()), 1)
    self.assertEqual(run(det, 10, release()), 0)
    self.assertEqual(run(det, HOLD, press()), 1)

  def test_cancelling_engaged_cruise_never_counts(self):
    """The normal use of CANCEL: press while engaged, cruise drops, thumb stays down."""
    det = CancelHoldToggle()
    self.assertEqual(run(det, 5, press(), cruise=True), 0)
    self.assertEqual(run(det, 300, cruise=False), 0)

  def test_cruise_engaging_mid_hold_aborts(self):
    det = CancelHoldToggle()
    self.assertEqual(run(det, 50, press()), 0)
    self.assertEqual(run(det, 1, cruise=True), 0)
    self.assertEqual(run(det, 300, cruise=False), 0)

  def test_other_buttons_ignored(self):
    det = CancelHoldToggle()
    self.assertEqual(run(det, 300, press(ButtonType.decelCruise)), 0)
    self.assertEqual(run(det, 300, press(ButtonType.accelCruise)), 0)


class FakeParams:
  def __init__(self, **vals):
    self.vals = dict(vals)

  def get_bool(self, k):
    return bool(self.vals.get(k, False))

  def put_bool(self, k, v):
    self.vals[k] = bool(v)


class FakeEvents:
  def __init__(self):
    self.names = []

  def add(self, name):
    self.names.append(name)


def cs(events=(), cruise=False):
  return SimpleNamespace(buttonEvents=list(events), cruiseState=SimpleNamespace(enabled=cruise))


class TestCooperModeToggle(OpenpilotTestCase):
  def hold(self, tog, events):
    tog.update(cs(press()), events)
    for _ in range(HOLD + 5):
      tog.update(cs(), events)
    tog.update(cs(release()), events)

  def test_flips_param_and_alerts_both_ways(self):
    params = FakeParams(CruiseButtonMpc=True)
    tog, events = CooperModeToggle(params), FakeEvents()

    self.hold(tog, events)
    self.assertTrue(params.vals[E2E_PARAM])
    self.assertEqual(events.names, [EventNameSP.cooperCruiseE2e])

    self.hold(tog, events)
    self.assertFalse(params.vals[E2E_PARAM])
    self.assertEqual(events.names, [EventNameSP.cooperCruiseE2e, EventNameSP.cooperCruiseStandard])

  def test_inert_without_cooper_cruise(self):
    params = FakeParams(CruiseButtonMpc=False)
    tog, events = CooperModeToggle(params), FakeEvents()
    self.hold(tog, events)
    self.assertNotIn(E2E_PARAM, params.vals)
    self.assertEqual(events.names, [])


class TestAlertsDefined(OpenpilotTestCase):
  def test_mode_alerts_show_with_cruise_off(self):
    """Made with cruise off, so they must be PERMANENT (WARNING isn't shown then)."""
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
    from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET
    for name in (EventNameSP.cooperCruiseE2e, EventNameSP.cooperCruiseStandard):
      self.assertIn(ET.PERMANENT, EVENTS_SP[name])
