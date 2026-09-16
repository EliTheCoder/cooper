"""
Press/release shaping for cruise buttons.

A cruise button used as an increment must be pressed and released. The planner's
request is a pulse train, not a level -- measured at 10Hz on/off on the road --
so the shaping has to run on wall-clock frames and latch the button, or each
press reaches the bus as a ~14ms tap that the BCM never latches. That failure
left an 11 second decrease request with the setpoint unmoved.
"""
import numpy as np

from openpilot.common.test import OpenpilotTestCase

PRESS_FRAMES = 15
RELEASE_FRAMES = 10


class Shaper:
  """Mirrors the state machine in opendbc/sunnypilot/car/hyundai/icbm.py update()."""

  def __init__(self):
    self.press_frames_left = 0
    self.release_frames_left = 0
    self.press_button = None

  def step(self, requested):
    requesting = requested is not None
    if self.release_frames_left > 0:
      self.release_frames_left -= 1
      self.press_button = None
    elif self.press_frames_left > 0:
      self.press_frames_left -= 1
      if self.press_frames_left == 0:
        self.release_frames_left = RELEASE_FRAMES
    elif requesting:
      self.press_frames_left = PRESS_FRAMES
      self.press_button = requested
    if self.press_button is not None and self.press_frames_left > 0:
      return self.press_button
    return None


def bursts(out):
  runs, cur = [], 0
  for x in out:
    if x is not None:
      cur += 1
    elif cur:
      runs.append(cur)
      cur = 0
  if cur:
    runs.append(cur)
  return runs


def run(pattern, n=1100):
  s = Shaper()
  return [s.step(pattern(f)) for f in range(n)]


class TestButtonPressShaping(OpenpilotTestCase):
  def test_continuous_request_gives_human_length_presses(self):
    b = bursts(run(lambda f: "dec"))
    assert b, "no presses at all"
    assert int(np.median(b)) == PRESS_FRAMES, f"median burst {np.median(b)} frames"

  def test_pulsed_request_gives_the_same_presses(self):
    """The real failure: a 10Hz on/off request used to emit 14ms taps."""
    b = bursts(run(lambda f: "dec" if (f // 5) % 2 == 0 else None))
    assert b, "no presses at all"
    assert int(np.median(b)) == PRESS_FRAMES, f"median burst {np.median(b)} frames"
    assert min(b) >= PRESS_FRAMES, f"a press was cut short: {min(b)} frames"

  def test_presses_are_separated_by_a_release(self):
    out = run(lambda f: "dec")
    gaps, cur = [], 0
    for x in out:
      if x is None:
        cur += 1
      elif cur:
        gaps.append(cur)
        cur = 0
    assert gaps, "button never released"
    assert min(gaps) >= RELEASE_FRAMES, f"release too short: {min(gaps)} frames"

  def test_press_rate_is_sane(self):
    """~4 presses/sec at 100Hz; fast enough to track, slow enough to be distinct."""
    b = bursts(run(lambda f: "dec"))
    rate = len(b) / 11.0
    assert 3.0 <= rate <= 5.0, f"{rate:.1f} presses/sec"

  def test_button_is_latched_for_the_whole_press(self):
    """A request that flips direction mid-press must not change the button."""
    s = Shaper()
    seen = []
    for f in range(PRESS_FRAMES):
      seen.append(s.step("dec" if f == 0 else "inc"))
    assert {x for x in seen if x} == {"dec"}, f"button changed mid-press: {set(seen)}"

  def test_no_output_when_nothing_requested(self):
    assert all(x is None for x in run(lambda f: None))
