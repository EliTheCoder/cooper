import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.selfdrive.ui.mici.onroad.torque_bar import arc_bar_pts
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.sunnypilot.selfdrive.car.cruise_button_control.plant import PlantParams

# the torque bar's arc, turned to run up the right edge of the camera view.
# the screen is shorter than it is wide, so the span is shorter too.
ACCEL_ANGLE_SPAN = 7.5


class AccelBar(Widget):
  """Longitudinal counterpart to the torque bar.

  The full length of the bar is the cruise control's authority at the current
  speed: the part above the center dot is the most it can accelerate, the part
  below is the most it can coast down. The fill is the measured acceleration as
  a fraction of that side, and thickens as it nears the limit.
  """

  def __init__(self, demo: bool = False, scale: float = 1.0):
    super().__init__()
    self._demo = demo
    self._scale = scale
    self._plant = PlantParams()
    self._accel_filter = FirstOrderFilter(0, 0.15, 1 / gui_app.target_fps)
    # share of the bar given to accel; the rest is decel. moves with speed
    self._split_filter = FirstOrderFilter(0.5, 0.5, 1 / gui_app.target_fps)
    self._alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def update_filter(self, value: float, split: float = 0.5):
    """Update the accel filter value (for demo mode)."""
    self._accel_filter.update(value)
    self._split_filter.update(split)

  def _active(self) -> bool:
    if self._demo:
      return True
    return ui_state.started and ui_state.cruise_button_mpc and ui_state.sm['carState'].cruiseState.enabled

  def _update_state(self):
    self._alpha_filter.update(float(self._active()))
    if self._demo:
      return

    car_state = ui_state.sm['carState']
    a_max = float(self._plant.a_max_at(car_state.vEgo))
    a_min = float(self._plant.a_min_at(car_state.vEgo))
    self._split_filter.update(a_max / (a_max - a_min))

    if not self._active():
      self._accel_filter.update(0.0)
      return

    a_ego = car_state.aEgo
    frac = a_ego / a_max if a_ego >= 0 else a_ego / -a_min
    self._accel_filter.update(float(np.clip(frac, -1, 1)))

  def _render(self, rect: rl.Rectangle) -> None:
    alpha = self._alpha_filter.x
    if alpha < 0.01:
      return

    x = self._accel_filter.x
    line_offset = np.interp(abs(x), [0.5, 1], [22 * self._scale, 26 * self._scale])
    line_height = np.interp(abs(x), [0.5, 1], [14 * self._scale, 56 * self._scale])

    bg_alpha = np.interp(abs(x), [0.5, 1.0], [0.25, 0.5])
    bg_color = rl.Color(255, 255, 255, int(255 * bg_alpha * alpha))

    # arc centered far off the right edge, so the bar bows toward the road.
    # angles grow clockwise on screen, so accel (up) is 180 + and decel (down) is 180 -
    line_radius = 1200 * self._scale
    left_angle = 180
    span = alpha * ACCEL_ANGLE_SPAN
    split = self._split_filter.x
    # the center dot sits where zero accel falls, so the bar stays centered as a whole
    zero_angle = left_angle - span * (split - 0.5)
    top_angle = zero_angle + span * split
    bottom_angle = zero_angle - span * (1 - split)
    mid_r = line_radius + line_height / 2

    cx = rect.x + rect.width + line_radius - line_offset
    cy = rect.y + rect.height / 2
    offset = np.array([cx, cy], dtype=np.float32)

    bg_pts = arc_bar_pts(mid_r, line_height, bottom_angle, top_angle, cap_radius=7 * self._scale) + offset
    draw_polygon(rect, bg_pts, color=bg_color)

    zero_y = cy + np.sin(np.radians(zero_angle)) * mid_r
    if x >= 0:
      a1 = zero_angle + span * split * x
      grad_end_y = zero_y * (1 - 0.65) + min(bg_pts[:, 1]) * 0.65
    else:
      a1 = zero_angle + span * (1 - split) * x
      grad_end_y = zero_y * (1 - 0.65) + max(bg_pts[:, 1]) * 0.65
    fill_pts = arc_bar_pts(mid_r, line_height, zero_angle, a1, cap_radius=7 * self._scale) + offset

    # fade to orange as we approach the limit, same as the torque bar
    near_limit = max(0, abs(x) - 0.75) * 4
    start_color = blend_colors(
      rl.Color(255, 255, 255, int(255 * 0.9 * alpha)),
      rl.Color(255, 200, 0, int(255 * alpha)),  # yellow
      near_limit,
    )
    end_color = blend_colors(
      rl.Color(255, 255, 255, int(255 * 0.9 * alpha)),
      rl.Color(255, 115, 0, int(255 * alpha)),  # orange
      near_limit,
    )

    # the shader reads gl_FragCoord, whose y runs up from the bottom of the screen.
    # the torque bar's gradient is horizontal and never sees this; ours is vertical
    def frag_y(y):
      return (gui_app.height - y - rect.y) / rect.height

    gradient = Gradient(
      start=(0, frag_y(zero_y)),
      end=(0, frag_y(grad_end_y)),
      colors=[start_color, end_color],
      stops=[0.0, 1.0],
    )
    draw_polygon(rect, fill_pts, gradient=gradient)

    # zero-accel dot
    if abs(x) < 0.5:
      dot_x = cx - np.cos(np.radians(zero_angle - left_angle)) * mid_r
      rl.draw_circle(int(dot_x), int(zero_y), (10 // 2 * self._scale),
                     rl.Color(182, 182, 182, int(255 * 0.9 * alpha)))
