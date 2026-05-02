import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.widgets import Widget

T_GAP = 2.0
MIN_FOLLOW_DIST = 8.0

_LABEL_COLOR = rl.Color(255, 255, 255, 180)
_VALUE_COLOR = rl.WHITE
_ACTIVE_COLOR = rl.Color(80, 220, 80, 255)
_INACTIVE_COLOR = rl.Color(220, 220, 220, 160)
_PANEL_COLOR = rl.Color(0, 0, 0, 120)

_FONT_SIZE = 30
_LINE_HEIGHT = 38
_PAD = 16
_WIDTH = 240


class LeadFollowHUD(Widget):
  def __init__(self):
    super().__init__()
    self._enabled = False
    self._active = False
    self._v_ego = 0.0
    self._d_rel = 0.0
    self._v_lead = 0.0
    self._v_rel = 0.0
    self._v_target = 0.0
    self._model_prob = 0.0

  def _update_state(self):
    sm = ui_state.sm
    lf = sm['longitudinalPlanSP'].leadFollow
    self._enabled = lf.enabled
    self._active = lf.active
    self._v_target = lf.vTarget

    lead = sm['radarState'].leadOne
    self._d_rel = lead.dRel
    self._v_lead = lead.vLead
    self._v_rel = lead.vRel
    self._model_prob = lead.modelProb

    self._v_ego = sm['carState'].vEgo

  def _render(self, rect: rl.Rectangle):
    if not self._enabled:
      return

    desired_dist = max(self._v_ego * T_GAP, MIN_FOLLOW_DIST)
    dist_error = self._d_rel - desired_dist
    creep = self._v_target - self._v_lead if self._active else 0.0

    lines = [
      ("LF", "ON" if self._active else "IDLE", _ACTIVE_COLOR if self._active else _INACTIVE_COLOR),
      ("dist", f"{self._d_rel:.0f}m", _VALUE_COLOR),
      ("goal", f"{desired_dist:.0f}m", _VALUE_COLOR),
      ("err",  f"{dist_error:+.1f}m", _VALUE_COLOR),
      ("creep", f"{creep:+.1f}", _VALUE_COLOR),
      ("prob", f"{self._model_prob:.0%}", _VALUE_COLOR),
    ]

    height = _PAD * 2 + len(lines) * _LINE_HEIGHT
    x = rect.x + _PAD
    y = rect.y + rect.height - height - _PAD

    rl.draw_rectangle_rounded(rl.Rectangle(x, y, _WIDTH, height), 0.15, 6, _PANEL_COLOR)

    font = gui_app.font(FontWeight.ROMAN)
    for i, (label, value, color) in enumerate(lines):
      row_y = y + _PAD + i * _LINE_HEIGHT
      rl.draw_text_ex(font, label, rl.Vector2(x + _PAD, row_y), _FONT_SIZE, 0, _LABEL_COLOR)
      rl.draw_text_ex(font, value, rl.Vector2(x + _WIDTH - _PAD - rl.measure_text_ex(font, value, _FONT_SIZE, 0).x, row_y),
                      _FONT_SIZE, 0, color)
