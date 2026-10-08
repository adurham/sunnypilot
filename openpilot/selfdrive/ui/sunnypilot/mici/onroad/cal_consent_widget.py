"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fork (adurham): FCA11-long CAL driver-consent prompt -- "tap to fire" (opendbc patch 0033).

0034 adds a STEERING-WHEEL path to the same consent (opendbc patch 0034): a double-press of the UP cruise
arrow fires the parked rep, a double-press of the DOWN arrow dismisses it. That path writes the SAME
``consent.json`` this widget writes, so the prompt below also shows a small "wheel: double-press" hint.
The touch path is unchanged -- the owner may still tap.

WHAT THIS IS
------------
The companion UI to the car-layer consent gate in
``opendbc/sunnypilot/car/hyundai/cal_mode.py``. The 0029/0031/0032 sequencer arms a cal rep on
engaged + no lead + near-straight + no rear-quarter blind spot; this widget makes the owner part of
that decision. When the sequencer parks a rep awaiting consent it writes
``/data/fca11-cal/pending.json``; this widget shows an unobtrusive prompt on the on-road view
("CAL REP READY -- 0.24 g / 60 km/h") with a clear TAP target and a small DISMISS target.

THE CHANNEL (and why a file, not a cereal field)
------------------------------------------------
The car layer and the UI talk over TWO small JSON files under ``/data/fca11-cal/``:

  * ``pending.json``  -- written by the sequencer (ADVISORY). Read here, throttled, to show the prompt.
  * ``consent.json``  -- written HERE on a tap: ``{rep_id, ts, accept}``. The sequencer consumes it and
    acts ONLY if it is fresh (<= 2 s), bound to the pending rep_id, and accepted.

This mirrors the existing cal design (plan.json / progress.json / reps.jsonl) and keeps the WHOLE
fail-closed argument in one auditable place (cal_mode.py + /data/fca11-cal/). A cereal/CarStateSP
field would couple custom.capnp + opendbc structs + card.py + helpers.py into a schema change for a
signal that is UI-only, per-prompt, and ephemeral. The tap itself must be an explicit file write
anyway (it is what the car layer consumes); a cereal message would only replace the ADVISORY half.

FAIL-CLOSED
-----------
This widget NEVER actuates anything: it only writes a request. The authority is entirely the
sequencer, which refuses to fire on an absent / malformed / stale / mismatched / non-accepted
request, and RE-VERIFIES every condition at the instant of the tap. A bug here can therefore at
worst fail to prompt, never fire a brake.

NON-BLOCKING
------------
Rendering must never block. State is a tiny throttle-gated file poll (5 Hz, best-effort, wrapped in
OSError/ValueError); rendering is pure draw calls. This widget is INVISIBLE and processes NO touch
events unless a rep is genuinely pending. It only ever acts on a tap inside its own button rects, so
it cannot steal the bookmark swipe (which the road view dispatches first, and which needs a >50 px
drag anyway).
"""
import json
import os
import time
from enum import IntEnum

import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.ui_state import ui_state

KPH_TO_MPH = 0.621371

CAL_DIR = os.environ.get("FCA11_CAL_DIR", "/data/fca11-cal")
POLL_INTERVAL_S = 0.2      # re-stat the two tiny files at 5 Hz -- never every frame
FIRING_SHOW_S = 1.2        # keep the "firing..." confirmation up briefly
PILL_H = 150
BUTTON_H = 108
BUTTON_W = 300
DISMISS_W = 96


class CalConsentState(IntEnum):
  HIDDEN = 0
  PENDING = 1
  FIRING = 2


class CalConsentWidget(Widget):
  """The on-road consent prompt. Render-only; the ONLY write it makes is ``consent.json``."""

  def __init__(self, cal_dir: str = CAL_DIR):
    super().__init__()
    self.dir = cal_dir
    self._pending_path = os.path.join(cal_dir, "pending.json")
    self._consent_path = os.path.join(cal_dir, "consent.json")

    self._state = CalConsentState.HIDDEN
    self._rep_id = None
    self._level_lsb = 0
    self._level_g = 0.0
    self._band_kph = 0.0
    self._seq = -1
    self._plan_id = ""
    self._last_poll = -1e9
    self._firing_until = -1e9

    # touch: which target the current press started in ("tap" / "dismiss" / None)
    self._press_target = None
    self._interacting = False   # mirrors BookmarkIcon.interacting(): a press is in flight here
    # Fonts are resolved LAZILY at first render (the GL window exists then; constructing the widget
    # must not require it, so the widget stays importable / unit-testable off-device).
    self._font = None

  def _get_font(self):
    if self._font is None:
      self._font = gui_app.font(FontWeight.SEMI_BOLD)
    return self._font

  # ------------------------------------------------------------------ state
  def is_prompting(self) -> bool:
    return self._state in (CalConsentState.PENDING, CalConsentState.FIRING)

  def interacting(self) -> bool:
    """Consume-once, like BookmarkIcon.interacting(): the road view uses this to guard its own click."""
    interacting, self._interacting = self._interacting, False
    return interacting

  def _read_pending(self):
    try:
      with open(self._pending_path) as f:
        obj = json.load(f)
    except (OSError, ValueError):
      return None
    return obj if isinstance(obj, dict) else None

  def update(self) -> None:
    """Throttled best-effort poll. Called once per render from the road view."""
    now = time.monotonic()
    if self._state == CalConsentState.FIRING:
      if now >= self._firing_until:
        self._state = CalConsentState.HIDDEN
        self._rep_id = None
      return
    if (now - self._last_poll) < POLL_INTERVAL_S:
      return
    self._last_poll = now
    doc = self._read_pending()
    if doc is None or not bool(doc.get("pending", False)):
      self._state = CalConsentState.HIDDEN
      self._rep_id = None
      return
    # a pending prompt for a rep we have not seen before: show it (fresh, from the top)
    if (self._state != CalConsentState.PENDING or doc.get("rep_id") != self._rep_id
        or doc.get("seq") != self._seq):
      self._state = CalConsentState.PENDING
      self._rep_id = doc.get("rep_id")
      self._seq = doc.get("seq")
    self._level_lsb = int(doc.get("level_lsb", 0) or 0)
    self._level_g = float(doc.get("level_g", 0.0) or 0.0)
    self._band_kph = float(doc.get("band_kph", 0.0) or 0.0)
    self._seq = doc.get("seq")
    self._plan_id = doc.get("plan_id", "")

  def _write_consent(self, accept: bool) -> None:
    """Write the tap. Atomic (tmp + os.replace). A failed write simply means no consent is
    recorded and the rep stays pending/expires -- never actuates. Binds to the EXACT prompt the owner
    saw (rep_id + plan_id + seq), so a tap for an older prompt of the same rep cannot fire a newer one."""
    doc = {"rep_id": self._rep_id, "plan_id": self._plan_id, "seq": self._seq,
           "ts": time.time(), "accept": bool(accept)}
    tmp = self._consent_path + ".tmp"
    try:
      os.makedirs(self.dir, exist_ok=True)
      with open(tmp, "w") as f:
        json.dump(doc, f, sort_keys=True)
      os.replace(tmp, self._consent_path)
    except OSError:
      pass

  # ------------------------------------------------------------------ geometry
  def _pill_rect(self) -> rl.Rectangle:
    w = max(360.0, min(self._rect.width * 0.62, 900.0))
    x = self._rect.x + (self._rect.width - w) / 2.0
    y = self._rect.y + self._rect.height - PILL_H - self._rect.height * 0.16
    return rl.Rectangle(x, y, w, PILL_H)

  def _tap_rect(self) -> rl.Rectangle:
    p = self._pill_rect()
    return rl.Rectangle(p.x + p.width - BUTTON_W - 16, p.y + (p.height - BUTTON_H) / 2.0, BUTTON_W, BUTTON_H)

  def _dismiss_rect(self) -> rl.Rectangle:
    p = self._pill_rect()
    return rl.Rectangle(p.x + 10, p.y + (p.height - DISMISS_W) / 2.0, DISMISS_W, DISMISS_W)

  # ------------------------------------------------------------------ touch
  def _handle_mouse_event(self, mouse_event) -> None:
    if self._state != CalConsentState.PENDING:
      return
    if mouse_event.left_pressed:
      if rl.check_collision_point_rec(mouse_event.pos, self._tap_rect()):
        self._press_target = "tap"
      elif rl.check_collision_point_rec(mouse_event.pos, self._dismiss_rect()):
        self._press_target = "dismiss"
      else:
        self._press_target = None
      if self._press_target is not None:
        self._interacting = True
    elif mouse_event.left_released:
      tgt = self._press_target
      self._press_target = None
      if tgt == "tap" and rl.check_collision_point_rec(mouse_event.pos, self._tap_rect()):
        self._write_consent(True)
        self._state = CalConsentState.FIRING
        self._firing_until = time.monotonic() + FIRING_SHOW_S
      elif tgt == "dismiss" and rl.check_collision_point_rec(mouse_event.pos, self._dismiss_rect()):
        self._write_consent(False)
        self._state = CalConsentState.HIDDEN
        self._rep_id = None

  # ------------------------------------------------------------------ render
  def _render(self, _):
    if self._state == CalConsentState.HIDDEN:
      return
    font = self._get_font()
    p = self._pill_rect()
    rl.draw_rectangle_rounded(p, 0.28, 8, rl.Color(12, 12, 14, 224))
    rl.draw_rectangle_rounded_lines_ex(p, 0.28, 8, rl.Color(255, 255, 255, 90))

    level_txt = f"{self._level_g:.2f} g"
    # fork: show the owner's own units -- the prompt is for HIM to approve, and this car is imperial.
    band_txt = (f"{self._band_kph:.0f} km/h" if ui_state.is_metric
                else f"{self._band_kph * KPH_TO_MPH:.0f} mph")
    msg = "CAL REP READY"
    sub = f"{level_txt} / {band_txt}"
    text_x = p.x + DISMISS_W + 26
    rl.draw_text_ex(font, msg, rl.Vector2(text_x, p.y + 30), 44, 0, rl.Color(255, 255, 255, 235))
    rl.draw_text_ex(font, sub, rl.Vector2(text_x, p.y + 82), 40, 0, rl.Color(200, 200, 205, 220))
    # 0034: the STEERING-WHEEL hint. The touch path is unchanged (the TAP button below still fires); this
    # only tells the owner the wheel gesture exists. Rendered in the pill's left region, clear of the TAP
    # button, in a smaller font so it never competes with the level/band readout.
    rl.draw_text_ex(font, "wheel: double-press  \u25b2 / \u25bc", rl.Vector2(text_x, p.y + 120), 24, 0,
                    rl.Color(170, 175, 185, 210))

    if self._state == CalConsentState.FIRING:
      firing = "firing..."
      fw = measure_text_cached(font, firing, 44).x
      bx = p.x + p.width - 40 - fw
      rl.draw_text_ex(font, firing, rl.Vector2(bx, p.y + (p.height - 44) / 2.0), 44, 0,
                      rl.Color(120, 220, 150, 235))
      return

    # TAP target
    b = self._tap_rect()
    pressed = self.is_pressed and rl.check_collision_point_rec(rl.get_mouse_position(), b)
    rl.draw_rectangle_rounded(b, 0.35, 6, rl.Color(48, 73, 244, 255) if pressed else rl.Color(70, 91, 234, 255))
    tap = "TAP TO FIRE"
    tw = measure_text_cached(font, tap, 40).x
    rl.draw_text_ex(font, tap, rl.Vector2(b.x + (b.width - tw) / 2.0, b.y + (b.height - 40) / 2.0),
                    40, 0, rl.WHITE)

    # DISMISS target
    d = self._dismiss_rect()
    rl.draw_rectangle_rounded(d, 0.35, 6, rl.Color(52, 52, 56, 235))
    x = "✕"
    xw = measure_text_cached(font, x, 44).x
    rl.draw_text_ex(font, x, rl.Vector2(d.x + (d.width - xw) / 2.0, d.y + (d.height - 44) / 2.0),
                    44, 0, rl.Color(235, 235, 235, 235))
