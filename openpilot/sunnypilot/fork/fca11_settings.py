"""
fork (0044): PURE visibility / enable decisions for the Hyundai FCA11 settings toggles.

The raylib settings panel (openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py) cannot run
headless, so the decision lives here, free of UI imports, and is unit-tested in fork/tests/test_fca11_settings.py.

  FCA11 Braking (Beta)          HyundaiFca11Brake       visible: pedal on;                 enabled: offroad
  FCA11 Brake Shaping (Beta)    HyundaiFca11AffineGain  visible: pedal on AND braking on;  enabled: offroad
  FCA11 Shadow (Log-Only)       HyundaiFca11Shadow      visible: pedal on;                 enabled: offroad   (0045)

Both keys are read at CarParams build time, so a change takes effect at the NEXT ignition; offroad-only enabling makes
that consistent. The dependency (shaping needs braking) is ALSO enforced in the car layer (opendbc setup_interfaces), so
a persisted shaping ON with braking off is inert even if this UI rule were bypassed.
"""
from typing import NamedTuple


class ToggleState(NamedTuple):
  visible: bool
  enabled: bool


def _on(raw) -> bool:
  """BOOL-param truthiness for a UI read: Params.get() returns a python bool for BOOL keys; tolerate "1"/b"1"/1."""
  if isinstance(raw, bytes):
    raw = raw.decode(errors="ignore")
  return bool(raw is True or str(raw if raw is not None else "0").strip() == "1")


def fca11_brake_toggle_state(pedal_raw, offroad: bool) -> ToggleState:
  pedal_on = _on(pedal_raw)
  return ToggleState(visible=pedal_on, enabled=pedal_on and bool(offroad))


def fca11_shaping_toggle_state(pedal_raw, brake_raw, offroad: bool) -> ToggleState:
  visible = _on(pedal_raw) and _on(brake_raw)
  return ToggleState(visible=visible, enabled=visible and bool(offroad))


def fca11_shadow_toggle_state(pedal_raw, offroad: bool) -> ToggleState:
  """0045: FCA11 Shadow (Log-Only). Visible with the pedal on (independent of FCA11 Braking / Shaping: Shadow computes both
  laws regardless), settable offroad only (read at CarParams build time => next ignition)."""
  pedal_on = _on(pedal_raw)
  return ToggleState(visible=pedal_on, enabled=pedal_on and bool(offroad))
