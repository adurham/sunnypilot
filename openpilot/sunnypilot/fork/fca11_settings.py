"""
fork (0046): PURE visibility / enable rules for the consolidated Hyundai settings page.

The raylib page (openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py)
cannot run headless, so every decision the page makes about a row lives here, free of UI imports,
and is unit-tested in fork/tests/test_fca11_settings.py.

The page has one master selector (HyundaiLongitudinalMode, see fork/hyundai_settings.py) and three
kinds of rows:

  * path rows (pedal / FCA11 modifiers) are always listed and GREYED when their path is not selected,
    so the owner can see what exists without being misled into thinking it does anything;
  * experimental rows are hidden behind the "Show experimental" reveal, except while their own
    stored value is ON (a backup restore must never leave a live lane with no switch in sight);
  * the Stock tuning row is listed only in Stock mode, where it can actually reach the car.

Every row that changes a car value is enabled offroad only: the car layer reads these keys at
CarParams build time, so a change takes effect at the next ignition.
"""
from typing import NamedTuple

from openpilot.sunnypilot.fork.hyundai_settings import MODE_STOCK


class ToggleState(NamedTuple):
  visible: bool
  enabled: bool


def _on(raw) -> bool:
  """BOOL-param truthiness for a UI read: Params.get() returns a python bool for BOOL keys; tolerate
  "1"/b"1"/1.  None (missing key) is OFF, never an exception."""
  if isinstance(raw, bytes):
    raw = raw.decode(errors="ignore")
  return bool(raw is True or str(raw if raw is not None else "0").strip() == "1")


def mode_toggle_state(offroad: bool) -> ToggleState:
  """The master selector is always listed; settable offroad only."""
  return ToggleState(visible=True, enabled=bool(offroad))


def path_toggle_state(active: bool, offroad: bool) -> ToggleState:
  """A row whose path is not selected is still listed (the owner can see what the mode unlocks), but
  it is disabled so nothing looks live that is not."""
  return ToggleState(visible=True, enabled=bool(active and offroad))


def tuning_toggle_state(mode: int, alpha_available: bool, long_enabled: bool, offroad: bool) -> ToggleState:
  """The upstream tuning selector is inert while the pedal is on (scc_long is False and this car's
  DEFAULT tuning is the 0.50 s the interceptor already sets), so it is listed only in Stock mode."""
  visible = bool(alpha_available and mode == MODE_STOCK)
  return ToggleState(visible=visible, enabled=visible and bool(long_enabled and offroad))


def experimental_header_state(pedal: bool) -> ToggleState:
  """The experimental section exists only when a pedal path is selected: every row in it is inert
  without the pedal."""
  return ToggleState(visible=bool(pedal), enabled=bool(pedal))


def show_experimental_toggle_state(pedal: bool) -> ToggleState:
  """The reveal is a UI-only switch; it is available whenever the section is."""
  return ToggleState(visible=bool(pedal), enabled=bool(pedal))


def experimental_row_state(pedal: bool, prerequisite: bool, show: bool, own_raw, offroad: bool) -> ToggleState:
  """An experimental row is hidden until the reveal, but a stored ON is always shown: the keys are
  PERSISTENT | BACKUP and a restore can write them back with no switch in sight.  The row is enabled
  only when its prerequisites are met and the car is offroad."""
  visible = bool(pedal and prerequisite and (show or _on(own_raw)))
  return ToggleState(visible=visible, enabled=visible and bool(offroad))
