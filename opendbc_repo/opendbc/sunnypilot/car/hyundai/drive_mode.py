"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fork (adurham): CN7 Elantra N drive-mode decode -> openpilot personality / longitudinal gate.

The cluster broadcasts the current drive mode on CLU13 (0x50C) byte 5 HIGH nibble (DBC signal
``CF_Clu_DriveMode``, 44|4, 10 Hz on C-CAN). The decode (car-features/0x4a0-decode.md, route 00000147;
cross-checked against 0x4A0 b2 bits 17-19, 0x2FC b7>>5 and MDPS11 CF_Mdps_CurrMode on 19/19 changes) is:

    1 = NORMAL   2 = ECO   3 = SPORT   6 = N-CUSTOM (slot in the CLU13 b5 low nibble)   7 = N

This module owns the two pure, offline-testable pieces of the feature:

* ``map_drive_mode``: one drive-mode value -> the action the superproject should take
  (a ``DriveModeResult``: an openpilot ``LongitudinalPersonality`` int for NORMAL/ECO/SPORT, ``BLOCK``
  for N / N-CUSTOM, or ``UNKNOWN`` for anything else / a signal that is absent).
* ``DriveModeDebouncer``: a one-frame debounce so a single-frame transient cannot change the mode.

Nothing here touches CAN, params, or selfdrived — it is deliberately pure so it can be unit-tested
without a car.
"""

from enum import IntEnum

# CLU13 CF_Clu_DriveMode raw values (10 Hz, C-CAN). 0 and every value outside this set mean
# "not decoded / absent / unknown" and must never force a personality or block engagement.
DRIVE_MODE_NORMAL = 1
DRIVE_MODE_ECO = 2
DRIVE_MODE_SPORT = 3
DRIVE_MODE_N_CUSTOM = 6
DRIVE_MODE_N = 7

DRIVE_MODE_SIGNAL = "CF_Clu_DriveMode"
DRIVE_MODE_MSG = "CLU13"

# One extra SOURCE frame of confirmation: a value must be seen on this many consecutive CLU13 frames
# before it is published. A single-frame transient is dropped; a real mode change (held for seconds)
# lands one frame (100 ms at 10 Hz) later. Counting source frames (not 100 Hz CarState ticks) is what
# makes this a true "one frame" debounce of the 10 Hz signal.
DRIVE_MODE_DEBOUNCE_FRAMES = 2


class DriveModeResult(IntEnum):
  """
  Outcome of mapping a drive-mode value.

  The non-negative members are exactly ``log.LongitudinalPersonality`` (aggressive=0, standard=1,
  relaxed=2), so a caller can do ``if result >= 0: personality = int(result)``. The negative members
  are the two non-personality outcomes.
  """
  BLOCK = -2       # N / N-CUSTOM: block openpilot longitudinal, leave lateral alone
  UNKNOWN = -1     # absent / unrecognized -> do nothing (never force, never block)
  AGGRESSIVE = 0   # SPORT
  STANDARD = 1     # NORMAL
  RELAXED = 2      # ECO


# NORMAL -> standard, ECO -> relaxed, SPORT -> aggressive. N / N-CUSTOM -> BLOCK. Everything else ->
# UNKNOWN. A plain table so the mapping is auditable at a glance.
DRIVE_MODE_TO_RESULT: dict[int, DriveModeResult] = {
  DRIVE_MODE_NORMAL: DriveModeResult.STANDARD,
  DRIVE_MODE_ECO: DriveModeResult.RELAXED,
  DRIVE_MODE_SPORT: DriveModeResult.AGGRESSIVE,
  DRIVE_MODE_N: DriveModeResult.BLOCK,
  DRIVE_MODE_N_CUSTOM: DriveModeResult.BLOCK,
}


def _as_mode(raw) -> int | None:
  """Strict int coercion: integral numbers only (1.0 ok, 1.5 / True / "1" / junk -> None)."""
  if isinstance(raw, bool):
    return None
  try:
    value = int(raw)
  except (TypeError, ValueError):
    return None
  return value if value == raw else None


def map_drive_mode(raw_mode: int) -> DriveModeResult:
  """
  Pure mapping: a raw ``CF_Clu_DriveMode`` value -> a ``DriveModeResult``.

  Unknown / absent (0, 4, 5, 8..15, junk, None-ish) -> ``UNKNOWN`` (the caller must do nothing). This
  function never mutates state and never returns a personality for a value it does not recognise, so
  the "unknown -> leave untouched" rule holds by construction.
  """
  value = _as_mode(raw_mode)
  if value is None:
    return DriveModeResult.UNKNOWN
  return DRIVE_MODE_TO_RESULT.get(value, DriveModeResult.UNKNOWN)


class DriveModeDebouncer:
  """
  One-source-frame debounce for the raw drive-mode value.

  ``update(frames)`` is fed the CLU13 frame values parsed since the last call (oldest..newest; an empty
  list when no frame arrived this tick) and returns the currently-published raw value. A new value is
  published only after it has been observed on ``hold_frames`` consecutive source frames, so a
  single-frame transient is dropped while a sustained change lands one frame late. The published value
  starts at ``unknown`` (0) until a value is confirmed.
  """

  def __init__(self, hold_frames: int = DRIVE_MODE_DEBOUNCE_FRAMES):
    self.hold_frames = max(1, int(hold_frames))
    self.unknown = 0
    self._candidate = self.unknown
    self._count = 0
    self._out = self.unknown

  def update(self, frames) -> int:
    if isinstance(frames, (int, float)):
      frames = [frames]
    for raw in frames:
      value = _as_mode(raw)
      if value is None:
        value = self.unknown

      if value == self._candidate:
        self._count += 1
      else:
        self._candidate = value
        self._count = 1

      if self._count >= self.hold_frames:
        self._out = self._candidate
    return self._out
