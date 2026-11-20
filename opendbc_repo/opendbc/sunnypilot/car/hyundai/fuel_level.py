"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Fork (adurham): CN7 Elantra N fuel level from CLU13 (0x50C).

KDMLink (github.com/531dev/kdmlink, CC BY-SA 4.0) validated the 16-bit LE fuel-sender word at CLU13
bits 24-39 on the Elantra N / Kona N sender family - pct = raw * 0.30201 - 612.17 - after the older
0x5CD D0 read proved to be a drive-frozen non-fuel byte. The same bits carry the legacy 10-bit
CF_Clu_DTE interpretation (distance to empty); both decode from the same bytes.

Verified against the owner's own routes (519 segments, 2026-10-13): the word behaves as a real
sender - 0.302 % per count, monotonic decrease, continuous across drives (24.8 -> 21.4 % over
routes 154-155, refuel, 97.3 -> 95.1 -> 91.2 % over 156-157). Two bus transients are visible:
  * a start burst on some ignitions (~6 s of raw 6194-6445 = +1258..+1334 %) - out of the validity
    band, dropped by the band check;
  * a single zero-payload frame at each drive end (raw 2048 = +6.3 %, IN band) - rejected by the
    tracker's step limit (runs of at most 3 frames observed; FUEL_CONFIRM_WINDOWS sightings of a
    jumped value are required before it is accepted).

Pure functions + a small pure tracker - no CAN, params or selfdrived imports - offline-testable.
"""

# CLU13 CF_Clu_FuelLevel (16-bit LE sender counts, 10 Hz on C-CAN).
FUEL_MSG = "CLU13"
FUEL_SIGNAL = "CF_Clu_FuelLevel"

FUEL_FACTOR = 0.30201
FUEL_OFFSET = -612.17

# Validity band in physical percent: the sender family is valid down to the -5 % intercept
# (KDMLink's ignoredRawValues boundary; raw >= 2011); anything above 150 % is junk.
FUEL_MIN_PCT = -5.0
FUEL_MAX_PCT = 150.0

# Tracker tuning: a level may move at most FUEL_STEP_MAX per update immediately (the sender steps
# 0.302 % at a time); a larger jump (refuel while powered, sender re-init) must repeat to
# FUEL_CONFIRM_TOL accuracy over FUEL_CONFIRM_WINDOWS sightings before it is accepted.
FUEL_STEP_MAX = 3.0
FUEL_CONFIRM_TOL = 0.5
FUEL_CONFIRM_WINDOWS = 10


def fuel_pct_from_counts(raw: int) -> float:
  """16-bit sender counts -> tank percentage. Out of the valid band -> 0.0 (no data)."""
  pct = raw * FUEL_FACTOR + FUEL_OFFSET
  if pct < FUEL_MIN_PCT or pct > FUEL_MAX_PCT:
    return 0.0
  return max(pct, 0.0)


def latest_fuel_pct(frames: list) -> float | None:
  """Tank percentage from the last VALID frame of a vl_all window (physical % - the parser applies
  the DBC factor). None = no valid frame in the window."""
  for v in reversed(frames):
    if FUEL_MIN_PCT <= v <= FUEL_MAX_PCT:
      return max(v, 0.0)
  return None


class FuelLevelTracker:
  """Holds the published level across CAN gaps and rejects transient outliers.

  * an empty window (no valid frame) leaves the level untouched - the sender is slow, holding is
    correct, and "no data" must never oscillate with a reading;
  * a within-FUEL_STEP_MAX move is accepted immediately (normal sender steps, <= 0.302 % per count);
  * anything else must be sighted consistently (FUEL_CONFIRM_TOL) FUEL_CONFIRM_WINDOWS times before
    it is accepted - this rejects the one-frame end-of-drive zero-payload frame (raw 2048 -> 6.3 %)
    while letting a real step change through in ~1 s of frames.
  """
  def __init__(self):
    self.level: float | None = None   # last accepted level; None = nothing accepted yet
    self._pending: float | None = None
    self._pending_count = 0

  def update(self, frames: list) -> float:
    """Feed one CANParser vl_all window; returns the level to publish (0.0 = none accepted yet)."""
    pct = latest_fuel_pct(frames)
    if pct is None:
      return self.level if self.level is not None else 0.0

    if self.level is None or abs(pct - self.level) <= FUEL_STEP_MAX:
      self.level = pct
      self._pending, self._pending_count = None, 0
    elif self._pending is not None and abs(pct - self._pending) <= FUEL_CONFIRM_TOL:
      self._pending_count += 1
      if self._pending_count >= FUEL_CONFIRM_WINDOWS:
        self.level = pct
        self._pending, self._pending_count = None, 0
    else:
      self._pending, self._pending_count = pct, 1

    return self.level if self.level is not None else 0.0
