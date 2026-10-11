"""
fork (0045): the superproject's last gate for FCA11 SHADOW (log-only) mode.

In Shadow (CP_SP.fca11Shadow, param HyundaiFca11Shadow) the FCA11 car layer computes + logs its would-be output and NEVER
sends. The opendbc car layer already returns no host 0x38D in Shadow (Fca11LongBrake.update returns [] and the
gas_interceptor.py hand-off seam skips the extend). This filter is the belt-and-braces gate right before card.py publishes
sendcan: any host 0x38D still present in Shadow is DROPPED and counted (the count must stay 0; a non-zero count is a
car-layer regression, logged once per ignition). With Shadow OFF it returns the input list object unchanged (identity),
so the non-Shadow path is byte-for-byte untouched.
"""
from dataclasses import dataclass

FCA11_ADDR = 0x38D


@dataclass
class Fca11ShadowStats:
  dropped: int = 0
  warned: bool = False


def fca11_shadow_filter(CP_SP, can_sends, stats: Fca11ShadowStats | None = None):
  """Shadow ON -> can_sends without any 0x38D (dropped frames counted on `stats`); Shadow OFF -> can_sends itself."""
  if not bool(getattr(CP_SP, "fca11Shadow", False)):
    return can_sends
  kept = [m for m in can_sends if int(m[0] if isinstance(m, tuple) else m.address) != FCA11_ADDR]
  n = len(can_sends) - len(kept)
  if n and stats is not None:
    stats.dropped += n
    if not stats.warned:
      stats.warned = True
      try:
        from openpilot.common.swaglog import cloudlog
        cloudlog.error(f"fca11 shadow: dropped {n} host 0x38D frame(s) the car layer produced in Shadow (must be 0)")
      except Exception:  # noqa: BLE001 - logging must never break the send loop
        pass
  return kept
