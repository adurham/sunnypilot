"""
fork (0045 v2, settlement Amendment 1): the superproject's FCA11 SHADOW post-send hook.

In FCA11 SHADOW (CP_SP.fca11Shadow, param HyundaiFca11Shadow) the wire carries the OLD-LAW (the currently deployed FCA11
law) exactly as today; V3 (the shaped candidate law) is computed + logged alongside and NEVER sent. card.py calls this hook
right AFTER it publishes sendcan, so the V3 computation runs strictly after the OLD-LAW frames of the frame are out: an
exception or a slow V3 step can never delay or alter them (V3-fault isolation). The opendbc side
(GasInterceptorCarController.fca11_post_send -> Fca11LongBrake.v3_post_send) already catches + logs every V3 failure; this
hook adds a last try/except so even a car-layer regression here can never take card's send loop down.

No-op for any CarController without ``fca11_post_send`` (every non-Hyundai-pedal car) and for Shadow OFF (nothing pending).
There is NO send filter here: dropping host 0x38D (the first shadow build's FCA11-DISABLED semantics) would remove the
OLD-LAW braking the owner has today.
"""


def fca11_shadow_post_send(CI) -> None:
  fn = getattr(getattr(CI, "CC", None), "fca11_post_send", None)
  if fn is None:
    return
  try:
    fn()
  except Exception:  # noqa: BLE001 - V3 is log-only; it must never break the send loop
    pass
