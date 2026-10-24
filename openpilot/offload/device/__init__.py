"""openpilot.offload.device — WS-D device-side offload daemon and tooling.

Contents:
  offloadd.py       device-side republish daemon (P2 code-complete, P4-prep; not activated in P1-P4).
  check_device.sh   read-only ssh audit (run from the Mac).
  bench/            user-executed tether bring-up / smoke / teardown + P3 checklist.
  tests/            offline unit tests (runnable on the Mac, no device needed).

See ../INTERFACES.md (FROZEN) and ../contract.py (FROZEN) for the wire semantics this conforms to.
"""
