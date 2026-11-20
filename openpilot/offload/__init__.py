"""openpilot/offload — comma 3X -> MacBook driving-model offload.

Package layout (see INTERFACES.md for the frozen contract):

  ports.py       FROZEN (PM)  ZMQ port scheme for cereal services (bridge.cc parity).
  contract.py    FROZEN (PM)  Shared dataclasses + service lists.
  INTERFACES.md  FROZEN (PM)  Workstream ownership, semantics, gates, decision tables.
  mac/           WS-B         Mac-side frame path (vtdec Swift, framebridge, frametable).
  models/        WS-A         Mac-compiled per-platform model artifacts.
  replay/        WS-C         Offline replay publisher + metrics + p1_gate.
  device/        WS-D         Device-side daemon, bench scripts, docs.
"""
