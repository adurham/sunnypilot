#!/usr/bin/env bash
# ============================================================================
# 04_disable_tether.sh — USER-GATED: tear the tether back down.
#
# PURPOSE
#   Reverse of 01: stop adbd and clear AdbEnabled so the device AUX USB port returns to
#   its normal role and no offload networking persists.
#
# PREREQS  Run FROM the Mac. Device reachable; ssh key loaded.
# EXPECTED READINGS (after)
#   AdbEnabled == "" or "0";  systemctl is-active adbd == inactive/failed;  usb0 gone.
# USER-VISIBLE EFFECTS  (state CHANGE — user-gated)
#   * Stops the adbd systemd service; the gadget unbinds from UDC a600000.dwc3.
#   * Clears the AdbEnabled param (manager may briefly restart adbd-gated logic).
# ============================================================================
set -euo pipefail

HOST="${1:-comma@comma-b203ed6e.local}"
if [[ "$HOST" == -* || -z "$HOST" ]]; then
  echo "usage: bash 04_disable_tether.sh [user@]host" >&2; exit 2
fi
DEV_PY="/usr/local/venv/bin/python3"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "$HOST")

echo ">> [1/4] reachability: $HOST"
"${SSH[@]}" 'echo reachable' || { echo "ssh failed"; exit 1; }

echo ">> [2/4] stop the adbd gadget service"
if "${SSH[@]}" 'systemctl cat adbd.service >/dev/null 2>&1'; then
  "${SSH[@]}" 'sudo systemctl stop adbd' || echo "   WARN: stop failed (already stopped?)"
else
  echo "   no adbd.service — nothing to stop"
fi

echo ">> [3/4] clear AdbEnabled (USER-GATED param write)"
# NOTE: PYTHONPATH=/data/openpilot is required for `openpilot.*` imports on this fork.
"${SSH[@]}" "PYTHONPATH=/data/openpilot $DEV_PY - <<'PY'
from openpilot.common.params import Params
p = Params()
p.put_bool('AdbEnabled', False, block=True)
print('AdbEnabled ->', repr(p.get('AdbEnabled')))
PY"

echo ">> [4/4] verify teardown"
"${SSH[@]}" 'echo "AdbEnabled=[$(cat /data/params/d/AdbEnabled 2>/dev/null)]"; echo "adbd=$(systemctl is-active adbd 2>/dev/null)"; ip -o link show usb0 2>/dev/null || echo "usb0 gone (expected)"'
echo "DONE — tether disabled."
