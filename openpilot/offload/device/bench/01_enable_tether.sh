#!/usr/bin/env bash
# ============================================================================
# 01_enable_tether.sh — USER-GATED: bring up the USB tether on the comma device.
#
# PURPOSE
#   Enable the NCM+FunctionFS USB gadget on the device so a network interface (usb0)
#   appears on the Mac, giving a direct link for the offload data path.
#
# PREREQS
#   * Run FROM the Mac. Device reachable via mDNS, ssh key loaded.
#   * USB-C cable in the device AUX port (USB 3.1 Gen2, UDC a600000.dwc3) — NOT the
#     OBD-C/panda port.
#   * The device's own python (`/usr/local/venv/bin/python3`) has openpilot Params.
#
# EXPECTED READINGS (after)
#   AdbEnabled == "1";  systemctl is-active adbd == active;  `ip -o link show usb0` shows usb0.
#   On the Mac: a new ethernet-style interface with a 169.254.x.x link-local address.
#
# USER-VISIBLE EFFECTS  (this is a state CHANGE — user-gated)
#   * Writes the AdbEnabled param on the device (< 5 s onroad disruption: manager may notice).
#   * Starts the systemd `adbd` service (builds the gadget, binds UDC a600000.dwc3).
#   * The device AUX USB port switches from storage/other gadget to a network interface.
#   * Reverse with 04_disable_tether.sh.
#
# DEVICE CAPTURE (this session; re-verify with `ssh <host> cat /usr/comma/set_adb.sh` when reachable)
#   /usr/comma/set_adb.sh does roughly:  mkdir -p /config/usb_gadget/g1 ; (configfs boilerplate
#   creating the NCM function + FunctionFS for adb) ; echo a600000.dwc3 > /config/usb_gadget/g1/UDC
#   It is normally invoked by the `adbd` systemd unit, gated on AdbEnabled=1.
# ============================================================================
set -euo pipefail

HOST="${1:-comma@comma-b203ed6e.local}"
if [[ "$HOST" == -* || -z "$HOST" ]]; then
  echo "usage: bash 01_enable_tether.sh [user@]host" >&2; exit 2
fi

DEV_PY="/usr/local/venv/bin/python3"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "$HOST")

echo ">> [1/4] reachability: $HOST"
"${SSH[@]}" 'echo reachable: $(uname -sr)' || { echo "ssh failed — load your key / check mDNS"; exit 1; }

echo ">> [2/4] set AdbEnabled=1 (USER-GATED param write)"
# NOTE: the device's fork requires PYTHONPATH=/data/openpilot for `openpilot.*` imports
# (see comma-car-bridge skill: import path is openpilot.cereal.messaging, not cereal.messaging).
"${SSH[@]}" "PYTHONPATH=/data/openpilot $DEV_PY - <<'PY'
from openpilot.common.params import Params
p = Params()
p.put_bool('AdbEnabled', True, block=True)
print('AdbEnabled ->', p.get('AdbEnabled'))
PY"

echo ">> [3/4] start the adbd gadget service (USER-GATED)"
# Prefer the systemd unit; fall back to the vendor script if the unit is absent.
if "${SSH[@]}" 'systemctl list-unit-files adbd.service >/dev/null 2>&1 && systemctl cat adbd.service >/dev/null 2>&1'; then
  "${SSH[@]}" 'sudo systemctl start adbd'
else
  echo "   adbd.service not found; falling back to /usr/comma/set_adb.sh (verify it exists first)"
  "${SSH[@]}" 'test -x /usr/comma/set_adb.sh && sudo /usr/comma/set_adb.sh || { echo "no adbd unit and no set_adb.sh" >&2; exit 3; }'
fi

echo ">> [4/4] verify"
"${SSH[@]}" 'echo "AdbEnabled=$(cat /data/params/d/AdbEnabled 2>/dev/null)"; echo "adbd=$(systemctl is-active adbd 2>/dev/null)"; ip -o link show usb0 2>/dev/null || echo "usb0 not up yet (may take a few seconds)"'
echo "OK — now run: bash 02_mac_side.sh   (find the Mac-side link-local interface and ping/iperf)"
