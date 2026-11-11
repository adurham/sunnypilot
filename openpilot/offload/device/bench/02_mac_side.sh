#!/usr/bin/env bash
# ============================================================================
# 02_mac_side.sh — USER-GATED: bring up the Mac-side of the tether, show both ends,
#                  ping, and (if available) run an iperf3 throughput test.
#
# PURPOSE
#   After 01_enable_tether.sh the device exposes usb0. On macOS the NCM gadget binds
#   to a NEW ethernet interface (hardware port "Linux USB Gadget", e.g. en12) with the
#   driver (AppleUSBNCMControl/Data) attached — but macOS does NOT autoconfigure it
#   (no DHCP server on the device, and its autoconf does not come up on its own).
#   This script finds that interface, brings it UP, assigns link-local addresses on
#   BOTH ends, pings across the link, and runs iperf3.
#
# PREREQS
#   * 01_enable_tether.sh completed (device usb0 up, AdbEnabled=1, adbd active).
#   * AUX USB-C port cabled to the Mac (NOT OBD-C/panda).
#   * Mac needs sudo for `ifconfig` (it will prompt). Device sudo is `sudo -n`-capable.
#   * Mac needs iperf3 for the throughput test (`brew install iperf3`); skipped if absent.
#
# EXPECTED READINGS  (measured 2026-10-07, M4 Max <-> comma 3X)
#   * New interface with hardware port "Linux USB Gadget"; `status: active`,
#     `media: autoselect (100baseTX full-duplex)` — note 100BaseTX: the gadget comes up
#     at USB HIGH-SPEED (480 Mb/s = USB2), which is ample for our ~20 Mbps video budget.
#   * ping RTT < 1 ms (measured ~0.9 ms), 0% loss.
#   * iperf3 ~310-320 Mbit/s Mac<-device (measured 313). Jetlink measured 309-340 Mbit/s
#     for the same NCM link on a Jetson host, so this is the expected band.
#
# USER-VISIBLE EFFECTS  (state change — user-gated)
#   * Mac: `sudo ifconfig <iface> up` + assigns 169.254.60.2/16 on the new port.
#   * Device: `ip addr add 169.254.60.1/16 dev usb0` (idempotent).
#   * Starts a one-shot iperf3 server ON the device for the duration of the test.
#   * No persistent writes on either side; both address assignments vanish on unplug/reboot.
#
# REVERT:  sudo ifconfig <iface> down   ;  ssh <host> 'sudo ip addr flush dev usb0'
# ============================================================================
set -euo pipefail

HOST="${1:-comma@comma-b203ed6e.local}"
SECS="${2:-10}"
DEV_ADDR="169.254.60.1"
MAC_ADDR="169.254.60.2"
if [[ "$HOST" == -* || -z "$HOST" ]]; then
  echo "usage: bash 02_mac_side.sh [user@]host [iperf_seconds]" >&2; exit 2
fi
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "$HOST")

echo ">> [0/6] reachability: $HOST"
"${SSH[@]}" 'echo reachable' || { echo "ssh failed — load your ssh key (1Password) / check mDNS, or pass a host argument"; exit 1; }

echo ">> [1/6] device: bring usb0 up + assign $DEV_ADDR/16"
"${SSH[@]}" "sudo ip link set usb0 up; sudo ip addr add $DEV_ADDR/16 dev usb0 2>/dev/null || true; ip -4 -o addr show usb0 | awk '{print \"   device usb0 = \" \$4}'"

echo ">> [2/6] Mac: find the 'Linux USB Gadget' interface (NCM driver bound)"
GADGET_DEV="$(networksetup -listallhardwareports 2>/dev/null | awk '/Linux USB Gadget/{getline; print $2}' | head -1)"
if [[ -z "$GADGET_DEV" ]]; then
  echo "   WARN: no 'Linux USB Gadget' port found. Is the AUX cable in the Mac, and adbd running?"
  echo "   --- interfaces present ---"; ifconfig -l | tr ' ' '\n' | grep -E '^(en|bridge)' || true
  exit 1
fi
echo "   found: $GADGET_DEV"

echo ">> [3/6] Mac: bring it up + assign $MAC_ADDR/16  (sudo)"
sudo ifconfig "$GADGET_DEV" up
# macOS will not autoconfigure this port (no DHCP server device-side); assign explicitly.
sudo ifconfig "$GADGET_DEV" inet "$MAC_ADDR" netmask 255.255.0.0
sleep 2
ifconfig "$GADGET_DEV" | grep -E 'inet |status|media' | sed 's/^/   /'

echo ">> [4/6] ping across the tether ($MAC_ADDR -> $DEV_ADDR)"
if ping -c 5 -W 2000 "$DEV_ADDR"; then :; else echo "   WARN: ping failed (interface up? cable? device usb0 up?)"; fi

echo ">> [5/6] iperf3 (device server one-shot -> Mac client)"
if command -v iperf3 >/dev/null 2>&1; then
  "${SSH[@]}" "nohup iperf3 -s -1 >/tmp/iperf3s.log 2>&1 </dev/null & sleep 1; echo started" >/dev/null 2>&1 || true
  sleep 1
  iperf3 -c "$DEV_ADDR" -t "$SECS" -f m 2>&1 | tail -8 || echo "   WARN: iperf3 client failed (device server running?)"
else
  echo "   skip: iperf3 not on the Mac (brew install iperf3). Device has it."
fi

echo ">> [6/6] summary"
echo "   Mac:    $GADGET_DEV = $MAC_ADDR/16"
echo "   Device: usb0        = $DEV_ADDR/16   (over the AUX USB-C tether)"
echo "OK — framebridge target host is: $DEV_ADDR"
echo "     next: bash 03_live_smoke.sh $DEV_ADDR <seconds>      (needs the CAR ON for camera frames)"
echo "REVERT: sudo ifconfig $GADGET_DEV down ; ssh $HOST 'sudo ip addr flush dev usb0'"
