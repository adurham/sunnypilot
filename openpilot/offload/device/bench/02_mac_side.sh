#!/usr/bin/env bash
# ============================================================================
# 02_mac_side.sh — USER-GATED: find the Mac-side of the tether, show both ends,
#                  ping, and (if available) run an iperf3 throughput test.
#
# PURPOSE
#   After 01_enable_tether.sh the device exposes usb0; the Mac gets a new ethernet
#   interface. This script detects it, prints link-local addresses on both ends,
#   pings across the link, and runs iperf3 device-server / Mac-client.
#
# PREREQS
#   * 01_enable_tether.sh completed (device usb0 up, AdbEnabled=1, adbd active).
#   * Device HAS iperf3; Mac needs iperf3 for the throughput test (`brew install iperf3`).
#     If the Mac lacks it, the script still reports link + ping and skips throughput.
#
# EXPECTED READINGS
#   * New interface `enX` with a 169.254.x.x/16 link-local address (autoconf), NOT DHCP.
#   * `ping` RTT well under 1 ms across the direct link.
#   * iperf3 throughput >> narrowRoadEncodeData's ~10-12 Mbps.
#
# USER-VISIBLE EFFECTS  (state change — user-gated)
#   * Starts a one-shot iperf3 server ON the device for the duration of the test.
#   * No persistent writes on either side.
# ============================================================================
set -euo pipefail

HOST="${1:-comma@comma-b203ed6e.local}"
SECS="${2:-10}"
if [[ "$HOST" == -* || -z "$HOST" ]]; then
  echo "usage: bash 02_mac_side.sh [user@]host [iperf_seconds]" >&2; exit 2
fi
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "$HOST")

echo ">> [0/5] reachability: $HOST"
"${SSH[@]}" 'echo reachable' || { echo "ssh failed — load your ssh key (1Password) / check mDNS, or pass a host argument"; exit 1; }

echo ">> [1/5] device usb0 address"
DEV_IP="$("${SSH[@]}" "ip -4 -o addr show usb0 2>/dev/null | awk '{print \$4}' | cut -d/ -f1")"
[[ -n "$DEV_IP" ]] || { echo "device usb0 has no IPv4 yet — did 01_enable_tether.sh succeed?"; exit 1; }
echo "   device usb0 = $DEV_IP"

echo ">> [2/5] Mac interfaces (look for a new ethernet port with 169.254.x)"
ifconfig -l | tr ' ' '\n' | grep -E '^(en|bridge)' || true
echo "   --- hardware ports ---"
networksetup -listallhardwareports 2>/dev/null | grep -E 'Hardware Port|Device' || true

echo ">> [3/5] Mac link-local addresses (169.254.x on the new port)"
MAC_IP="$(ifconfig 2>/dev/null | awk '/inet 169\.254\./{print $2}' | head -1)"
if [[ -n "$MAC_IP" ]]; then echo "   Mac link-local = $MAC_IP"; else echo "   WARN: no 169.254.x on the Mac yet (cable? give it ~10 s for autoconf)"; fi

echo ">> [4/5] ping across the tether"
if [[ -n "$MAC_IP" ]]; then
  ping -c 5 -t 5 "$DEV_IP" || echo "   WARN: ping failed (check the interface, or device firewall)"
else
  ping -c 5 -t 5 "$DEV_IP" || echo "   WARN: ping to device link-local failed"
fi

echo ">> [5/5] iperf3 (device server one-shot -> Mac client)"
if command -v iperf3 >/dev/null 2>&1; then
  # start one-shot server on the device, backgrounded, then measure from the Mac
  "${SSH[@]}" "iperf3 -s -1 -D" >/dev/null 2>&1 || "${SSH[@]}" "nohup iperf3 -s -1 >/tmp/iperf3_srv.log 2>&1 &" >/dev/null 2>&1
  sleep 1
  iperf3 -c "$DEV_IP" -t "$SECS" -f m || echo "   WARN: iperf3 client failed (device server running?)"
else
  echo "   skip: iperf3 not on the Mac (brew install iperf3). Device has it."
fi
echo "OK — now run: bash 03_live_smoke.sh <device-ip>"
