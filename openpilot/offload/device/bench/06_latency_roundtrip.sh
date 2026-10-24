#!/usr/bin/env bash
# ============================================================================
# 06_latency_roundtrip.sh — USER-GATED: measure the tether ROUND-TRIP latency
#                           distribution, in BOTH directions, at the sizes our
#                           return path and Jetlink's INFER_RESP actually use.
#
# WHY
#   Our return path is deferred, and its viability hinges on a number we have never measured:
#   the NCM+TCP round-trip p99/p99.9. Jetlink's 46 ms hold rule (model_state.py:HOLD_FRAME) was
#   tuned against a RAW BULK USB3 link whose p99 was 45.7 ms (mac-performance.md:57-61) — right
#   at the boundary — carrying 393 KB up + 8 KB down. Ours is NCM + TCP over the same cable,
#   carrying ~8 KB. Absolute RTT should be far lower than theirs (we move 50x fewer bytes), but
#   NCM+TCP has fatter tails than raw bulk; the 46 ms rule is a TAIL rule, so p99.9/max decide.
#
#   Sizes: 8192 B (our return-path payload class) and 8324 B (Jetlink INFER_RESP, the model
#   output vector), plus 256 B as a floor (pure per-trip overhead, no serialization cost).
#
# HOW
#   Scp's device/bench/echo_server.py to the device's /tmp (stdlib TCP echo; no /data write) and
#   runs two passes, each N round trips per size:
#     A) Mac drives, DEVICE echoes  -> measures the device's TCP RX + TX path under our client.
#     B) DEVICE drives, Mac echoes  -> measures the Mac-hosted path the device would see.
#   Both are the same round-trip clock; comparing them isolates direction-specific stalls.
#
# USAGE
#   bash 06_latency_roundtrip.sh [user@]host [n] [sizes]
#     n      round trips per size per direction (default 2000)
#     sizes  comma-separated bytes                 (default 256,8192,8324)
#   Override the detected addresses with OFFLOAD_DEV_IP / OFFLOAD_MAC_IP if mDNS/usb0 detection
#   is wrong.
#
# EXPECTED READINGS
#   * Local loopback sanity (same script, both ends local): p50 ~0.03 ms, p99.9 < 0.15 ms —
#     proves the harness itself is not the tail.
#   * Ours, NCM+TCP ~8 KB RTT: expect p50 low single-digit ms; the DECISION number is p99.9 and
#     max. If p99.9 sits well under 46 ms there is headroom for the HOLD rule; a p99.9 at or over
#     46 ms means a held frame is a routine event and the return path is NOT viable as-is.
#   * Compare against Jetlink's raw-bulk USB3 p99 45.7 ms — read ours as a TALLER-or-shorter TAIL,
#     not as the same measurement (different transport, different payload).
#
# PREREQS  Run FROM the Mac. 01+02 done (tether up), ssh key loaded. Run OFF-drive.
# USER-VISIBLE EFFECTS  (state change — user-gated)
#   * Copies echo_server.py to device /tmp; starts a short-lived echo server on each end for
#     the duration of each pass and kills it after. No /data writes, no persistent state.
# ============================================================================
set -euo pipefail

HOST="${1:-comma@comma-b203ed6e.local}"
N="${2:-2000}"
SIZES="${3:-256,8192,8324}"
if [[ "$HOST" == -* || -z "$HOST" ]]; then
  echo "usage: bash 06_latency_roundtrip.sh [user@]host [n] [sizes]" >&2; exit 2
fi

DEV_PY="/usr/local/venv/bin/python3"
DEV_ECHO="/tmp/offload_echo.py"
LOCAL_ECHO="openpilot/offload/device/bench/echo_server.py"
PY=".venv/bin/python"
PORT="${OFFLOAD_LAT_PORT:-5599}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "$HOST")

cleanup() {
  [[ -n "${MAC_PID:-}" ]] && kill "$MAC_PID" 2>/dev/null || true
  [[ -n "${DEV_PID:-}" ]] && "${SSH[@]}" "kill $DEV_PID 2>/dev/null" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo ">> target: $HOST   n=$N   sizes=$SIZES   port=$PORT"
"${SSH[@]}" 'echo reachable' || { echo "ssh failed — load your key / check mDNS"; exit 1; }

echo ">> [0/4] copy the echo helper to the device (/tmp, RAM) and resolve addresses"
scp -q -o BatchMode=yes -o StrictHostKeyChecking=accept-new "$LOCAL_ECHO" "$HOST:$DEV_ECHO"
DEV_IP="${OFFLOAD_DEV_IP:-$("${SSH[@]}" "ip -4 -o addr show usb0 2>/dev/null | awk '{print \$4}' | cut -d/ -f1")}"
MAC_IP="${OFFLOAD_MAC_IP:-$(ifconfig 2>/dev/null | awk '/inet 169\.254\./{print $2}' | head -1)}"
[[ -n "$DEV_IP" ]] || { echo "no device usb0 IPv4 — did 01_enable_tether.sh succeed?"; exit 1; }
[[ -n "$MAC_IP" ]] || { echo "no Mac 169.254.x address — cable? give autoconf ~10 s; or set OFFLOAD_MAC_IP"; exit 1; }
echo "   device usb0 = $DEV_IP    Mac 169.254 = $MAC_IP"

echo ">> [1/4] pass A: Mac drives, DEVICE echoes (device RX+TX path)"
"${SSH[@]}" "nohup $DEV_PY $DEV_ECHO --tcp $PORT >/tmp/offload_echo.log 2>&1 & echo \$!" > /tmp/offload_dev_pid
DEV_PID="$(cat /tmp/offload_dev_pid)"
sleep 1
"$PY" "$LOCAL_ECHO" --client --host "$DEV_IP" --port "$PORT" --sizes "$SIZES" --n "$N"
"${SSH[@]}" "kill $DEV_PID 2>/dev/null" >/dev/null 2>&1 || true
DEV_PID=""

echo ">> [2/4] pass B: DEVICE drives, Mac echoes (the path the device would see)"
"$PY" "$LOCAL_ECHO" --tcp "$PORT" >/tmp/offload_echo_mac.log 2>&1 &
MAC_PID=$!
sleep 1
"${SSH[@]}" "$DEV_PY $DEV_ECHO --client --host $MAC_IP --port $PORT --sizes $SIZES --n $N"
kill "$MAC_PID" 2>/dev/null || true
MAC_PID=""

echo ">> [3/4] reading guide"
cat <<'GUIDE'
   p50   = steady round trip; sanity only.
   p99.9 = THE decision number (the 46 ms HOLD rule is a tail rule).
   max   = worst stall seen; a max >> p99.9 means rare multi-frame freezes, not a fat body.
   Compare our ~8 KB RTT tail against Jetlink's raw-bulk USB3 p99 45.7 ms (mac-performance.md:57-61).
GUIDE

echo ">> [4/4] done. If p99.9/max is near or over 46 ms, the return path is NOT viable as-is —"
echo "         record the numbers in device/RESULTS.md and re-open the transport choice."
