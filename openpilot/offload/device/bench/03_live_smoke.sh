#!/usr/bin/env bash
# ============================================================================
# 03_live_smoke.sh — run the Mac frame bridge against the DEVICE over the tether,
#                    then report frame counts and decode latency percentiles.
#
# PURPOSE
#   End-to-end smoke of the offload frame path: the Mac subscribes to the device's
#   ZMQ bridge (EncodeData) and runs the WS-B frame pipeline (framebridge.py).
#
# PREREQS
#   * 01+02 done (tether up, iperf3 healthy) — OR device reachable on the LAN.
#   * WS-B landed: openpilot/offload/mac/framebridge.py (+ built vtdec) present.
#   * Run from the repo root with the repo venv.
#
# EXPECTED READINGS
#   * frame count ~= duration * 20 fps per camera (narrow [+ wide]).
#   * decode p50 <= 3 ms, p99.9 <= 10 ms (INTERFACES gate G7).
#   * SOF->publish p50 15-25 ms at the frame layer (INTERFACES §6 soak readings).
#
# USER-VISIBLE EFFECTS
#   * Starts nothing on the device (the device's bridge must already be running for a
#     live capture — see device/README.md; that start is user-gated).
#   * Writes /tmp/fb.jsonl on the Mac only.
# ============================================================================
set -euo pipefail

DEV_IP="${1:-}"
SECS="${2:-30}"
if [[ -z "$DEV_IP" || "$DEV_IP" == -* ]]; then
  echo "usage: bash 03_live_smoke.sh <device-ip> [seconds]   (e.g. 169.254.x.y)" >&2; exit 2
fi

OUT="/tmp/fb.jsonl"
FB="openpilot/offload/mac/framebridge.py"
PY=".venv/bin/python"

echo ">> target device: $DEV_IP   duration: ${SECS}s   out: $OUT"
if [[ ! -f "$FB" ]]; then
  echo "WS-B framebridge not present yet: $FB" >&2
  echo "Intended invocation (WS-B interface, INTERFACES §3):" >&2
  echo "  $PY $FB --host $DEV_IP --out $OUT    # run for ${SECS}s" >&2
  echo "SKIP: build WS-B first, then re-run." >&2
  exit 3
fi

echo ">> running: $PY $FB --host $DEV_IP --out $OUT   (timeout ${SECS}s)"
rm -f "$OUT"
# macOS has no GNU timeout by default; use it if present, else perl alarm shim.
if command -v timeout >/dev/null 2>&1; then
  timeout "$SECS" "$PY" "$FB" --host "$DEV_IP" --out "$OUT" || true
elif command -v gtimeout >/dev/null 2>&1; then
  gtimeout "$SECS" "$PY" "$FB" --host "$DEV_IP" --out "$OUT" || true
else
  perl -e 'alarm shift; exec @ARGV' "$SECS" "$PY" "$FB" --host "$DEV_IP" --out "$OUT" || true
fi

echo ">> reporting from $OUT"
"$PY" - "$OUT" <<'PY'
import json, sys, statistics as st
path = sys.argv[1]
rows = []
try:
  with open(path) as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      try: rows.append(json.loads(line))
      except json.JSONDecodeError: pass
except FileNotFoundError:
  print("no output file — framebridge produced nothing (check device bridge + host)"); sys.exit(1)

print(f"frames: {len(rows)}")
if rows:
  print("keys:", sorted(rows[0].keys()))
def pct(vals, p):
  if not vals: return None
  vals = sorted(vals); k = min(len(vals)-1, int(round((p/100.0)*(len(vals)-1))))
  return vals[k]
for key in ("decode_ms", "sof_to_arrival_ms", "sof_to_publish_ms"):
  vals = [r[key] for r in rows if isinstance(r.get(key), (int, float)) and r[key] >= 0]
  if vals:
    print(f"{key}: n={len(vals)} p50={pct(vals,50):.2f} p99={pct(vals,99):.2f} p99.9={pct(vals,99.9):.2f} mean={st.fmean(vals):.2f}")
PY
echo "OK — compare against gate G7 (decode p50<=3 / p99.9<=10 ms) in INTERFACES §5."
