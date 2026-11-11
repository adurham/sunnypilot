#!/usr/bin/env bash
# ============================================================================
# integrity_run.sh — end-to-end uplink BYTE-INTEGRITY run (WS-C, offline).
#
#   fixture -> replayd (ZMQ PUB, device wire) -> framebridge (ZMQ SUB,
#   --digest-out) -> integrity.py check
#
# Proves, for the ZMQ/NCM uplink we actually ship on (COMPARISON-COMMS R11):
#   * every received EncodeData carries the EXACT fixture bytes (sha256 match);
#   * frame_id / encode_id are strictly increasing with ZERO duplicates and ZERO
#     regressions across the whole run;
#   * missing (never-received) frames are counted, not failed — EXCEPT they are
#     still reported, so a reconnect boundary is visible but not a failure.
#
# --stress N additionally kills + restarts framebridge N times mid-run while
# replayd keeps publishing (consumer-reconnect stress); all restarts append to
# the SAME digest jsonl, so the post-reconnect stream is checked in one pass for
# zero duplicated frameIds and zero digest mismatches.
#
# Usage (from the worktree root):
#   bash openpilot/offload/replay/integrity_run.sh \
#     [--fixture NAME_OR_DIR] [--speed S|max] [--stress N] [--frames N] \
#     [--host IP] [--keep] [--strict-missing]
#
# Exit: 0 clean (PASS), 1 any mismatch/dup/replay/regression, 2 setup error.
# ============================================================================
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PY="$REPO/.venv/bin/python"
FIXROOT="${HOME}/.hermes/cache/scratch/car-features/offload/replay-fixtures"
RUNROOT="${HOME}/.hermes/cache/scratch/car-features/offload/integrity"
export PYTHONPATH="$REPO:$REPO/opendbc_repo:$REPO/msgq_repo:$REPO/tinygrad_repo"

FIXTURE="113-x17"        # 17 contiguous segments => 20400 frames/cam = 40800 total
SPEED="8"                # paced: framebridge must decode ~5.5 ms/frame (~110 s/cam); 1020 s
                         # device span / 8 = 127 s wall, so it keeps up. 'max' only for the soak.
STRESS="0"
STRESS_INTERVAL="60"     # seconds between stress restarts (spread across the run)
HOST="127.0.0.1"
KEEP=0
STRICT=""
FRAMES=""                # 1 for --strict-missing in the checker

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fixture) FIXTURE="$2"; shift 2 ;;
    --speed) SPEED="$2"; shift 2 ;;
    --stress) STRESS="$2"; shift 2 ;;
    --stress-interval) STRESS_INTERVAL="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    --keep) KEEP=1; shift ;;
    --strict-missing) FRAMES="1"; shift ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

log() { echo "[integrity_run] $*"; }

# --- resolve fixture --------------------------------------------------------
if [[ -d "$FIXTURE" ]]; then FIXDIR="$FIXTURE"; else FIXDIR="$FIXROOT/$FIXTURE"; fi
if [[ ! -f "$FIXDIR/meta.json" ]]; then
  log "ERROR: no fixture at $FIXDIR (run the extractor first; see INTEGRITY.md)"
  exit 2
fi
FIXNAME="$(basename "$FIXDIR")"

TS="$(date +%Y%m%d-%H%M%S)"
RUN="$RUNROOT/$TS-$FIXNAME"
mkdir -p "$RUN"
log "fixture=$FIXDIR  run=$RUN  speed=$SPEED  stress=$STRESS"

# --- params (VPS/SPS/PPS) for vtdec -----------------------------------------
PARAMS="$RUN/params"; mkdir -p "$PARAMS"
for cam in narrow wide; do
  [[ -f "$FIXDIR/$cam.header" ]] && cp "$FIXDIR/$cam.header" "$PARAMS/$cam.params"
done

# --- preflight: our own ports/processes clear -------------------------------
for i in $(seq 1 40); do
  busy="$(pgrep -fl 'offload.replay.replayd|offload.mac.framebridge' | grep -v pgrep || true)"
  [[ -z "$busy" ]] && break
  log "waiting for another session's replayd/framebridge to clear..."; sleep 3
done
# msgq shm segments from a just-exited run can still trip a new PubMaster
# (MultiplePublishersError on <cam>RoadCameraState); let them settle.
sleep 8

# --- expected digests (ground truth) ----------------------------------------
EXPECTED="$RUN/expected.jsonl"
"$PY" -m openpilot.offload.replay.integrity expected "$FIXDIR" --out "$EXPECTED" | tee "$RUN/expected.log"
EXP_ROWS="$(wc -l < "$EXPECTED" | tr -d ' ')"
log "expected rows: $EXP_ROWS"

# --- start replayd ----------------------------------------------------------
PUBLOG="$RUN/replayd_pub.jsonl"
"$PY" -m openpilot.offload.replay.replayd "$FIXDIR" --fixture-dir "$FIXDIR" \
  --host "$HOST" --speed "$SPEED" --pub-log "$PUBLOG" --stats > "$RUN/replayd.log" 2>&1 &
REPLAYD_PID=$!
sleep 1.5
if ! kill -0 "$REPLAYD_PID" 2>/dev/null; then
  log "ERROR: replayd died at startup:"; cat "$RUN/replayd.log"; exit 2
fi
log "replayd pid=$REPLAYD_PID"

# --- start framebridge ------------------------------------------------------
DIGEST="$RUN/digest.jsonl"
LAT="$RUN/latency.jsonl"
FBLOG="$RUN/framebridge.log"
FBPID="$RUN/fb.pid"
start_fb() {
  "$PY" "$REPO/openpilot/offload/mac/framebridge.py" --host "$HOST" \
    --vtdec "$REPO/openpilot/offload/mac/vtdec" --out "$LAT" \
    --digest-out "$DIGEST" --params-dir "$PARAMS" >> "$FBLOG" 2>&1 &
  echo $! > "$FBPID"
}
kill_fb() {
  local p; p="$(cat "$FBPID" 2>/dev/null || true)"
  [[ -n "$p" ]] && kill -TERM "$p" 2>/dev/null || true
}
FB_PID="$(start_fb; cat "$FBPID")"
sleep 1.5
log "framebridge pid=$FB_PID"

cleanup() {
  kill_fb
  [[ -n "${REPLAYD_PID:-}" ]] && kill -TERM "$REPLAYD_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

# --- reconnect-stress driver ------------------------------------------------
if [[ "$STRESS" -gt 0 ]]; then
  log "stress: $STRESS framebridge restarts every ${STRESS_INTERVAL}s while replayd publishes"
  (
    for n in $(seq 1 "$STRESS"); do
      sleep "$STRESS_INTERVAL"
      if ! kill -0 "$REPLAYD_PID" 2>/dev/null; then break; fi
      log "stress $n/$STRESS: killing framebridge pid=$(cat "$FBPID")"
      kill_fb
      sleep 3
      start_fb
      log "stress $n/$STRESS: restarted framebridge pid=$(cat "$FBPID")"
    done
  ) &
  STRESS_PID=$!
fi

# --- run to completion ------------------------------------------------------
log "running (replayd speed=$SPEED)..."
wait "$REPLAYD_PID" || true
REPLAYD_RC=$?
log "replayd exited rc=$REPLAYD_RC"
[[ "$STRESS" -gt 0 ]] && { kill "$STRESS_PID" 2>/dev/null || true; wait "$STRESS_PID" 2>/dev/null || true; }

# stop framebridge (the last one) and let it flush its digest jsonl
sleep 1.5
kill_fb
wait 2>/dev/null || true
sleep 0.5

# --- check ------------------------------------------------------------------
log "digest rows: $(wc -l < "$DIGEST" 2>/dev/null || echo 0)"
set +e
EXTRA=""; [[ "$FRAMES" == "1" ]] && EXTRA="--strict-missing"
"$PY" -m openpilot.offload.replay.integrity check "$EXPECTED" "$DIGEST" \
  --json "$RUN/integrity.json" $EXTRA | tee "$RUN/check.log"
CHECK_RC=$?
set -e

echo
echo "=========================================================================="
echo " integrity run: $FIXNAME   run dir: $RUN"
echo "   expected=$EXP_ROWS  received=$(wc -l < "$DIGEST" | tr -d ' ')  stress_restarts=$STRESS"
if [[ -f "$RUN/integrity.json" ]]; then
  "$PY" - "$RUN/integrity.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
t = d["totals"]
print(f"   status={d['status']}  expected={t['expected']} received={t['received']} "
      f"missing={t['missing']} mismatch={t['mismatch']} dup={t['duplicate']} "
      f"replay={t['replay']} regression={t['regression']} rate={t['digest_match_rate']}")
PY
fi
echo "=========================================================================="

exit "$CHECK_RC"
