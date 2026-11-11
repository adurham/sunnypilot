#!/usr/bin/env bash
# Live end-to-end: framebridge (device -> Mac VisionIPC) + the fork modeld_v2 runner.
set -u
cd "$HOME/repos/sunnypilot-offload" || exit 1
export PYTHONPATH="$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo"
rm -f /tmp/fb_model.jsonl /tmp/fb_model.log /tmp/mr.log

echo "[1/3] framebridge up"
.venv/bin/python openpilot/offload/mac/framebridge.py --host 169.254.60.1 --out /tmp/fb_model.jsonl \
  > /tmp/fb_model.log 2>&1 &
FB=$!
sleep 7   # VisionIPC + first IDR

echo "[2/3] modeld_v2 runner (live VisionIPC, OFFLOAD=1)"
# The runner needs the model pkl + CarParams. Use the fork-flavor METAL pkl built in-tree.
export COMBINED_MODEL_PKL="$PWD/openpilot/offload/models/driving_supercombo_fork_metal2.pkl"
[ -f "$HOME/.hermes/cache/scratch/car-features/offload/models/carparams.bin" ] && \
  export OFFLOAD_CARPARAMS_PKL="$HOME/.hermes/cache/scratch/car-features/offload/models/carparams.bin"
OFFLOAD=1 .venv/bin/python -m openpilot.offload.replay.modeld_runner --frames 300 --timeout 40 \
  > /tmp/mr.log 2>&1 &
MR=$!
wait $MR 2>/dev/null || true
sleep 1
kill $FB 2>/dev/null || true
sleep 2

echo "[3/3] results"
echo "--- framebridge ---"; tail -3 /tmp/fb_model.log 2>/dev/null
echo "--- modeld runner (head) ---"; head -25 /tmp/mr.log 2>/dev/null
echo "--- modeld runner (tail) ---"; tail -15 /tmp/mr.log 2>/dev/null
