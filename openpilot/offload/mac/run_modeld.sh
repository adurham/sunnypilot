#!/usr/bin/env bash
# run_modeld.sh — drive-mode launcher for the Mac-side modeld_v2 PROCESS.
#
# Why this exists: `modeld_runner` (replay/modeld_runner.py) is a BENCH harness that
# consumes N frames then exits. A live test/drive needs the real modeld_v2 process
# running continuously: it subscribes the Mac-local VisionIPC server that
# framebridge fills, publishes modelV2/cameraOdometry/drivingModelData/
# modelDataV2SP into the Mac-local msgq, and returnsend ships those back to the
# device. Nothing here touches the device or CAN.
#
# Usage:
#   openpilot/offload/mac/run_modeld.sh                       # default VIPC server 'camerad'
#   offload/mac/run_modeld.sh --server cameratest --stats    # isolated server name
#
# Env:
#   OFFLOAD_VIPC_SERVER   VisionIPC server name (default: camerad, i.e. framebridge's)
#   COMBINED_MODEL_PKL    model pkl base path (default: fork metal2 pkl)
#   OFFLOAD_CARPARAMS_PKL CarParams bytes; falls back to CarParamsPersistent, then demo
#   OFFLOAD_DEV / OFFLOAD_WARP_DEV  device selection (default METAL)
#   PY                    interpreter (default <repo>/.venv/bin/python)
#   OFFLOAD_NO_CAFFEINATE=1  skip the caffeinate wrap (CI / non-interactive)
#
# Prints the same pre-run visibility as run_framebridge.sh (caffeinate state,
# pmset assertions, App-Nap note) so a soak log records them.
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PY="${PY:-$REPO/.venv/bin/python}"
export PYTHONPATH="$REPO:$REPO/opendbc_repo:$REPO/msgq_repo:$REPO/tinygrad_repo${PYTHONPATH:+:$PYTHONPATH}"

# --- the OFFLOAD switch: without it modeld_v2 takes the device path and will fail
export OFFLOAD=1
export OFFLOAD_VIPC_SERVER="${OFFLOAD_VIPC_SERVER:-camerad}"
export OFFLOAD_DEV="${OFFLOAD_DEV:-METAL}"
export OFFLOAD_WARP_DEV="${OFFLOAD_WARP_DEV:-METAL}"
export COMBINED_MODEL_PKL="${COMBINED_MODEL_PKL:-$REPO/openpilot/offload/models/driving_supercombo_fork_metal2.pkl}"

# CarParams: prefer an explicit file, else the local param store, else modeld's demo fallback.
if [ -z "${OFFLOAD_CARPARAMS_PKL:-}" ] && [ -f "$HOME/.hermes/cache/scratch/car-features/offload/models/carparams.bin" ]; then
  export OFFLOAD_CARPARAMS_PKL="$HOME/.hermes/cache/scratch/car-features/offload/models/carparams.bin"
fi

echo "=== run_modeld.sh ==="
echo "repo:        $REPO"
echo "python:      $PY"
echo "pkl:         $COMBINED_MODEL_PKL"
echo "vipc server: $OFFLOAD_VIPC_SERVER"
echo "dev/warp:    $OFFLOAD_DEV / $OFFLOAD_WARP_DEV"
echo "carparams:   ${OFFLOAD_CARPARAMS_PKL:-<param store / demo fallback>}"

if [ "${OFFLOAD_NO_CAFFEINATE:-0}" != "1" ]; then
  echo "caffeinate:  wrapping (pmset assertions follow)"
  pmset -g assertions 2>/dev/null | head -6
  echo "appnap:      CLI process; caffeinate -dims keeps the system awake for the run"
  exec caffeinate -dims "$PY" -m openpilot.sunnypilot.modeld_v2.modeld "$@"
else
  echo "caffeinate:  SKIPPED (OFFLOAD_NO_CAFFEINATE=1)"
  exec "$PY" -m openpilot.sunnypilot.modeld_v2.modeld "$@"
fi
