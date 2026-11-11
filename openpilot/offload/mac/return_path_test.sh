#!/bin/bash
# return_path_test.sh — one-command, end-to-end RETURN-path test on THIS Mac (no device).
#
# Runs returnsend (Mac msgq -> ZMQ PUB) AND offloadd (ZMQ SUB -> device msgq SHADOW topics) as
# real subprocesses on 127.0.0.1, publishes fake modeld outputs with a recent SOF into the Mac
# msgq, then raw-subscribes the shadow topics and verifies the full chain: frameId preserved,
# payload identical except the header, and the header re-stamped to receipt time (offloadd's
# receipt re-stamp, §1.3/§7). One machine is the correct model: here offloadd's "device" clock IS
# this machine's monotonic clock, exactly as on a drive where both ends share one monotonic domain.
#
# Usage:
#   openpilot/offload/mac/return_path_test.sh
#   PY=/path/to/python openpilot/offload/mac/return_path_test.sh     # override interpreter
#
# Exit 0 = chain proven. Also see the pytest form:
#   PYTHONPATH=$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/tinygrad_repo \
#     .venv/bin/python -m pytest openpilot/offload/mac/tests/test_returnsend.py -v
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
DEFAULT_PY="$REPO/.venv/bin/python"
if [ -n "${PY:-}" ] && [ -x "${PY:-}" ]; then
  :
else
  PY="$DEFAULT_PY"
fi
export PYTHONPATH="$REPO:$REPO/opendbc_repo:$REPO/msgq_repo:$REPO/tinygrad_repo"

echo "=== return_path_test.sh ==="
echo "repo:   $REPO"
echo "python: $PY"
echo "target: openpilot/offload/mac/tests/test_returnsend.py::test_full_return_chain_into_shadow_topics"
echo

cd "$REPO"
"$PY" -m pytest \
  "openpilot/offload/mac/tests/test_returnsend.py::test_full_return_chain_into_shadow_topics" \
  -v -s
rc=$?
echo
if [ "$rc" -eq 0 ]; then
  echo "RETURN PATH OK (returnsend + offloadd chain proven end-to-end)"
else
  echo "RETURN PATH FAILED (pytest exit $rc)"
fi
exit "$rc"
