#!/bin/bash
# run_framebridge.sh — launch framebridge.py under caffeinate, with App-Nap visibility.
#
# Why: a soak/bench run on a Mac must not be interrupted by idle sleep, and an
# in-process server can be throttled by App Nap. jetlink measured a throttled
# in-process server at p50 75 ms vs 36 ms for the same work in a CLI process
# (JetlinkKit/.../Server.swift:530-545, ProcessInfo.beginActivity). Our framebridge
# is a plain CLI process and is very likely already exempt from App Nap, but
# wrapping it in `caffeinate -dims` is cheap insurance. Use this wrapper for ALL
# soak/bench runs.
#
# What it prints BEFORE exec (so the operator can see pre-existing state):
#   (a) whether caffeinate wrapping is active
#   (b) `pmset -g assertions` output (system-wide sleep assertions, non-sudo)
#   (c) a one-line App-Nap / NSAppSleepDisabled note
#
# All arguments are passed through to framebridge.py unchanged (ZMQ mode by
# default, or --replay ...).
#
# Usage:
#   openpilot/offload/mac/run_framebridge.sh --host 192.168.60.1 --out logs/run.jsonl
#   openpilot/offload/mac/run_framebridge.sh --replay --replay-path <dir> --replay-cam narrow
#
# Env:
#   OFFLOAD_NO_CAFFEINATE=1   skip the caffeinate wrap (for CI / non-interactive)
#   PY                        python interpreter (default <repo>/.venv/bin/python)
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
# PY may be caller-exported (e.g. a relative ".venv/bin/python" from a different
# cwd); prefer it only if it actually runs, else fall back to the repo venv.
DEFAULT_PY="$REPO/.venv/bin/python"
if [ -n "${PY:-}" ] && [ -x "$PY" ] 2>/dev/null; then
  :
elif [ -n "${PY:-}" ] && command -v "$PY" >/dev/null 2>&1; then
  PY="$(command -v "$PY")"
else
  if [ -n "${PY:-}" ]; then
    echo "run_framebridge.sh: PY='$PY' is not executable; using $DEFAULT_PY" >&2
  fi
  PY="$DEFAULT_PY"
fi
export PYTHONPATH="$REPO:$REPO/opendbc_repo:$REPO/msgq_repo:$REPO/tinygrad_repo${PYTHONPATH:+:$PYTHONPATH}"

MODE="zmq"
for a in "$@"; do
  [ "$a" = "--replay" ] && MODE="replay"
done

echo "=== run_framebridge.sh ==="
echo "repo:       $REPO"
echo "python:     $PY"
echo "framebridge: mode=$MODE args=$*"
echo

# (b) pre-existing sleep assertions, BEFORE we start anything.
echo "--- pmset -g assertions (BEFORE) ---"
if command -v pmset >/dev/null 2>&1; then
  pmset -g assertions 2>&1 || echo "(pmset -g assertions unavailable)"
else
  echo "(pmset not found; cannot report assertions)"
fi
echo

# (c) one-line App-Nap / NSAppSleepDisabled note.
NSAPP="$(defaults read -g NSAppSleepDisabled 2>/dev/null || true)"
if [ "${NSAPP:-0}" = "1" ]; then
  echo "App Nap: globally disabled (NSAppSleepDisabled=1); the CLI is not a bundled NSApplication either way — caffeinate is still cheap insurance."
else
  echo "App Nap: active for GUI apps (NSAppSleepDisabled unset); framebridge is a plain CLI process, so App Nap should not apply — caffeinate is cheap insurance anyway."
fi
echo

# (a) decide on the caffeinate wrap.
if [ "${OFFLOAD_NO_CAFFEINATE:-0}" = "1" ]; then
  echo "caffeinate: NOT wrapping (OFFLOAD_NO_CAFFEINATE=1)."
  echo
  echo "--- exec framebridge (no caffeinate) ---"
  exec "$PY" -m openpilot.offload.mac.framebridge "$@"
elif command -v caffeinate >/dev/null 2>&1; then
  echo "caffeinate: WRAPPING as \`caffeinate -dims\` (display+idle+disk+system)."
  echo
  echo "--- exec caffeinate -dims framebridge ---"
  exec caffeinate -dims "$PY" -m openpilot.offload.mac.framebridge "$@"
else
  echo "caffeinate: NOT wrapping (caffeinate not found); idle sleep is NOT prevented."
  echo
  echo "--- exec framebridge (no caffeinate) ---"
  exec "$PY" -m openpilot.offload.mac.framebridge "$@"
fi
