#!/usr/bin/env bash
# Apply the fork's tinygrad patches to the tinygrad_repo submodule (idempotent).
# Usage: bash openpilot/offload/tinygrad-fixes/apply.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"          # .../sunnypilot-offload
TG="$REPO/tinygrad_repo"

cd "$TG"

for p in "$HERE"/[0-9]*.patch; do
  [ -f "$p" ] || continue
  name="$(basename "$p")"
  if git apply --reverse --check "$p" >/dev/null 2>&1; then
    echo "[tinygrad-fixes] $name already applied"
  elif git apply --check "$p" >/dev/null 2>&1; then
    git apply "$p"
    echo "[tinygrad-fixes] $name applied"
  else
    echo "[tinygrad-fixes] ERROR: $name does not apply cleanly to $(git rev-parse --short HEAD)" >&2
    echo "[tinygrad-fixes] (the submodule pin probably moved — rebase the patch first)" >&2
    exit 1
  fi
done

echo "[tinygrad-fixes] done; submodule state:"
git status --short
