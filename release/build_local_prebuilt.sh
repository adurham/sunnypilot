#!/usr/bin/env bash

# Local prebuilt release builder for sunnypilot
#
# Builds the full project in Docker (linux/arm64) and pushes a prebuilt branch.
# Requires: Docker Desktop with >=16GB memory, git, gh CLI
#
# Usage: ./release/build_local_prebuilt.sh [target-branch]

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
SOURCE_DIR="$(cd "$DIR/.." && pwd)"
cd "$SOURCE_DIR"

SOURCE_BRANCH="$(git branch --show-current)"
PREBUILT_BRANCH="${1:-${SOURCE_BRANCH}-prebuilt}"
GH_USER=$(gh api user --jq '.login' 2>/dev/null || echo "adurham")
OUTPUT_DIR="/tmp/sp_prebuilt_output"

echo "=== sunnypilot local prebuilt builder ==="
echo "Source: $SOURCE_BRANCH ($(git rev-parse --short HEAD))"
echo "Target: $PREBUILT_BRANCH"
echo ""

# Step 1: Submodules
echo "[-] Submodules... T=$SECONDS"
git submodule update --init --recursive > /dev/null 2>&1

# Step 2: Docker image (cached after first build)
echo "[-] Docker image... T=$SECONDS"
docker build --platform linux/arm64 -t sunnypilot-builder -f Dockerfile.openpilot . > /dev/null 2>&1 || {
  echo "!!! Docker build failed"; exit 1
}

# Step 3: Compile in Docker with output via volume mount
# NOTE: scons output is redirected to log files inside the container to avoid
# stdout buffer issues that cause Docker to exit 128.
echo "[-] Compiling in Docker... T=$SECONDS"
rm -rf "$OUTPUT_DIR" && mkdir -p "$OUTPUT_DIR"

docker run --platform linux/arm64 --rm \
  -v "$OUTPUT_DIR:/output" \
  sunnypilot-builder bash -c '
cd /home/batman/openpilot
source .venv/bin/activate
export PYTHONPATH=/home/batman/openpilot
git init . > /dev/null 2>&1 && git add -A > /dev/null 2>&1 && git commit -m "build" --allow-empty > /dev/null 2>&1
sed -i "/from .board.jungle import/s/^/#/" panda/__init__.py 2>/dev/null || true

echo "[-] scons --minimal"
scons -j$(nproc) --minimal > /tmp/scons.log 2>&1 || { echo "!!! scons --minimal FAILED"; tail -30 /tmp/scons.log; exit 1; }
echo "[-] scons panda/"
scons -j$(nproc) panda/ > /tmp/panda.log 2>&1 || { echo "!!! scons panda/ FAILED"; tail -30 /tmp/panda.log; exit 1; }

echo "[-] Cleaning up..."
find . -name "*.a" -delete
find . -name "*.o" -delete
find . -name "*.os" -delete
find . -name "*.pyc" -delete
find . -name "moc_*" -delete
find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
rm -rf .sconsign.dblite Jenkinsfile release/ .git SConstruct .venv .gitmodules .github
rm -f .gitattributes .lfsconfig
find . -name "SConscript" -delete
rm -f selfdrive/modeld/models/*.onnx sunnypilot/modeld*/models/*.onnx
find third_party/ -name "*x86*" -exec rm -r {} + 2>/dev/null || true
find third_party/ -name "*Darwin*" -exec rm -r {} + 2>/dev/null || true
touch prebuilt

# Patch launch_env.sh for non-TICI build: C++ binaries compiled without
# __TICI__ use HardwarePC paths by default. These env vars override them
# to device paths at runtime. (Python side detects /TICI at runtime, unaffected.)
cat >> launch_env.sh <<ENVEOF

# Override C++ HardwarePC paths for prebuilt (compiled without __TICI__)
export PARAMS_ROOT="/data/params"
export LOG_ROOT="/data/media/0/realdata"
ENVEOF

echo "[-] Copying to output..."
cp -a . /output/openpilot
echo "=== BUILD COMPLETE ==="
'
DOCKER_RC=$?

if [ $DOCKER_RC -ne 0 ]; then
  echo "!!! BUILD FAILED (exit $DOCKER_RC) !!!"; exit 1
fi
echo "[-] Build succeeded. T=$SECONDS"

# Step 4: Publish
echo "[-] Publishing... T=$SECONDS"
cd "$OUTPUT_DIR/openpilot"

find . -name ".git" -exec rm -rf {} + 2>/dev/null || true
rm -f .gitattributes .lfsconfig  # prevent LFS smudge on device checkout
git init -q
git add -A -f > /dev/null
git commit -q -m "sunnypilot prebuilt ($SOURCE_BRANCH)
source: $(cd "$SOURCE_DIR" && git rev-parse HEAD)
date: $(date '+%Y-%m-%dT%H:%M:%S')
"
GIT_LFS_SKIP_PUSH=1 git push -f "https://github.com/${GH_USER}/sunnypilot.git" HEAD:"$PREBUILT_BRANCH" 2>&1 | grep -v warning | tail -3

cd "$SOURCE_DIR"
rm -rf "$OUTPUT_DIR"

echo ""
echo "=== Done! T=$SECONDS ==="
echo "Deploy: ssh comma@<ip> 'cd /data/openpilot && GIT_LFS_SKIP_SMUDGE=1 git fetch adurham $PREBUILT_BRANCH --depth=1 && GIT_LFS_SKIP_SMUDGE=1 git checkout -B $PREBUILT_BRANCH FETCH_HEAD && rm -f .gitattributes .lfsconfig .overlay_init && sudo reboot'"
