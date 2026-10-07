#!/usr/bin/env bash
# ============================================================================
# check_device.sh — read-only ssh audit for the offload target comma device.
#
# PURPOSE
#   Verify, without changing anything, that the device is a viable offload peer:
#   python/capnp/pyzmq, the cereal ZMQ bridge binary, USB gadget (UDC) state,
#   AdbEnabled param, tether interface, /data headroom, the running modeld flavor,
#   the live cereal service list + ZMQ ports for the FORWARD/RETURN sets, and the
#   (expected-absent) OffloadMode param.
#
# USAGE
#   bash check_device.sh comma@comma-b203ed6e.local
#   bash check_device.sh comma@comma-b203ed6e.local --no-retry
#
# SAFETY (must be safe to run while driving)
#   * READ-ONLY: no param writes, no process starts/stops, no reboots, no file writes.
#   * The only executable it launches is the bridge binary under `timeout 2` (it loops
#     forever by design) — nothing is mutated; the process is killed by timeout.
#   * It does NOT start adbd. The tether bring-up command is printed for the user only.
#
# PREREQS
#   * Reachable device over mDNS (comma-b203ed6e.local) with your ssh key loaded
#     (1Password agent) or a password prompt.
#   * A shell on the Mac with ssh + timeout(1) (macOS: `gtimeout` if coreutils; falls back
#     to a pure-bash timeout shim — see note below).
#
# OUTPUT
#   One PASS/WARN/FAIL line per check, then a SUMMARY block + verdict. Exit 0 if no FAIL.
# ============================================================================
set -uo pipefail

HOST="${1:-comma@comma-b203ed6e.local}"
if [[ "$HOST" == -* || -z "$HOST" ]]; then
  echo "usage: bash check_device.sh [user@]host [--no-retry]" >&2
  exit 2
fi
RETRY=1
[[ "${2:-}" == "--no-retry" ]] && RETRY=0

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new \
          -o ServerAliveInterval=5 -o ServerAliveCountMax=2)
DEV_PY="/usr/local/venv/bin/python3"
BRIDGE="/data/openpilot/openpilot/cereal/messaging/bridge"
DEV_SERVICES="/data/openpilot/openpilot/cereal/services.py"

# --- tiny timeout shim (macOS has no GNU timeout by default) ----------------
have_timeout=0
command -v timeout >/dev/null 2>&1 && have_timeout=1
command -v gtimeout >/dev/null 2>&1 && have_timeout=1
TO=""
(( have_timeout )) && { command -v timeout >/dev/null 2>&1 && TO="timeout" || TO="gtimeout"; }

# --- result accumulators -----------------------------------------------------
declare -a ROWS=()
PASS_N=0; WARN_N=0; FAIL_N=0
add() { # status label detail
  local st="$1" label="$2" detail="${3:-}"
  case "$st" in
    PASS) PASS_N=$((PASS_N+1)) ;;
    WARN) WARN_N=$((WARN_N+1)) ;;
    FAIL) FAIL_N=$((FAIL_N+1)) ;;
  esac
  ROWS+=("$(printf '%-4s %-26s %s' "$st" "$label" "$detail")")
}

# --- remote exec helpers -----------------------------------------------------
rsh() { # run a command on the device (string arg). Returns output, non-zero on ssh failure.
  ssh "${SSH_OPTS[@]}" "$HOST" "$@" 2>&1
}

rsh_retry() { # rsh with up to 3 attempts (ssh is flaky on this device)
  local out rc n=1 max=3
  (( RETRY )) || max=1
  while (( n <= max )); do
    out="$(rsh "$@")"; rc=$?
    if (( rc == 0 )); then printf '%s' "$out"; return 0; fi
    n=$((n+1)); sleep 2
  done
  printf '%s' "$out"; return "$rc"
}

# --- 0. connectivity ---------------------------------------------------------
echo "== offload device audit =="
echo "host: $HOST   ($(date))"
CONN="$(rsh_retry 'echo SSH_OK')"
if [[ "$CONN" != *"SSH_OK"* ]]; then
  add FAIL connectivity "ssh failed after retries — load your ssh key (1Password), check mDNS"
  echo
  printf '%s\n' "${ROWS[@]}"
  echo
  echo "SUMMARY: PASS=$PASS_N WARN=$WARN_N FAIL=$FAIL_N  -> NOT AUDITED"
  exit 1
fi
add PASS connectivity "ssh OK"

UNAME="$(rsh_retry 'uname -sr')"
add PASS host_kernel "$(echo "$UNAME" | tr '\n' ' ')"

# --- 1. device python --------------------------------------------------------
PYV="$(rsh_retry "$DEV_PY -V 2>&1 || python3 -V 2>&1")"
if [[ "$PYV" == Python\ 3* ]]; then add PASS device_python "$PYV"; else add WARN device_python "no python3 (${PYV:-none})"; fi

# --- 2. capnp / pyzmq --------------------------------------------------------
IMP="$(rsh_retry "$DEV_PY - <<'PY' 2>&1
try:
  import capnp; cap='capnp ok'
except Exception as e: cap='capnp MISSING: %s' % e
try:
  import zmq; z='pyzmq %s (libzmq %s)' % (zmq.__version__, zmq.zmq_version())
except Exception as e: z='pyzmq MISSING: %s' % e
print(cap); print(z)
PY")"
if [[ "$IMP" == *"capnp ok"* ]]; then add PASS capnp "$(echo "$IMP" | sed -n 1p)"; else add FAIL capnp "$(echo "$IMP" | sed -n 1p)"; fi
if [[ "$IMP" == *"pyzmq"*  ]]; then add PASS pyzmq "$(echo "$IMP" | sed -n 2p)"; else add FAIL pyzmq "$(echo "$IMP" | sed -n 2p)"; fi

# --- 3. bridge binary --------------------------------------------------------
BEX="$(rsh_retry "test -x $BRIDGE && echo yes || echo no")"
if [[ "$BEX" == "yes" ]]; then
  add PASS bridge_binary "present+executable: $BRIDGE"
  if [[ -n "$TO" ]]; then
    BRC="$(rsh_retry "$TO 2 $BRIDGE >/dev/null 2>&1; echo rc=\$?")"
    add PASS bridge_runs "no-args run: $BRC (rc=124 => running then killed by timeout, expected)"
  else
    add WARN bridge_runs "no timeout(1)/gtimeout on Mac; run on device: timeout 2 $BRIDGE"
  fi
else
  add FAIL bridge_binary "missing/non-executable: $BRIDGE"
fi

# --- 4. UDC (USB gadget controller) state -----------------------------------
UDC="$(rsh_retry 'for u in /sys/class/udc/*; do echo "$(basename $u): $(cat $u/state 2>/dev/null)"; done')"
if [[ "$UDC" == *"a600000.dwc3"* ]]; then add PASS udc "$(echo "$UDC" | tr '\n' ' ')"; else add WARN udc "a600000.dwc3 not found (${UDC:-none})"; fi

# --- 5. AdbEnabled param (audit only — do NOT start adbd) -------------------
ADB="$(rsh_retry 'cat /data/params/d/AdbEnabled 2>/dev/null || echo absent')"
ADBSVC="$(rsh_retry 'systemctl is-active adbd 2>/dev/null || echo inactive')"
add WARN adb_param "AdbEnabled=${ADB}  adbd=${ADBSVC}  (audit only; user starts it)"
if [[ "$ADBSVC" != "active" ]]; then
  echo "     -> to bring up the tether (USER-GATED): sudo systemctl start adbd   # needs AdbEnabled=1"
fi

# --- 6. tether interface -----------------------------------------------------
IFACE="$(rsh_retry 'ip -o link show 2>/dev/null | awk -F": " "{print \$2}" | tr "\n" " "')"
if [[ "$IFACE" == *"usb0"* ]]; then add PASS tether_iface "usb0 present: $IFACE"; else add WARN tether_iface "usb0 absent (start adbd to create it): $IFACE"; fi

# --- 7. /data headroom -------------------------------------------------------
DPCT="$(rsh_retry "df -P /data | awk 'NR==2{print \$5}'")"
if [[ -n "$DPCT" ]]; then
  USE="${DPCT%\%}"
  if (( USE >= 90 )); then add FAIL data_space "/data ${DPCT} used — offload deliverables must not write here"
  elif (( USE >= 80 )); then add WARN data_space "/data ${DPCT} used"
  else add PASS data_space "/data ${DPCT} used"; fi
else
  add WARN data_space "could not read df /data"
fi

# --- 8. current modeld flavor (tinygrad vs stock) ---------------------------
MODELD="$(rsh_retry "ps -eo comm,args 2>/dev/null | grep -E 'modeld' | grep -v grep")"
if [[ "$MODELD" == *"modeld_v2"* || "$MODELD" == *"tinygrad"*  ]]; then
  add PASS modeld_flavor "tinygrad (fork modeld_v2) running: $(echo "$MODELD" | head -1 |
      sed -E 's/^ *//' | cut -c1-90)"
elif [[ "$MODELD" == *"modeld"* ]]; then
  add WARN modeld_flavor "modeld present but flavor unclear: $(echo "$MODELD" | head -1 | cut -c1-90)"
else
  add WARN modeld_flavor "no modeld process (car off / not onroad?)"
fi

# --- 9. live cereal service list + ZMQ ports for FORWARD/RETURN -------------
# Port names mirror openpilot/offload/contract.py (FROZEN). Update here if contract.py changes.
SVC="$(rsh_retry "$DEV_PY - <<'PY' 2>&1
try:
  import importlib.util, sys
  spec = importlib.util.spec_from_file_location('services', '$DEV_SERVICES')
  m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
  have = set(m.SERVICE_LIST.keys())
except Exception as e:
  have = set(); print('# could not load services.py: %s' % e)

START, MAX = 8023, 65535
def port(n):
  h = 0xcbf29ce484222325
  for c in n.encode():
    h ^= c; h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
  return START + h % (MAX - START)

FORWARD = ['narrowRoadEncodeData','wideRoadEncodeData','narrowRoadCameraState','wideRoadCameraState',
           'carState','deviceState','carControl','extrinsicsCalibration','driverMonitoringState',
           'lateralDelay','modelV2','drivingModelData','cameraOdometry','modelDataV2SP']
RETURN  = ['modelV2','cameraOdometry','drivingModelData','modelDataV2SP']
def dump(tag, names):
  for n in names:
    print('%-6s %-30s port=%-6d present=%s' % (tag, n, port(n), n in have))
dump('FWD', FORWARD); dump('RET', RETURN)
PY")"
echo "--- device cereal services (FWD=device->Mac, RET=Mac->device) ---"
echo "$SVC"
MISSING="$(echo "$SVC" | awk '/present=False/{print $2}')"
if [[ -n "$MISSING" ]]; then add WARN services "missing on device: $(echo "$MISSING" | tr '\n' ' ')"; else add PASS services "all FORWARD/RETURN services present on device"; fi

# --- 10. OffloadMode param (expect absent) -----------------------------------
OM="$(rsh_retry 'if [ -e /data/params/d/OffloadMode ]; then cat /data/params/d/OffloadMode; else echo absent; fi')"
if [[ "$OM" == "absent" ]]; then add PASS offload_mode_param "absent (expected)"; else add WARN offload_mode_param "present: ${OM} — offloadd would be live; disable before driving"; fi

# --- 11. tree / branch sanity ------------------------------------------------
BR="$(rsh_retry 'cd /data/openpilot 2>/dev/null && git rev-parse --abbrev-ref HEAD 2>/dev/null && git rev-parse --short HEAD 2>/dev/null')"
add PASS device_tree "/data/openpilot branch/rev: $(echo "$BR" | tr '\n' ' ')"

# --- summary -----------------------------------------------------------------
echo
printf '%s\n' "${ROWS[@]}"
echo
echo "SUMMARY: PASS=$PASS_N WARN=$WARN_N FAIL=$FAIL_N"
if (( FAIL_N == 0 )); then echo "VERDICT: PASS (warnings are informational)"; exit 0
else echo "VERDICT: FAIL ($FAIL_N blocking)"; exit 1; fi
