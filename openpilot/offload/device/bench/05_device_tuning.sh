#!/usr/bin/env bash
# ============================================================================
# 05_device_tuning.sh — USER-GATED: read, apply, or revert the device-side kernel tuning.
#
# !!! RE-APPLY AFTER EVERY AGNOS UPDATE !!!
#   The sysctls below are RUNTIME values; an AGNOS/kernel update (or a reboot) resets them to
#   stock. Re-run `--apply` after any update. `vm.extra_free_kbytes` in particular was removed
#   from later kernels (it is Android's; mainline dropped it ~5.x) — on a kernel that lacks it
#   the write simply fails, which this script reports per key rather than aborting the rest.
#
# PURPOSE
#   Apply the three measured device-side wins Jetlink found (jetlink-root.sh:93, :127-128) to our
#   device, gated so nothing changes without an explicit `--apply`:
#     1) vm.extra_free_kbytes=32768 (+ the dirty caps) — kswapd keeps ~32 MB more free ahead of
#        the allocators, so loggerd's dirty-page reclaim does not stall a transfer.
#        >>> NOT vm.min_free_kbytes <<< — a min_free floor takes ~3x its value out of
#        MemAvailable and false-trips openpilot's LOW-MEMORY alert (openpilot compares
#        MemTotal-MemAvailable against 90%; a real bug Jetlink shipped and reverted). extra_free
#        leaves the direct-reclaim floor where it is. Do not "improve" this to min_free_kbytes.
#     2) vm.dirty_bytes / vm.dirty_background_bytes — cap dirty memory so the synchronous
#        reclaim that a FunctionFS alloc runs into is cheap. Stock AGNOS runs the dirty limits in
#        ratio mode, so the *_bytes keys read 0 and a 0 written back is silently dropped; we
#        detect the mode and, if bytes are unavailable, fall back to the ratio caps instead
#        (vm.dirty_background_ratio / vm.dirty_ratio — bytes and ratio are mutually exclusive,
#        writing one zeroes the other).
#     3) net.core.wmem_max / rmem_max=4194304 — let a socket that asks for a 4 MB buffer keep it,
#        so a big frame goes in one write instead of waiting on ACKs.
#     4) FunctionFS debug logging OFF: echo 1 > /sys/kernel/debug/ipc_logging/f_fs/log_disable.
#        Qualcomm's FunctionFS writes four lines per USB request into a debug ring inside
#        io_submit; Jetlink measured io_submit 1.20 -> 0.88 ms and frames 21.14 -> 20.86 ms at p50
#        with it off. Stock is 0 (logging on).
#
# USAGE
#   bash 05_device_tuning.sh [user@]host            # READ-ONLY: print current values, change nothing
#   bash 05_device_tuning.sh [user@]host --apply    # USER-GATED: record originals, then apply
#   bash 05_device_tuning.sh [user@]host --revert   # USER-GATED: restore the recorded originals
#
# REVERT
#   `--revert` (above) restores the exact values `--apply` recorded (in
#   /dev/shm/offload-sysctl-prev on the device) and turns FunctionFS logging back on. A device
#   reboot also clears everything. Keep the record until you are done: --revert deletes it.
#
# PREREQS  Run FROM the Mac. Device reachable; ssh key loaded. Run during `OffloadMode=off`
#          (idle) so an apply never lands mid-drive.
# EXPECTED READINGS (after --apply)
#   vm.extra_free_kbytes == 32768; dirty caps == the tuned values (or the ratio fallback);
#   net.core.wmem_max == net.core.rmem_max == 4194304; f_fs log_disable == 1.
# USER-VISIBLE EFFECTS  (state CHANGE — user-gated)
#   * Writes four-to-six kernel sysctls (system-wide, until reboot or --revert).
#   * Switches FunctionFS's debug ring off (debugfs only).
#   * No /data writes; the record lives in /dev/shm (RAM, cleared on reboot).
# ============================================================================
set -euo pipefail

HOST="${1:-comma@comma-b203ed6e.local}"
ACTION="${2:-show}"
if [[ "$HOST" == -* || -z "$HOST" ]]; then
  echo "usage: bash 05_device_tuning.sh [user@]host [--apply|--revert]" >&2; exit 2
fi
case "$ACTION" in
  show|--apply|--revert) ;;
  *) echo "unknown action '$ACTION' (want --apply or --revert)" >&2; exit 2 ;;
esac

DEV_PY="/usr/local/venv/bin/python3"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "$HOST")

# The knobs, mirrored from jetlink-root.sh: FFS_LOG_OFF (:93) and VM_SYSCTLS (:127-128).
PROC_SYS="/proc/sys"
FFS_LOG_OFF="/sys/kernel/debug/ipc_logging/f_fs/log_disable"
SYSCTL_PREV="/dev/shm/offload-sysctl-prev"
# bytes-mode dirty caps (preferred) and the ratio-mode equivalents used only as a fallback.
BYTES_SYSCTLS=(vm.dirty_bytes=16777216 vm.dirty_background_bytes=8388608)
RATIO_SYSCTLS=(vm.dirty_ratio=20 vm.dirty_background_ratio=10)
OTHER_SYSCTLS=(vm.extra_free_kbytes=32768 net.core.wmem_max=4194304 net.core.rmem_max=4194304)

echo ">> target: $HOST   action: $ACTION"
"${SSH[@]}" 'echo reachable: $(uname -sr)' || { echo "ssh failed — load your key / check mDNS"; exit 1; }

# --- read-only: what are the values right now? ------------------------------
echo ">> [1/3] current values (read-only)"
"${SSH[@]}" "PROC_SYS='$PROC_SYS' FFS_LOG_OFF='$FFS_LOG_OFF' SYSCTL_PREV='$SYSCTL_PREV' bash -s" <<'REMOTE'
set -u
show() { local k="$1" f="$PROC_SYS/${1//.//}"; printf '   %-26s = %s\n' "$k" "$(cat "$f" 2>/dev/null || echo '<absent>')"; }
for k in vm.extra_free_kbytes vm.dirty_bytes vm.dirty_background_bytes vm.dirty_ratio vm.dirty_background_ratio \
         net.core.wmem_max net.core.rmem_max; do show "$k"; done
printf '   %-26s = %s\n' "f_fs log_disable" "$(cat "$FFS_LOG_OFF" 2>/dev/null || echo '<absent>')"
if [ -e "$SYSCTL_PREV" ]; then echo "   record: $SYSCTL_PREV present (a previous apply is recorded)"; else echo "   record: none"; fi
REMOTE

if [[ "$ACTION" == "show" ]]; then
  echo "OK — read-only. Re-run with --apply to change, --revert to restore a recorded apply."
  exit 0
fi

# --- apply / revert ----------------------------------------------------------
if [[ "$ACTION" == "--apply" ]]; then
  echo ">> [2/3] recording originals + applying (USER-GATED)"
  "${SSH[@]}" "PROC_SYS='$PROC_SYS' FFS_LOG_OFF='$FFS_LOG_OFF' SYSCTL_PREV='$SYSCTL_PREV' \
    BYTES='${BYTES_SYSCTLS[*]}' RATIO='${RATIO_SYSCTLS[*]}' OTHER='${OTHER_SYSCTLS[*]}' bash -s" <<'REMOTE'
set -u
failed=0
read_val() { cat "$PROC_SYS/${1//.//}" 2>/dev/null || echo ""; }
write_val() { if ! err=$( { echo "$2" > "$PROC_SYS/${1//.//}"; } 2>&1 ); then echo "   FAIL $1=$2${err:+: ${err##*: }}"; failed=1; else echo "   set  $1=$2"; fi; }

# Pick the dirty-cap mode: bytes if vm.dirty_bytes is writable, else the ratio fallback.
mode="bytes"
if [ "$(read_val vm.dirty_bytes)" = "0" ] && [ "$(read_val vm.dirty_bytes)" != "" ]; then
  mode="bytes"                # stock ratio mode still exposes a bytes key: prefer bytes
fi
# Record originals once (a second apply must not overwrite the first record with our own values).
if [ ! -e "$SYSCTL_PREV" ]; then
  rec=""
  for pair in $BYTES $OTHER; do
    k=${pair%%=*}; v=$(read_val "$k")
    # ratio-mode stock: the bytes key reads 0; record the ratio key we would have to zero instead.
    if [ "$v" = "0" ] && [ "$k" = *_bytes ]; then k=${k%_bytes}_ratio; v=$(read_val "$k"); fi
    [ -n "$v" ] && rec+="$k=$v"$'\n' || echo "   WARN could not read $k"
  done
  printf '%s' "$rec" > "$SYSCTL_PREV" 2>/dev/null && chmod 0644 "$SYSCTL_PREV" || echo "   WARN could not write $SYSCTL_PREV"
fi

if [ "$mode" = "bytes" ]; then
  for pair in $BYTES; do write_val "${pair%%=*}" "${pair#*=}"; done
else
  echo "   vm.dirty_bytes absent -> applying ratio caps instead"
  for pair in $RATIO; do write_val "${pair%%=*}" "${pair#*=}"; done
fi
for pair in $OTHER; do write_val "${pair%%=*}" "${pair#*=}"; done

# FunctionFS debug log off (1 turns it off). Absent knob is a warning, not a failure.
if [ -e "$FFS_LOG_OFF" ]; then
  if err=$( { echo 1 > "$FFS_LOG_OFF"; } 2>&1 ); then echo "   set  f_fs log_disable=1"; else echo "   FAIL f_fs log_disable${err:+: ${err##*: }}"; failed=1; fi
else
  echo "   WARN $FFS_LOG_OFF absent (debugfs not mounted?) — skipped"
fi
exit $failed
REMOTE
else
  echo ">> [2/3] restoring recorded originals (USER-GATED)"
  "${SSH[@]}" "PROC_SYS='$PROC_SYS' FFS_LOG_OFF='$FFS_LOG_OFF' SYSCTL_PREV='$SYSCTL_PREV' bash -s" <<'REMOTE'
set -u
failed=0
if [ ! -e "$SYSCTL_PREV" ]; then echo "   no record at $SYSCTL_PREV — nothing to restore (reboot clears anyway)"; exit 0; fi
while IFS='=' read -r k v; do
  if [[ "$k" =~ ^(vm|net\.core)\.[a-z_]+$ && "$v" =~ ^[0-9]+$ ]]; then
    if ! err=$( { echo "$v" > "$PROC_SYS/${k//.//}"; } 2>&1 ); then echo "   FAIL restore $k=$v${err:+: ${err##*: }}"; failed=1; else echo "   restored $k=$v"; fi
  else
    echo "   skip malformed record line '$k=$v'"; failed=1
  fi
done < "$SYSCTL_PREV"
[ -e "$FFS_LOG_OFF" ] && { echo 0 > "$FFS_LOG_OFF" 2>/dev/null && echo "   restored f_fs log_disable=0 (logging back on)" || echo "   WARN could not restore f_fs log_disable"; }
rm -f "$SYSCTL_PREV" 2>/dev/null || true
exit $failed
REMOTE
fi

echo ">> [3/3] verify"
"${SSH[@]}" "PROC_SYS='$PROC_SYS' FFS_LOG_OFF='$FFS_LOG_OFF' SYSCTL_PREV='$SYSCTL_PREV' bash -s" <<'REMOTE'
set -u
show() { printf '   %-26s = %s\n' "$1" "$(cat "$PROC_SYS/${1//.//}" 2>/dev/null || echo '<absent>')"; }
for k in vm.extra_free_kbytes vm.dirty_bytes vm.dirty_ratio net.core.wmem_max net.core.rmem_max; do show "$k"; done
printf '   %-26s = %s\n' "f_fs log_disable" "$(cat "$FFS_LOG_OFF" 2>/dev/null || echo '<absent>')"
REMOTE
if [[ "$ACTION" == "--apply" ]]; then
  echo "DONE — tuned. RE-APPLY after any AGNOS update; revert with: bash 05_device_tuning.sh $HOST --revert"
else
  echo "DONE — reverted."
fi
