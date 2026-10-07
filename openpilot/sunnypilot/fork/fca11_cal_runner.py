#!/usr/bin/env python3
"""
Fork: FCA11 calibration-drive runner (adurham/sunnypilot; opendbc patch 0029
``0029-hyundai-fca11-cal-command-mode``).

WHAT THIS IS / IS NOT
---------------------
The ACTUATION lives in the car layer (``opendbc/sunnypilot/car/hyundai/cal_mode.py``): an
autonomous sequencer ticked at 100 Hz that reads ``/data/fca11-cal/plan.json`` and executes due
reps opportunistically whenever the owner is driving with the FCA11 toggle ON. **This script is
NOT in the actuation path and is NOT required for collection.** It is the operator's tool:

  * ``--write-plan``  write /data/fca11-cal/plan.json (the ONE-TIME setup + offline gap-fill
    revisions). The rep matrix is deterministic and lives here so it is reviewable + reproducible.
  * ``--status``      read-only progress monitor (plan validity, inert reason, reps done/total).
  * ``--tail``        follow /data/fca11-cal/reps.jsonl (the per-rep log the car layer appends).
  * ``--dry-run``     offline pacing validator: walk the whole sequence against a synthetic clock
    with NO state writes, and print the schedule (per-rep start/hold/release/recovery + the total
    active time). Validates the matrix + timing off-car.

The owner just drives; the FCA11 UI toggle is the stop switch; progress persists across reboots.

PROTOCOL (CAL-DRIVE-CONTRACT (b)(c)(d))
--------------------------------------
  steps     : ladder {2,3,5,8,10,12,15,20,25,30} LSB, >=3 reps each, >=5 at 10 and 30 LSB
              (0.10 g / 0.30 g), two speed bands (~60 and ~90 km/h); each hold 2.2 s.
  staircase : 2 -> 30 LSB up AND down in ~2-3 LSB steps (one clean plateau rep per level), then a
              repeat of the two extremes.
  releases  : hold 15 LSB -> release x5; hold 30 LSB -> release x3; partial 20 -> 10 LSB x3 (a
              mid-hold step-down, the 0029 release-fade/partial-release test).

1 LSB = 0.01 g of CR_VSM_DecCmd; 30 LSB = 0.30 g (the panda gate).

SAFETY
------
* ``--write-plan`` writes atomically (tmp + os.replace) and refuses to overwrite an existing
  DIFFERENT revision without ``--force`` (so a gap-fill revision is an explicit, reviewed act).
* Every mode except ``--write-plan`` is READ-ONLY (no file writes at all).
* There is no code path here that sends CAN, touches the panda, or needs the device.
"""
import argparse
import json
import os
import sys
import time

CAL_DIR = os.environ.get("FCA11_CAL_DIR", "/data/fca11-cal")
PLAN_PATH = os.path.join(CAL_DIR, "plan.json")
PROGRESS_PATH = os.path.join(CAL_DIR, "progress.json")
LOG_PATH = os.path.join(CAL_DIR, "reps.jsonl")

# ---- protocol constants (must match cal_mode defaults / the contract) ----
DEFAULT_TTL_S = None    # None -> derive from expires_days: under the autonomous model the plan
                        # stays valid until it EXPIRES (a short ttl would silently stale the plan
                        # minutes after writing it and no rep would ever run)
DEFAULT_EXPIRES_DAYS = 30.0
DEFAULT_PER_DRIVE_CAP = 15
DEFAULT_HOLD_S = 2.2
BAND_LO_KPH = 60.0
BAND_HI_KPH = 90.0
STAIRCASE_STEP_LSB = 3          # ~2-3 LSB per step

# the ladder (b): {2,3,5,8,10,12,15,20,25,30} LSB; >=5 reps at the two extremes, >=3 elsewhere
LADDER_LSB = [2, 3, 5, 8, 10, 12, 15, 20, 25, 30]
EXTREME_REPS = 5                # 10 and 30 LSB
MID_REPS = 3                    # everything else in the ladder


def _mk_rep(rid, phase, level, band, hold=DEFAULT_HOLD_S, level2=None, switch_at=None):
  r = {"id": rid, "phase": phase, "band_kph": float(band), "level_lsb": int(level), "hold_s": float(hold)}
  if level2 is not None:
    r["level2_lsb"] = int(level2)
    r["switch_at_s"] = float(switch_at) if switch_at is not None else float(hold) / 2.0
  return r


def build_steps(bands=(BAND_LO_KPH, BAND_HI_KPH)):
  """(b) the level ladder. The two extremes get EXTREME_REPS; the >=3 mid reps are taken in the LOW
  band, plus a band-pair pass at 20 LSB in the HIGH band (contract (b): two speed bands, >=3 reps
  each at 0.20 g)."""
  reps = []
  for lv in LADDER_LSB:
    n = EXTREME_REPS if lv in (10, 30) else MID_REPS
    for i in range(n):
      reps.append(_mk_rep(f"step-{lv}-{bands[0]:.0f}-{i}", "steps", lv, bands[0]))
  # a band pass at 20 LSB in the HIGH band (the contract's two-band 0.20 g requirement)
  for i in range(3):
    reps.append(_mk_rep(f"step-20-{bands[1]:.0f}-{i}", "steps", 20, bands[1]))
  return reps


def build_staircase(bands=(BAND_LO_KPH,)):
  """(c) 2 -> 30 up, then 30 -> 2 down, ~2-3 LSB per step (one clean plateau rep per level; a
  documented choice -- see --dry-run output), then a repeat of the two extremes (2 and 30)."""
  up = list(range(2, 30 + 1, STAIRCASE_STEP_LSB))
  if up[-1] != 30:
    up.append(30)
  down = list(reversed(up))
  reps = []
  for lv in up:
    reps.append(_mk_rep(f"stair-up-{lv}", "staircase", lv, bands[0]))
  for lv in down:
    reps.append(_mk_rep(f"stair-down-{lv}", "staircase", lv, bands[0]))
  for lv in (2, 30):                       # the staircase repeatability pass
    reps.append(_mk_rep(f"stair-rep-{lv}", "staircase", lv, bands[0]))
  return reps


def build_releases(bands=(BAND_LO_KPH,)):
  """(d) release tests: from a 15 LSB and a 30 LSB plateau release-to-zero x5 / x3, plus the partial
  release 20 -> 10 LSB x3 (a mid-hold step-down; release goes through the production fade)."""
  reps = []
  for i in range(5):
    reps.append(_mk_rep(f"rel-15-{i}", "releases", 15, bands[0]))
  for i in range(3):
    reps.append(_mk_rep(f"rel-30-{i}", "releases", 30, bands[0]))
  for i in range(3):
    reps.append(_mk_rep(f"rel-part-20-10-{i}", "releases", 20, bands[0], hold=DEFAULT_HOLD_S, level2=10,
                        switch_at=DEFAULT_HOLD_S / 2.0))
  return reps


def build_plan(phases, rev=1, issued_at=None, ttl_s=DEFAULT_TTL_S, expires_days=DEFAULT_EXPIRES_DAYS,
               per_drive_cap=DEFAULT_PER_DRIVE_CAP, cond=None):
  reps = []
  if "steps" in phases:
    reps += build_steps()
  if "staircase" in phases:
    reps += build_staircase()
  if "releases" in phases:
    reps += build_releases()
  doc = {
    "rev": int(rev),
    "issued_at": float(issued_at if issued_at is not None else time.time()),
    "ttl_s": float(ttl_s) if ttl_s is not None else float(expires_days * 86400.0),
    "expires_at": float(time.time() + expires_days * 86400.0),
    "per_drive_cap": int(per_drive_cap),
    "conditions": {
      "min_lead_m": 60.0,
      "turn_lat_max": 0.5,
      "straight_min_s": 2.0,
      "band_tol_kph": 5.0,
      "band_stable_s": 5.0,
      "recovery_s": 3.5,
    },
    "reps": reps,
  }
  if cond:
    doc["conditions"].update(cond)
  return doc


# ------------------------------------------------------------------------------------------------
# --dry-run : offline pacing validator (NO state writes)
# ------------------------------------------------------------------------------------------------
def dry_run(phases, *, band_assumed=60.0, cooldown_s=3.5, recovery_s=2.0, start=0.0):
  """Walk the sequence against a synthetic clock: each rep = hold_s + cooldown + recovery (the
  band/lead/straight waits are real-drive events; here we only check the pacing arithmetic). Prints
  the schedule and the totals. No file writes."""
  doc = build_plan(phases)
  t = start
  rows = []
  for r in doc["reps"]:
    band = r["band_kph"]
    hold = r["hold_s"]
    step = ""
    if "level2_lsb" in r:
      step = f" (step {r['level_lsb']}->{r['level2_lsb']} @{r['switch_at_s']:.2f}s)"
    rows.append((r["id"], r["phase"], r["level_lsb"], band, t, t + hold, step))
    t += hold + cooldown_s + recovery_s
  total = t - start
  print(f"[dry-run] plan phases={sorted(phases)} reps={len(doc['reps'])} "
        f"assumed band={band_assumed:.0f} km/h cooldown={cooldown_s}s recovery={recovery_s}s")
  print(f"[dry-run] {'rep':28s} {'phase':10s} {'LSB':>4s} {'band':>5s} {'start':>8s} {'end':>8s}  step")
  for rid, ph, lv, band, s, e, step in rows:
    print(f"[dry-run] {rid:28s} {ph:10s} {lv:4d} {band:5.0f} {s:8.1f} {e:8.1f}  {step}")
  print(f"[dry-run] TOTAL active sequence approx {total:.1f} s ({total/60.0:.1f} min) across "
        f"{len(rows)} reps; per-rep rep time = hold {DEFAULT_HOLD_S}s + {cooldown_s}s cooldown + "
        f"{recovery_s}s recovery")
  print("[dry-run] NOTE: one clean plateau rep per staircase level is used instead of the "
        "contract's '4 steps x 0.55s per 2.2s episode'; the K(level) curve is finer this way and "
        "each plateau is uncontaminated by intra-episode steps.")
  print("[dry-run] NO state was written.")
  return rows


# ------------------------------------------------------------------------------------------------
# --write-plan / --status / --tail
# ------------------------------------------------------------------------------------------------
def cmd_write_plan(args):
  phases = ["steps", "staircase", "releases"] if "all" in args.phase else args.phase
  doc = build_plan(phases, rev=getattr(args, "rev", 1),
                   expires_days=args.expires_days, per_drive_cap=args.per_drive_cap)
  os.makedirs(CAL_DIR, exist_ok=True)
  if os.path.exists(PLAN_PATH) and not args.force:
    old = _load(PLAN_PATH)
    old_ids = sorted(r.get("id") for r in (old or {}).get("reps", []))
    new_ids = sorted(r["id"] for r in doc["reps"])
    if old is not None and old_ids == new_ids:
      print(f"plan.json already matches this matrix ({len(new_ids)} reps); nothing to do "
            f"(use --force to rewrite or bump --rev for a gap-fill revision).")
      return 0
    print(f"REFUSING to overwrite existing {PLAN_PATH} (different matrix: "
          f"{len(old_ids)} vs {len(new_ids)} reps). Use --force (or --rev N) if intended.")
    return 2
  tmp = PLAN_PATH + ".tmp"
  with open(tmp, "w") as f:
    json.dump(doc, f, indent=1, sort_keys=True)
  os.replace(tmp, PLAN_PATH)
  n = len(doc["reps"])
  print(f"wrote {PLAN_PATH}: rev {doc['rev']}, {n} reps, phases {sorted(phases)}, "
        f"ttl {doc['ttl_s']:.0f}s, per_drive_cap {doc['per_drive_cap']}, "
        f"expires {time.strftime('%Y-%m-%d', time.localtime(doc['expires_at']))}")
  print("ARM: only with the owner's FCA11 toggle ON. DISARM: toggle it off, delete the file, or let "
        "the TTL/expiry lapse.")
  return 0


def _load(path):
  try:
    with open(path) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def cmd_status(_args):
  plan = _load(PLAN_PATH)
  prog = _load(PROGRESS_PATH)
  print(f"cal dir: {CAL_DIR}")
  if plan is None:
    print("plan.json: ABSENT or MALFORMED -> cal mode is INERT (no actuation possible).")
  else:
    reps = plan.get("reps", [])
    issued = plan.get("issued_at")
    ttl = plan.get("ttl_s")
    exp = plan.get("expires_at")
    now = time.time()
    fresh = isinstance(ttl, (int, float)) and ttl > 0 and isinstance(issued, (int, float)) \
        and (now - issued) <= ttl
    expired = isinstance(exp, (int, float)) and now > exp
    iss = time.strftime('%Y-%m-%d %H:%M', time.localtime(issued)) if isinstance(issued, (int, float)) else issued
    print(f"plan.json: rev {plan.get('rev')}, {len(reps)} reps, ttl {ttl}, issued {iss}")
    print(f"           fresh={fresh} expired={expired} -> "
          f"{'VALID' if (fresh and not expired) else 'INERT (stale TTL or expired)'}")
    if expired:
      print(f"           expires_at {time.strftime('%Y-%m-%d', time.localtime(exp))} (PAST) -- "
              f"re-issue the plan (--write-plan --rev N+1) to collect more.")
    elif isinstance(exp, (int, float)):
      print(f"           expires_at {time.strftime('%Y-%m-%d', time.localtime(exp))} "
              f"({(exp - now) / 86400.0:.1f} days left)")
  if prog is None:
    print("progress.json: none yet (no reps recorded).")
  else:
    done = prog.get("reps", {})
    ok = [k for k, v in done.items() if isinstance(v, dict) and v.get("outcome") == "ok"]
    print(f"progress.json: {len(ok)} reps done" + ("  done=true (matrix complete)" if prog.get("done") else ""))
  if plan is not None:
    ids = [r.get("id") for r in plan.get("reps", [])]
    done_ids = {k for k, v in ((prog or {}).get("reps") or {}).items()
                if isinstance(v, dict) and v.get("outcome") == "ok"}
    remaining = [i for i in ids if i not in done_ids]
    print(f"remaining: {len(remaining)} reps"
          + (f" ({remaining[:8]}{'...' if len(remaining) > 8 else ''})" if remaining else ""))
  return 0


def cmd_tail(args):
  if not os.path.exists(LOG_PATH):
    print(f"no log yet at {LOG_PATH}")
    return 0
  with open(LOG_PATH) as f:
    lines = f.readlines()
  for ln in lines[-args.n:]:
    try:
      rec = json.loads(ln)
    except ValueError:
      print(ln.rstrip())
      continue
    ts = time.strftime("%H:%M:%S", time.localtime(rec.get("ts", 0)))
    print(f"{ts} {rec.get('event', '?'):12s} "
          f"{str(rec.get('rep', rec.get('plan_id', ''))):22s} "
          f"lvl={rec.get('level_lsb', '')} band={rec.get('band_kph', '')} "
          f"dir={rec.get('direction', '')} outcome={rec.get('outcome', '')}")
  return 0


# ------------------------------------------------------------------------------------------------
def main(argv=None):
  ap = argparse.ArgumentParser(description="FCA11 calibration-drive runner (plan writer / monitor). "
                                           "The actuation is in-car (cal_mode.py); this tool is not in that path.")
  g = ap.add_mutually_exclusive_group(required=True)
  g.add_argument("--write-plan", action="store_true", help="write /data/fca11-cal/plan.json (one-time setup / gap-fill)")
  g.add_argument("--status", action="store_true", help="read-only progress monitor")
  g.add_argument("--tail", action="store_true", help="tail the per-rep log")
  g.add_argument("--dry-run", action="store_true", help="offline pacing validator (no state writes)")
  ap.add_argument("--phase", nargs="+", choices=["steps", "staircase", "releases", "all"],
                  default=["all"], help="phase selector (default all)")
  ap.add_argument("--rev", type=int, default=1, help="plan revision (bump for a gap-fill revision)")
  ap.add_argument("--expires-days", type=float, default=DEFAULT_EXPIRES_DAYS)
  ap.add_argument("--per-drive-cap", type=int, default=DEFAULT_PER_DRIVE_CAP)
  ap.add_argument("--force", action="store_true", help="overwrite an existing different plan.json")
  ap.add_argument("-n", type=int, default=25, help="--tail: how many lines")
  args = ap.parse_args(argv)

  phases = ["steps", "staircase", "releases"] if "all" in args.phase else args.phase

  if args.write_plan:
    return cmd_write_plan(args)
  if args.status:
    return cmd_status(args)
  if args.tail:
    return cmd_tail(args)
  if args.dry_run:
    dry_run(phases)
    return 0
  return 0


if __name__ == "__main__":
  sys.exit(main())
