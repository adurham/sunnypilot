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
  * ``--why``         per-rep "is this a redo, and why" report, joined against the rev-1
    validated-progress analysis (``rep_validity.csv``). Read-only.

The owner just drives; the FCA11 UI toggle is the stop switch; progress persists across reboots.

REV-2 MATRIX (``--matrix rev2``, default)
-----------------------------------------
The rev-1 matrix (rev 1) was 72 reps over a {2..30} LSB ladder, a 24-rep staircase and an 11-rep
release set. Driving it produced 16 "ok" reps of which only 1 was actually fit-valid, for two
reasons the car data proves (harvest-14b P0, ``out/rep_validity.csv``):

  * 7 of 16 were REFUSED end-to-end (3 camera_owns latch @ 20/90, 2 stale act_active @ 5, 1 budget
    edge, 1 unmatched) and delivered nothing, yet were recorded as ok;
  * the whole {2,3,5,8} band is DEGENERATE: the production onset floor is 8 LSB, so every scripted
    level <= 8 measured exactly 8 on the wire (``cmd_peak == 8`` for levels 2/3/5/8) -- those reps
    cannot resolve a difference. The smallest the BITE fit can resolve is 12 LSB.

rev 2 applies Fable's rules: NOTHING below 12 LSB, spacing >= 4 LSB, and it REDOES the 15 invalid
rev-1 reps (fresh ids), so the whole matrix is ~24 reps at the levels the fit can actually use.

The shipped progress model keys completion on the REP ID (``cal_mode.CalSequencer._is_complete``):
a done rep never re-runs. Bumping ``rev`` alone does NOT reset progress, so rev 2 gives every rep a
FRESH id (``r2-*``); that is what makes the redo set actually re-run. ``plan_id`` is a hash of
{rev, reps} and is recorded for provenance/labelling, not used to gate completion.

PROTOCOL
--------
  steps     : the useful ladder {12,16,20,24,27,30} LSB @60 km/h (3 reps at the two extremes,
              2 elsewhere), plus a 2nd speed band at 20 and 28 LSB @90 km/h.
  staircase : 12 -> 30 -> 12 up AND down in 4-LSB steps (one clean plateau rep per level).
  releases  : hold 24 LSB -> release x2; partial 24 -> 12 LSB x2 (a mid-hold step-down).

1 LSB = 0.01 g of CR_VSM_DecCmd; 30 LSB = 0.30 g (the panda gate).

SAFETY
------
* ``--write-plan`` writes atomically (tmp + os.replace) and refuses to overwrite an existing
  DIFFERENT revision without ``--force`` (so a gap-fill revision is an explicit, reviewed act).
* Every mode except ``--write-plan`` is READ-ONLY (no file writes at all).
* There is no code path here that sends CAN, touches the panda, or needs the device.
"""
import argparse
import hashlib
import json
import os
import sys
import time

CAL_DIR = os.environ.get("FCA11_CAL_DIR", "/data/fca11-cal")
PLAN_PATH = os.path.join(CAL_DIR, "plan.json")
PROGRESS_PATH = os.path.join(CAL_DIR, "progress.json")
LOG_PATH = os.path.join(CAL_DIR, "reps.jsonl")

# where the validated-progress analysis lives (produced by cal-refresh/scripts/rep_validity.py)
HERE = os.path.dirname(os.path.abspath(__file__))
VALIDITY_CANDIDATES = [
  os.environ.get("FCA11_VALIDITY_CSV", ""),
  os.path.join(HERE, "..", "..", "..", "..", "..", "car-features", "cal-refresh", "out", "rep_validity.csv"),
  os.path.join(HERE, "cal-refresh", "out", "rep_validity.csv"),
]

# ---- protocol constants (must match cal_mode defaults / the contract) ----
DEFAULT_TTL_S = None    # None -> derive from expires_days: under the autonomous model the plan
                        # stays valid until it EXPIRES (a short ttl would silently stale the plan
                        # minutes after writing it and no rep would ever run)
DEFAULT_EXPIRES_DAYS = 30.0
DEFAULT_PER_DRIVE_CAP = 15
DEFAULT_HOLD_S = 2.2
BAND_LO_KPH = 60.0
BAND_HI_KPH = 90.0
STAIRCASE_STEP_LSB = 3          # ~2-3 LSB per step (rev 1)

# rev 1: the ladder {2,3,5,8,10,12,15,20,25,30} LSB; >=5 reps at the two extremes, >=3 elsewhere
LADDER_LSB = [2, 3, 5, 8, 10, 12, 15, 20, 25, 30]
EXTREME_REPS = 5                # 10 and 30 LSB
MID_REPS = 3                    # everything else in the ladder

# ---- rev 2: Fable's rules. NOTHING <= 8 LSB (the production onset floor is 8, so every level <= 8
# measures exactly 8 on the wire); nothing <= 12 (the smallest level the BITE fit can resolve);
# adjacent levels >= 4 LSB apart. ----
R2_MIN_LSB = 12
R2_SPACING_LSB = 4
R2_LADDER = [12, 16, 20, 24, 30]        # the fit-resolvable ladder @60: gaps 4,4,4,6 (all >= 4);
                                       # 12 = smallest resolvable, 24 = the release level, 30 = the panda gate
R2_REPS_PER_LEVEL = 1                  # one clean plateau per rung @60 (the staircase repeats them)
# rev 3 (review guidance: 'prefer high-g reps at the lower speed band -- both crash energy and
# follower surprise scale with speed'): the 90 km/h band keeps ONLY its existing 20-LSB level (2 reps,
# unchanged); the high-g 28-LSB rep MOVES to the 60 km/h band. Total stays 24 reps. The moved rep gets
# a FRESH id (``r3-step-28-60-0``) because completion keys on rep id -- the unchanged r2-* reps keep
# their recorded state, and only the moved rep re-runs.
R2_BANDS_HI = ((20, 2),)               # 2nd speed band @90 km/h: 20 LSB x2 (only; the 28 LSB moved out)
R2_MOVED_28_BAND = 60.0                # the 28-LSB rep's NEW band (was 90)
R2_MOVED_28_ID = "r3-step-28-60-0"     # fresh id -> it RUNS even if a rev-2 plan recorded an r2 28 rep
R2_PLAN_REV = 4                        # rev 4: adds the 0033 driver-consent (tap-to-fire) config
# 0033 driver consent ("tap to fire"): the default the builder emits. "off" = the backward-
# compatible autonomous behaviour; "all" / "high" require a driver tap per rep (see cal_mode.py).
# min_level_lsb 22 = 0.22 g: at/above it a rep is "high-g" and (in "high" mode) needs a tap.
CONSENT_MODE_DEFAULT = "off"
CONSENT_MIN_LEVEL_LSB = 22
CONSENT_TIMEOUT_S = 8.0
R2_STAIR = [12, 16, 20, 24, 30]         # up + down sweep: 5 clean plateaus each way
R2_REL_RELEASE_LSB = 24                # release-from-24 plateau x3 (release-to-zero repeatability)
R2_REL_PART = (24, 12)                 # partial release 24 -> 12 x3


def _mk_rep(rid, phase, level, band, hold=DEFAULT_HOLD_S, level2=None, switch_at=None):
  r = {"id": rid, "phase": phase, "band_kph": float(band), "level_lsb": int(level), "hold_s": float(hold)}
  if level2 is not None:
    r["level2_lsb"] = int(level2)
    r["switch_at_s"] = float(switch_at) if switch_at is not None else float(hold) / 2.0
  return r


def _conditions(consent_mode=CONSENT_MODE_DEFAULT, consent_min_level=CONSENT_MIN_LEVEL_LSB,
                consent_timeout=CONSENT_TIMEOUT_S):
  return {"min_lead_m": 60.0, "turn_lat_max": 0.5, "straight_min_s": 2.0,
          "band_tol_kph": 5.0, "band_stable_s": 5.0, "recovery_s": 3.5,
          # 0032 blind-spot veto: fail-closed default ON; 5 s hold-off after the last set sample.
          "blindspot_veto": True, "blindspot_holdoff_s": 5.0,
          # 0033 driver consent (tap-to-fire). "off" is the compat default; a collection drive that
          # wants a per-rep tap sets "all" (every rep) or "high" (only reps >= min_level_lsb).
          "consent_mode": str(consent_mode), "consent_min_level_lsb": int(consent_min_level),
          "consent_timeout_s": float(consent_timeout)}


# ------------------------------------------------------------------------------------------------
# plan_id: a stable hash of {rev, reps} = the immutable matrix identity.
# ------------------------------------------------------------------------------------------------
def plan_id(obj: dict) -> str:
  """Stable id over the rep matrix + revision; a NEW revision hashes to a NEW id.

  IMPORTANT (verified in cal_mode.py ``_is_complete``): completion is keyed on the REP ID, not this
  hash -- a new rev that re-lists already-done ids does NOT re-run them. Rev 2 therefore gives
  every rep a FRESH id. This function is for provenance/labels only."""
  reps = obj.get("reps") or []
  key = json.dumps({"rev": obj.get("rev", 1), "reps": reps}, sort_keys=True, separators=(",", ":"))
  return hashlib.sha256(key.encode()).hexdigest()[:16]


# ------------------------------------------------------------------------------------------------
# rev 2 builders
# ------------------------------------------------------------------------------------------------
def build_steps_rev2(bands=(BAND_LO_KPH, BAND_HI_KPH)):
  """Rev-3 steps: the {12,16,20,24,30} ladder @60, R2_REPS_PER_LEVEL reps each; the high-g 28-LSB
  rep MOVED to the 60 km/h band (rev 3); plus the 2nd high-speed band pass (R2_BANDS_HI) @90."""
  reps = []
  for lv in R2_LADDER:
    for i in range(R2_REPS_PER_LEVEL):
      reps.append(_mk_rep(f"r2-step-{lv}-{bands[0]:.0f}-{i}", "steps", lv, bands[0]))
  # rev 3: the moved high-g rep (was 28 LSB @90) now lives at the LOWER speed band, fresh id.
  reps.append(_mk_rep(R2_MOVED_28_ID, "steps", 28, R2_MOVED_28_BAND))
  for lv, n in R2_BANDS_HI:
    for i in range(n):
      reps.append(_mk_rep(f"r2-step-{lv}-{bands[1]:.0f}-{i}", "steps", lv, bands[1]))
  return reps


def build_staircase_rev2(bands=(BAND_LO_KPH,)):
  """Rev-2 staircase: 12 -> 30 up in 4-LSB steps, one plateau rep per level (6 clean plateaus)."""
  up = list(R2_STAIR)
  reps = [_mk_rep(f"r2-stair-up-{lv}", "staircase", lv, bands[0]) for lv in up]
  reps += [_mk_rep(f"r2-stair-down-{lv}", "staircase", lv, bands[0]) for lv in reversed(up)]
  return reps


def build_releases_rev2(bands=(BAND_LO_KPH,)):
  """Rev-2 releases: from a 24 LSB plateau release-to-zero x3, plus the partial release 24 -> 12 LSB
  x3 (a mid-hold step-down through the production fade)."""
  reps = [_mk_rep(f"r2-rel-24-{i}", "releases", R2_REL_RELEASE_LSB, bands[0]) for i in range(3)]
  lv, lv2 = R2_REL_PART
  for i in range(3):
    reps.append(_mk_rep(f"r2-rel-part-{lv}-{lv2}-{i}", "releases", lv, bands[0], level2=lv2,
                        switch_at=DEFAULT_HOLD_S / 2.0))
  return reps


def _plan_doc(reps, rev, issued_at, ttl_s, expires_days, per_drive_cap, cond,
              consent_mode=CONSENT_MODE_DEFAULT, consent_min_level=CONSENT_MIN_LEVEL_LSB,
              consent_timeout=CONSENT_TIMEOUT_S):
  doc = {
    "rev": int(rev),
    "issued_at": float(issued_at if issued_at is not None else time.time()),
    "ttl_s": float(ttl_s) if ttl_s is not None else float(expires_days * 86400.0),
    "expires_at": float(time.time() + expires_days * 86400.0),
    "per_drive_cap": int(per_drive_cap),
    "conditions": _conditions(consent_mode, consent_min_level, consent_timeout),
    "reps": reps,
  }
  if cond:
    docp = doc["conditions"]
    docp.update(cond)
  return doc


def build_plan_rev2(phases, rev=R2_PLAN_REV, issued_at=None, ttl_s=DEFAULT_TTL_S, expires_days=DEFAULT_EXPIRES_DAYS,
                    per_drive_cap=DEFAULT_PER_DRIVE_CAP, cond=None,
                    consent_mode=CONSENT_MODE_DEFAULT, consent_min_level=CONSENT_MIN_LEVEL_LSB,
                    consent_timeout=CONSENT_TIMEOUT_S):
  reps = []
  if "steps" in phases:
    reps += build_steps_rev2()
  if "staircase" in phases:
    reps += build_staircase_rev2()
  if "releases" in phases:
    reps += build_releases_rev2()
  return _plan_doc(reps, rev, issued_at, ttl_s, expires_days, per_drive_cap, cond,
                   consent_mode, consent_min_level, consent_timeout)


# ------------------------------------------------------------------------------------------------
# rev 1 builders (provenance / --why only; do not collect more of this)
# ------------------------------------------------------------------------------------------------
def build_steps(bands=(BAND_LO_KPH, BAND_HI_KPH)):
  reps = []
  for lv in LADDER_LSB:
    n = EXTREME_REPS if lv in (10, 30) else MID_REPS
    for i in range(n):
      reps.append(_mk_rep(f"step-{lv}-{bands[0]:.0f}-{i}", "steps", lv, bands[0]))
  for i in range(3):
    reps.append(_mk_rep(f"step-20-{bands[1]:.0f}-{i}", "steps", 20, bands[1]))
  return reps


def build_staircase(bands=(BAND_LO_KPH,)):
  up = list(range(2, 30 + 1, STAIRCASE_STEP_LSB))
  if up[-1] != 30:
    up.append(30)
  down = list(reversed(up))
  reps = [_mk_rep(f"stair-up-{lv}", "staircase", lv, bands[0]) for lv in up]
  reps += [_mk_rep(f"stair-down-{lv}", "staircase", lv, bands[0]) for lv in down]
  reps += [_mk_rep(f"stair-rep-{lv}", "staircase", lv, bands[0]) for lv in (2, 30)]
  return reps


def build_releases(bands=(BAND_LO_KPH,)):
  reps = [_mk_rep(f"rel-15-{i}", "releases", 15, bands[0]) for i in range(5)]
  reps += [_mk_rep(f"rel-30-{i}", "releases", 30, bands[0]) for i in range(3)]
  for i in range(3):
    reps.append(_mk_rep(f"rel-part-20-10-{i}", "releases", 20, bands[0], level2=10,
                        switch_at=DEFAULT_HOLD_S / 2.0))
  return reps


def build_plan_rev1(phases, rev=1, issued_at=None, ttl_s=DEFAULT_TTL_S, expires_days=DEFAULT_EXPIRES_DAYS,
                    per_drive_cap=DEFAULT_PER_DRIVE_CAP, cond=None):
  """REV 1: the original 72-rep matrix, kept for provenance/--why."""
  reps = []
  if "steps" in phases:
    reps += build_steps()
  if "staircase" in phases:
    reps += build_staircase()
  if "releases" in phases:
    reps += build_releases()
  return _plan_doc(reps, rev, issued_at, ttl_s, expires_days, per_drive_cap, cond)


def build_matrix(matrix, phases, rev=None, **kwargs):
  if matrix == "rev1":
    return build_plan_rev1(phases, rev=1 if rev is None else rev, **kwargs)
  return build_plan_rev2(phases, rev=R2_PLAN_REV if rev is None else rev, **kwargs)


def build_plan(phases, *args, matrix="rev2", **kwargs):
  """Back-compat shim: the default matrix is now rev 2."""
  return build_matrix(matrix, phases, *args, **kwargs)


# ------------------------------------------------------------------------------------------------
# --dry-run : offline pacing validator (NO state writes)
# ------------------------------------------------------------------------------------------------
def dry_run(phases, matrix="rev2", *, band_assumed=60.0, cooldown_s=3.5, recovery_s=2.0, start=0.0):
  doc = build_matrix(matrix, phases)
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
  print(f"[dry-run] matrix={matrix} plan_id={plan_id(doc)} phases={sorted(phases)} reps={len(doc['reps'])} "
        f"assumed band={band_assumed:.0f} km/h cooldown={cooldown_s}s recovery={recovery_s}s")
  print(f"[dry-run] {'rep':28s} {'phase':10s} {'LSB':>4s} {'band':>5s} {'start':>8s} {'end':>8s}  step")
  for rid, ph, lv, band, s, e, step in rows:
    print(f"[dry-run] {rid:28s} {ph:10s} {lv:4d} {band:5.0f} {s:8.1f} {e:8.1f}  {step}")
  print(f"[dry-run] TOTAL active sequence approx {total:.1f} s ({total / 60.0:.1f} min) across "
        f"{len(rows)} reps; per-rep rep time = hold {DEFAULT_HOLD_S}s + {cooldown_s}s cooldown + "
        f"{recovery_s}s recovery")
  if matrix == "rev2":
    print("[dry-run] NOTE: one clean plateau rep per staircase level is used instead of the "
          "contract's '4 steps x 0.55s per 2.2s episode'; the K(level) curve is finer this way and "
          "each plateau is uncontaminated by intra-episode steps.")
  print("[dry-run] NO state was written.")
  return rows


# ------------------------------------------------------------------------------------------------
# --why : per-rep redo report (read-only)
# ------------------------------------------------------------------------------------------------
def _load_validity():
  import csv
  for p in VALIDITY_CANDIDATES:
    if p and os.path.exists(p):
      try:
        with open(p, newline="") as f:
          return {row["rep_id"]: row for row in csv.DictReader(f)}
      except (OSError, ValueError, KeyError):
        return {}
  return {}


def _redo_reason(rid, validity):
  """Why is this rep a redo? Join the rev-1 validated-progress classification when we can (the rev-1
  ids carry no ``r2-`` prefix), else state the rule for the new coverage."""
  if not rid.startswith("r2-"):
    v = validity.get(rid)
    if v:
      cls = v.get("class", "")
      if cls == "VALID":
        return "kept from rev 1 (VALID)"
      return f"redo: rev-1 was {cls} ({v.get('why', '')})"
    return "redo (rev-1 rep; no validity row)"
  return "new coverage (rev-2 matrix; level >= 12 LSB, spacing >= 4)"


def cmd_why(args):
  phases = [k for k in ("steps", "staircase", "releases") if "all" in args.phase or k in args.phase]
  # the report is ABOUT the rev-2 plan vs the rev-1 outcomes, so always build rev 2 here
  doc = build_matrix("rev2", phases)
  validity = _load_validity()
  print(f"matrix=rev2 rev={doc['rev']} plan_id={plan_id(doc)} reps={len(doc['reps'])}")
  if not validity:
    print("(rep_validity.csv not found; redo reasons come from the rev-2 rules only. "
          "Set FCA11_VALIDITY_CSV to cal-refresh/out/rep_validity.csv for the per-rep join.)")
  n_redo = 0
  for r in doc["reps"]:
    why = _redo_reason(r["id"], validity)
    if "new coverage" in why or why.startswith("redo"):
      n_redo += 1
    print(f"  {r['id']:24s} lvl={r['level_lsb']:3d} band={r['band_kph']:5.0f}  {why}")
  print(f"\n{n_redo}/{len(doc['reps'])} reps are new coverage or an explicit redo.")
  print("NOTE: completion keys on rep ID (cal_mode._is_complete), not plan_id; rev 2 uses fresh ids "
        "so the REDO set re-runs. Bumping rev alone does not reset progress.")
  return 0


# ------------------------------------------------------------------------------------------------
# --write-plan / --status / --tail
# ------------------------------------------------------------------------------------------------
def _load(path):
  try:
    with open(path) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def cmd_write_plan(args):
  phases = [k for k in ("steps", "staircase", "releases") if "all" in args.phase or k in args.phase]
  doc = build_matrix(args.matrix, phases, rev=args.rev,
                     expires_days=args.expires_days, per_drive_cap=args.per_drive_cap,
                     consent_mode=args.consent_mode, consent_min_level=args.consent_min_level,
                     consent_timeout=args.consent_timeout)
  os.makedirs(CAL_DIR, exist_ok=True)
  if os.path.exists(PLAN_PATH) and not args.force:
    old = _load(PLAN_PATH)
    old_ids = sorted(r.get("id") for r in (old or {}).get("reps", []))
    new_ids = sorted(r["id"] for r in doc["reps"])
    if old is not None and old_ids == new_ids:
      print(f"plan.json already matches this matrix ({len(new_ids)} reps, plan_id {plan_id(doc)}); "
            f"nothing to do (use --force to rewrite or bump --rev for a gap-fill revision).")
      return 0
    print(f"REFUSING to overwrite existing {PLAN_PATH} (different matrix: "
          f"{len(old_ids)} vs {len(new_ids)} reps). Use --force if intended.\n"
          f"  existing: rev {old.get('rev') if old else '?'} plan_id {plan_id(old) if old else '?'}\n"
          f"  new     : rev {doc['rev']} plan_id {plan_id(doc)}")
    return 2
  tmp = PLAN_PATH + ".tmp"
  with open(tmp, "w") as f:
    json.dump(doc, f, indent=1, sort_keys=True)
  os.replace(tmp, PLAN_PATH)
  n = len(doc["reps"])
  print(f"wrote {PLAN_PATH}: rev {doc['rev']}, {n} reps, phases {sorted(phases)}, "
        f"ttl {doc['ttl_s']:.0f}s, per_drive_cap {doc['per_drive_cap']}, "
        f"expires {time.strftime('%Y-%m-%d', time.localtime(doc['expires_at']))}")
  print(f"plan_id {plan_id(doc)}  (informational: completion is keyed on REP ID, not this hash)")
  print("ARM: only with the owner's FCA11 toggle ON. DISARM: toggle it off, delete the file, or let "
        "the TTL/expiry lapse.")
  return 0


def cmd_consent(args):
  """0033: operator helper for a MANUAL consent tap (used in the on-device smoke test). Writes /
  removes /data/fca11-cal/consent.json bound to the rep_id currently in pending.json. Read-only with
  respect to the plan; it can only ever request a rep the sequencer already parked."""
  pend = _load(os.path.join(CAL_DIR, "pending.json")) or {}
  rep_id = pend.get("rep_id")
  if not pend.get("pending") or not rep_id:
    print("no rep is pending (pending.json absent or pending=false) -- nothing to consent to.")
    return 0
  path = os.path.join(CAL_DIR, "consent.json")
  if args.consent == "clear":
    try:
      os.remove(path)
      print(f"cleared {path}")
    except OSError:
      print(f"{path} already absent")
    return 0
  doc = {"rep_id": rep_id, "ts": time.time(), "accept": args.consent == "accept"}
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    json.dump(doc, f, sort_keys=True)
  os.replace(tmp, path)
  print(f"wrote {path}: rep_id={rep_id} accept={doc['accept']} (consumed only if fresh <= 2 s)")
  return 0


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
    iss = time.strftime("%Y-%m-%d %H:%M", time.localtime(issued)) if isinstance(issued, (int, float)) else issued
    print(f"plan.json: rev {plan.get('rev')}, {len(reps)} reps, plan_id {plan_id(plan)}, ttl {ttl}, issued {iss}")
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
      match = "  (matches the current plan)" if prog.get("plan_id") == plan_id(plan) \
          else "  (does NOT match the current plan / matrix)"
      print(f"             progress.plan_id {prog.get('plan_id')}{match}")
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
  g.add_argument("--why", action="store_true", help="per-rep redo report (read-only)")
  g.add_argument("--consent", choices=["accept", "dismiss", "clear"],
                  help="0033: write a driver-consent tap (accept/dismiss) or clear it, for the cal "
                       "dir (operates on pending.json's rep_id; read-only w.r.t. the plan)")
  ap.add_argument("--matrix", choices=["rev1", "rev2"], default="rev2",
                  help="which matrix to build: rev2 (default, slimmed ~24 reps) or rev1 (the original 72)")
  ap.add_argument("--phase", nargs="+", choices=["steps", "staircase", "releases", "all"],
                  default=["all"], help="phase selector (default all)")
  ap.add_argument("--rev", type=int, default=None, help="plan revision (defaults to the matrix's own rev)")
  ap.add_argument("--expires-days", type=float, default=DEFAULT_EXPIRES_DAYS)
  ap.add_argument("--per-drive-cap", type=int, default=DEFAULT_PER_DRIVE_CAP)
  ap.add_argument("--force", action="store_true", help="overwrite an existing different plan.json")
  ap.add_argument("--consent-mode", choices=["off", "all", "high"], default=CONSENT_MODE_DEFAULT,
                  help="0033 driver consent: off (autonomous, default) / all / high")
  ap.add_argument("--consent-min-level", type=int, default=CONSENT_MIN_LEVEL_LSB,
                  help="0033: in 'high' mode, reps at/above this LSB need a tap (default 22 = 0.22 g)")
  ap.add_argument("--consent-timeout", type=float, default=CONSENT_TIMEOUT_S,
                  help="0033: seconds an untouched prompt waits before it is dropped (default 8)")
  ap.add_argument("-n", type=int, default=25, help="--tail: how many lines")
  args = ap.parse_args(argv)

  if args.rev is None:
    args.rev = R2_PLAN_REV if args.matrix == "rev2" else 1

  phases = [k for k in ("steps", "staircase", "releases") if "all" in args.phase or k in args.phase]

  if args.write_plan:
    return cmd_write_plan(args)
  if args.status:
    return cmd_status(args)
  if args.tail:
    return cmd_tail(args)
  if args.why:
    return cmd_why(args)
  if args.consent:
    return cmd_consent(args)
  if args.dry_run:
    dry_run(phases, args.matrix)
    return 0
  return 0


if __name__ == "__main__":
  sys.exit(main())
