#!/usr/bin/env python3
"""
check_capnp_parity.py -- static guard: every opendbc SP dataclass field must have a
member in the matching openpilot cereal capnp struct.

Why this exists
---------------
Patch 0041 (2026-10-09) added ``fca11AffineGain`` to the ``CarParamsSP`` dataclass in
``opendbc/car/structs.py`` but not to ``struct CarParamsSP`` in
``openpilot/cereal/custom.capnp``.  ``card`` builds that message through
``openpilot/selfdrive/car/helpers.py:convert_to_capnp`` at every ignition, so the
mismatch raised ``capnp.lib.capnp.KjException: struct has no such member; name =
fca11AffineGain`` on every start: card crash-looped, no ``CarParams`` was ever
published, and openpilot showed "sunnypilot Unavailable / Waiting to start" for the
whole drive (route 154, 2026-10-09).  Nothing that runs before a deploy serializes
this structure -- the sims and unit tests exercise the dataclass alone, and ``card``
does not run offroad -- so the defect shipped.  This check closes that hole
mechanically: it runs in sync-upstream BEFORE the prebuilt tree is committed.

What it checks
--------------
For each of {CarParamsSP, CarStateSP, CarControlSP}: every field of the opendbc
dataclass must appear as a same-named field in the custom.capnp struct.  Extra capnp
fields are fine (old fields stay for log compatibility; that direction cannot crash).

Usage
-----
    python check_capnp_parity.py --tree <repo-or-prebuilt-tree-root>
Exit 0 = parity OK, 1 = mismatch (missing fields listed), 2 = usage/IO error.
Stdlib only -- runs in CI, on the Mac, and on the device.
"""
from __future__ import annotations

import argparse
import os
import re
import sys

SP_STRUCTS = ("CarParamsSP", "CarStateSP", "CarControlSP")


def capnp_fields(text: str, struct: str):
  # the struct block runs to the first column-0 closing brace (nested structs indent)
  m = re.search(rf"(?ms)^struct {re.escape(struct)} @0x[0-9a-fA-F]+ \{{\n(.*?)^\}}", text)
  if not m:
    return None
  return set(re.findall(r"(?m)^  ([A-Za-z_][A-Za-z0-9_]*)\s+@\d+", m.group(1)))


def dataclass_fields(text: str, cls: str):
  # the class block runs to the decorator/class header of the next top-level class;
  # top-level fields are the exactly-2-space ``name: ...`` lines (nested classes and
  # their members are 4-space indented; comments/decorators never match).
  m = re.search(rf"(?ms)^class {re.escape(cls)}:\n(.*?)(?=^@|^class |\Z)", text)
  if not m:
    return None
  return set(re.findall(r"(?m)^  ([A-Za-z_][A-Za-z0-9_]*):", m.group(1)))


def run(tree: str) -> int:
  tree = os.path.abspath(tree)
  capnp_path = os.path.join(tree, "openpilot", "cereal", "custom.capnp")
  if not os.path.isfile(capnp_path):
    print(f"ERROR: missing {capnp_path}")
    return 2
  structs_path = next((p for p in (
    os.path.join(tree, "opendbc", "car", "structs.py"),
    os.path.join(tree, "opendbc_repo", "opendbc", "car", "structs.py"),
  ) if os.path.isfile(p)), None)
  if structs_path is None:
    print(f"ERROR: missing opendbc structs.py under {tree}")
    return 2

  with open(capnp_path, encoding="utf-8") as fh:
    capnp_text = fh.read()
  with open(structs_path, encoding="utf-8") as fh:
    structs_text = fh.read()

  rc = 0
  for struct in SP_STRUCTS:
    df = dataclass_fields(structs_text, struct)
    sf = capnp_fields(capnp_text, struct)
    if df is None or sf is None:
      print(f"ERROR: could not parse {struct} (dataclass found={df is not None}, schema found={sf is not None})")
      rc = 2
      continue
    missing = sorted(df - sf)
    if missing:
      rc = 1
      print(f"FAIL {struct}: dataclass field(s) missing from custom.capnp: {missing}")
    else:
      print(f"ok   {struct}: all {len(df)} dataclass fields present in the schema")
  if rc == 1:
    print("card's convert_to_capnp would raise KjException at ignition -- add the missing field(s) to openpilot/cereal/custom.capnp")
  return rc


def main(argv=None):
  ap = argparse.ArgumentParser(description="dataclass<->capnp schema parity guard for the SP structs")
  ap.add_argument("--tree", required=True, help="repo or prebuilt-tree root to check")
  args = ap.parse_args(argv)
  return run(args.tree)


if __name__ == "__main__":
  sys.exit(main())
