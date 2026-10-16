#!/usr/bin/env python3
"""
check_patch_numbers.py -- guard against a DUPLICATE numeric patch prefix.

Why this exists
---------------
The deployed opendbc patch series is applied by CI in `LC_ALL=C` filename order
(.github/workflows/sync-upstream.yaml, "Apply the fork's opendbc patch series"). Two
patches sharing a numeric prefix -- which happened once: a DEPLOYED
`0029-...cal-command-mode` and a HELD `0029-...brake-shaping` -- makes the series
ambiguous to a human and to any tooling that keys on the number, and already caused a
real briefing error. This guard fails the moment two patches share a `NNNN-` prefix.

What it scans
-------------
* The SERIES directory (the one CI applies): every `*.patch` here MUST carry a
  `NNNN-` prefix, and no two may share it.
* Zero or more HELD directories (`--held`, repeatable): `*.patch` files here may be
  numbered or not; if numbered, their prefixes must also be unique WITHIN the dir.
  A held patch that carries no prefix is fine (and is what the held brake-shaping v1
  does deliberately, so it can never re-collide with the series).

Usage
-----
    python check_patch_numbers.py                     # scan this repo's series dir
    python check_patch_numbers.py --series DIR
    python check_patch_numbers.py --series DIR --held DIR1 --held DIR2

Exit 0 = clean, 1 = duplicate (or a series patch missing its prefix), 2 = usage/IO error.
Stdlib only. Also importable: find_patch_numbers(paths, require_numbered=...).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

PREFIX_RE = re.compile(r"^(\d{4})-")


def _patches(directory: Path) -> list[Path]:
  if not directory.is_dir():
    return []
  # same non-recursive glob CI uses: `find <dir> -maxdepth 1 -name '*.patch' | LC_ALL=C sort`
  return sorted((p for p in directory.iterdir() if p.is_file() and p.suffix == ".patch"),
                key=lambda p: p.name.encode("utf-8"))


def find_patch_numbers(directories: list[Path], require_numbered: bool = True) -> list[tuple[str, Path]]:
  """Return [(prefix, path)] for every numbered patch under `directories`.

  Raises ValueError on (a) a duplicate prefix within the SAME directory, or
  (b) a .patch lacking a `NNNN-` prefix while require_numbered is True.
  """
  out: list[tuple[str, Path]] = []
  for d in directories:
    seen: dict[str, Path] = {}
    for p in _patches(d):
      m = PREFIX_RE.match(p.name)
      if m is None:
        if require_numbered:
          raise ValueError(f"{p}: patch has no NNNN- numeric prefix (required in a series dir)")
        continue
      prefix = m.group(1)
      if prefix in seen:
        raise ValueError(f"DUPLICATE patch number {prefix}: "
                         f"{seen[prefix].name}  vs  {p.name}  (in {d})")
      seen[prefix] = p
      out.append((prefix, p))
  return out


def _default_series() -> Path:
  # this file lives at <repo>/openpilot/sunnypilot/fork/tests/check_patch_numbers.py
  return Path(__file__).resolve().parents[1] / "patches"


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description="Fail if two patches share a numeric prefix.")
  ap.add_argument("--series", type=Path, default=_default_series(),
                  help="the applied patch directory (default: this repo's fork/patches)")
  ap.add_argument("--held", type=Path, action="append", default=[],
                  help="a held-patch directory (repeatable); numbered patches must be unique within it")
  args = ap.parse_args(argv)

  if not args.series.is_dir():
    print(f"::error::series dir not found: {args.series}", file=sys.stderr)
    return 2

  try:
    series = find_patch_numbers([args.series], require_numbered=True)
    held = find_patch_numbers(args.held, require_numbered=False)
  except ValueError as e:
    print(f"::error::patch-number guard FAILED: {e}", file=sys.stderr)
    return 1

  print(f"series: {args.series}  -> {len(series)} numbered patch(es), all prefixes unique")
  for d in args.held:
    print(f"held:   {d}  -> {len(find_patch_numbers([d], require_numbered=False))} numbered patch(es), unique")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
