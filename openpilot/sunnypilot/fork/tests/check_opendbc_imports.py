#!/usr/bin/env python3
"""Static guard: every name the superproject imports FROM opendbc must exist in the opendbc tree.

Why this exists
  The 922513f312 sync removed `STD_CARGO_KG` from `opendbc.car`, and the fork's
  `openpilot/sunnypilot/fork/vehicle_specs.py` still imported it. Nothing static caught it:
  the capnp-parity and param-reachability gates look at schemas/params, and the opendbc test
  suites never import the superproject. The failure only appeared on the device, where the UI
  process crash-looped on `ImportError` and the car's screen went dead.

What it does (stdlib only, no imports of the target tree - safe on the arm64 prebuilt clone)
  * scans every .py in the superproject for `from opendbc... import a, b as c` and
    `from opendbc... import *`, plus `import opendbc.x.y as z`
  * resolves the module against the opendbc source tree and collects its top-level names
    (module-level assignments, defs, classes, and names re-exported from other modules)
  * reports every imported name that the module does not define

Usage
  check_opendbc_imports.py --tree <prebuilt-or-source-tree> [--opendbc <opendbc-root>]
  exit 0 = clean, 1 = missing names found, 2 = usage/setup error
"""
from __future__ import annotations

import argparse
import ast
import os
import sys
from pathlib import Path

SKIP_DIRS = {".git", "__pycache__", "node_modules", ".venv", "third_party"}


def iter_py(root: Path):
  for dirpath, dirnames, filenames in os.walk(root):
    dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
    for fn in filenames:
      if fn.endswith(".py"):
        yield Path(dirpath) / fn


def module_file(odb: Path, modpath: str) -> Path | None:
  """resolve 'opendbc.car.x' to a file under the opendbc root."""
  parts = modpath.split(".")
  if parts[0] != "opendbc":
    return None
  rel = odb.joinpath(*parts)
  if (rel / "__init__.py").is_file():
    return rel / "__init__.py"
  if rel.with_suffix(".py").is_file():
    return rel.with_suffix(".py")
  return None


def top_level_names(odb: Path, modpath: str, seen: set[str] | None = None) -> set[str] | None:
  """names importable from the module: local definitions plus its own re-exports (one level, recursive)."""
  seen = seen or set()
  if modpath in seen:
    return set()
  seen.add(modpath)
  f = module_file(odb, modpath)
  if f is None:
    return None
  try:
    tree = ast.parse(f.read_text(), filename=str(f))
  except (SyntaxError, UnicodeDecodeError, OSError):
    return None

  names: set[str] = set()
  for node in tree.body:
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
      names.add(node.name)
    elif isinstance(node, ast.Assign):
      for tgt in node.targets:
        if isinstance(tgt, ast.Name):
          names.add(tgt.id)
        elif isinstance(tgt, (ast.Tuple, ast.List)):
          names.update(e.id for e in tgt.elts if isinstance(e, ast.Name))
    elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
      names.add(node.target.id)
    elif isinstance(node, ast.If):     # typing.TYPE_CHECKING / version guards
      for sub in node.body + node.orelse:
        if isinstance(sub, (ast.FunctionDef, ast.ClassDef)):
          names.add(sub.name)
        elif isinstance(sub, ast.Assign):
          names.update(t.id for t in sub.targets if isinstance(t, ast.Name))
    elif isinstance(node, ast.ImportFrom):
      src = resolve_relative(modpath, node)
      if src is None:
        continue
      if any(a.name == "*" for a in node.names):
        names |= top_level_names(odb, src, seen) or set()
      else:
        names.update(a.asname or a.name for a in node.names)
    elif isinstance(node, ast.Import):
      for a in node.names:
        names.add(a.asname or a.name.split(".")[0])
  return names


def resolve_relative(modpath: str, node: ast.ImportFrom) -> str | None:
  if not node.level:
    return node.module
  base = modpath.split(".")
  if node.module:
    base = base[: len(base) - node.level] + node.module.split(".")
  else:
    base = base[: len(base) - node.level]
  return ".".join(base) if base else None


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--tree", required=True, help="superproject tree to scan (prebuilt clone or a checkout)")
  ap.add_argument("--all", action="store_true",
                  help="scan the whole tree (default: only the production <tree>/openpilot subtree; "
                       "tools/ scripts lag upstream renames and are not shipped to the device)")
  ap.add_argument("--opendbc", default=None, help="opendbc source root (default: <tree>/opendbc_repo)")
  a = ap.parse_args()

  tree = Path(a.tree).resolve()
  scan_root = tree if a.all else (tree / "openpilot" if (tree / "openpilot").is_dir() else tree)
  odb = Path(a.opendbc).resolve() if a.opendbc else tree / "opendbc_repo"
  if not (odb / "opendbc").is_dir():
    print(f"ERROR: no opendbc package under {odb}", file=sys.stderr)
    return 2

  cache: dict[str, set[str] | None] = {}
  missing: list[tuple[str, int, str, str]] = []
  scanned = 0
  star_checked = 0

  for py in iter_py(scan_root):
    try:
      src = py.read_text()
    except (UnicodeDecodeError, OSError):
      continue
    if "opendbc" not in src:
      continue
    scanned += 1
    try:
      mod = ast.parse(src, filename=str(py))
    except SyntaxError:
      continue
    # the file's own module path, for relative imports (only needed for opendbc-internal files)
    for node in ast.walk(mod):
      if not isinstance(node, ast.ImportFrom) or not node.module:
        continue
      if not (node.module == "opendbc" or node.module.startswith("opendbc.")):
        continue
      have = cache.get(node.module)
      if node.module not in cache:
        have = top_level_names(odb, node.module)
        cache[node.module] = have
      if have is None:
        missing.append((str(py.relative_to(scan_root)), node.lineno, node.module, "<module not found>"))
        continue
      for alias in node.names:
        if alias.name == "*":
          star_checked += 1
          continue
        if alias.name not in have and module_file(odb, node.module + "." + alias.name) is None:
          missing.append((str(py.relative_to(scan_root)), node.lineno, node.module, alias.name))

  print(f"scanned {scanned} file(s) importing opendbc; resolved {len(cache)} opendbc module(s); "
        f"{star_checked} star-import(s) not name-checked")
  if missing:
    for path, lineno, module, name in missing:
      print(f"MISSING {path}:{lineno}  from {module} import {name}")
    print(f"FAIL: {len(missing)} missing name(s)")
    return 1
  print("OK: every name imported from opendbc exists in the opendbc tree")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
