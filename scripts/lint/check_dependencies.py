#!/usr/bin/env python3
import sys
import tomllib
import importlib.metadata
from pathlib import Path


def main() -> int:
  if sys.prefix == sys.base_prefix:
    print("Dependency checks require a virtual environment. Run tools/op.sh setup first.")
    return 1

  project = tomllib.loads((Path(__file__).resolve().parents[2] / "pyproject.toml").read_text())["project"]
  direct = len(project["dependencies"]) + sum(len(deps) for deps in project["optional-dependencies"].values())
  # Count each installed package once, including transitive dependencies and all extras.
  packages = {dist.metadata["Name"].lower().replace("_", "-") for dist in importlib.metadata.distributions()}
  # Logical file sizes avoid filesystem block-size differences. Don't follow links to the interpreter or source tree.
  size = sum(path.stat().st_size for path in Path(sys.prefix).rglob("*") if not path.is_symlink() and path.is_file())

  """
    This test prevents our depency footprint from growing.
    These values are *not* intended to be increased, and we
    expect to strictly drive these down over time.

    Fork deltas (each one is a package the fork's own tree must have, not a new upstream dependency):
      * direct 39 -> 40: jetlink, the jetlink_repo submodule, is a python package like msgq/opendbc/pandacan --
        openpilot's sunnypilot/jetlink_adapter and its tests `import jetlink`.
      * total 84 -> 92: that package plus pytest (and its pluggy/iniconfig/packaging) -- the fork's test modules
        (openpilot/offload/**, openpilot/sunnypilot/fork/tests/**) import pytest at module scope, so
        `tools/op.sh test` cannot collect them without it. Upstream's suite is unittest-only, hence neither.
  """
  failed = False
  for name, value, limit in (
    ("Direct dependencies (all extras)", direct, 40),
    ("Total dependencies", len(packages), 92),
    ("Venv size (MiB)", size / 1024**2, 600),
  ):
    print(f"{name}: {value:g} (limit: {limit})")
    failed |= value > limit
  return int(failed)


if __name__ == "__main__":
  raise SystemExit(main())
