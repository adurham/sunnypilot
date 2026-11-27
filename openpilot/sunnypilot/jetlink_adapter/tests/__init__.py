"""Tests for the jetlink adapter.

The adapter itself treats a jetlink that cannot be imported as "the link is off" (see
openpilot/sunnypilot/jetlink_adapter/__init__.py::_guard), which is what a device without the jetlink checkout does.
These tests are the other case: they pin the fork's side of the interface against the REAL jetlink, so they need the
pinned jetlink_repo submodule importable before any test module does `import jetlink`.

jetlink is not a pip dependency of openpilot (pyproject.toml declares no jetlink dependency; the submodule is the
checkout the comma runs from), so put it on sys.path here, once, for the whole package.
"""
import sys
from pathlib import Path

# .../openpilot/sunnypilot/jetlink_adapter/tests -> repo root
JETLINK_REPO = Path(__file__).resolve().parents[4] / 'jetlink_repo'
if JETLINK_REPO.is_dir() and str(JETLINK_REPO) not in sys.path:
  sys.path.insert(0, str(JETLINK_REPO))
