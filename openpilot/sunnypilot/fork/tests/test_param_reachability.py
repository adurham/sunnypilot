"""
Param-reachability guard (regression for the 0043/0044 unreachable-shaping-param defect, 2026-10-10).

``HyundaiFca11AffineGain`` was read by the car layer (via ``CP_SP.fca11AffineGain``) but was not registered in
params_keys.h and nothing wrote the CarParamsSP field: the brake-shaping law could not be enabled on the car. The static
twin (check_param_reachability.py) runs in sync-upstream on the prebuilt tree; this file is the in-tree test, and it adds
the DYNAMIC half Fable asked for: every fork-owned key the code uses must be accepted by the REAL compiled libparams
(``Params().check_key``), not just present in a parse of the header.
"""
import contextlib
import io
import os
import tempfile
import unittest
from pathlib import Path

from openpilot.sunnypilot.fork.tests import check_param_reachability as CPR

REPO = Path(__file__).resolve().parents[4]


def _opendbc_root() -> Path:
  """The opendbc package root actually imported (the patched tree on PYTHONPATH, or the submodule)."""
  import opendbc
  return Path(opendbc.__file__).resolve().parents[1]


def _collect(tree: Path, opendbc_root: Path):
  keys = []
  for scan in CPR.KEY_SCAN:
    root = opendbc_root if scan.startswith("opendbc/") else tree
    base = root / scan
    if not base.is_dir():
      continue
    for p in sorted(base.rglob("*.py")):
      rel = p.relative_to(root).as_posix()
      if any(part in CPR.SKIP_DIRS for part in p.relative_to(root).parts[:-1]) or CPR._is_test_path(rel):
        continue
      k, _, _, _ = CPR.scan_file(rel, str(p))
      keys += k
  return keys


def _run(tree, opendbc=None):
  buf = io.StringIO()
  with contextlib.redirect_stdout(buf):
    rc = CPR.run(str(tree), None if opendbc is None else str(opendbc))
  return rc, buf.getvalue()


_REG = '#pragma once\ninline static std::unordered_map<std::string, ParamKeyAttributes> keys = {\n{reg}};\n'
_STRUCTS = "class CarParamsSP:\n  a: bool = x\n  b: bool = x\n\n@auto\nclass CarStateSP:\n  z: int = x\n"


def _mini_tree(root: Path, reg_keys, iface_body: str, reader_body: str) -> Path:
  (root / "openpilot" / "common").mkdir(parents=True)
  (root / "opendbc" / "car").mkdir(parents=True)
  (root / "opendbc" / "sunnypilot" / "car" / "hyundai").mkdir(parents=True)
  reg = "".join(f'    {{"{k}", {{PERSISTENT, BOOL}}}},\n' for k in reg_keys)
  (root / "openpilot" / "common" / "params_keys.h").write_text(_REG.replace("{reg}", reg), encoding="utf-8")
  (root / "opendbc" / "car" / "structs.py").write_text(_STRUCTS, encoding="utf-8")
  (root / "opendbc" / "sunnypilot" / "car" / "interfaces.py").write_text(iface_body, encoding="utf-8")
  (root / "opendbc" / "sunnypilot" / "car" / "hyundai" / "feature.py").write_text(reader_body, encoding="utf-8")
  return root


_IFACE_OK = 'def setup(CP_SP, params_dict):\n  if params_dict.get("HyundaiFca11X"):\n    CP_SP.a = True\n'
_READER_OK = 'def on(CP_SP):\n  return bool(CP_SP.a)\n'


class TestParamReachability(unittest.TestCase):

  def test_repo_tree_passes(self):
    rc, out = _run(REPO, _opendbc_root())
    self.assertEqual(0, rc, out)
    self.assertIn("HyundaiFca11AffineGain", out)       # the key is now seen AND registered
    self.assertIn("HyundaiFca11Shadow", out)           # 0045: seen AND registered
    self.assertIn("fca11Shadow", out)                  # 0045: read AND written (writer-lint green)

  def test_fork_keys_accepted_by_the_compiled_libparams(self):
    """DYNAMIC half: the real compiled registry (the libparams_c the Params wrapper loads) accepts every fork key the
    production code uses. Before 0044 HyundaiFca11AffineGain raised UnknownKeyName here."""
    os.environ.setdefault("PARAMS_ROOT", tempfile.mkdtemp(prefix="param-reachability-"))
    from openpilot.common.params import Params
    params = Params()
    keys = sorted({k for k, *_ in _collect(REPO, _opendbc_root()) if k.startswith(CPR.FORK_KEY_PREFIXES)})
    self.assertIn("HyundaiFca11AffineGain", keys)
    self.assertIn("HyundaiFca11Brake", keys)
    self.assertIn("HyundaiFca11Shadow", keys)          # 0045
    self.assertIn("HyundaiFca11Shadow", CPR.FORK_KEY_PREFIXES)
    for k in keys:
      with self.subTest(key=k):
        params.check_key(k)        # raises UnknownKeyName on a key the compiled registry does not know

  def test_key_lint_catches_an_unregistered_fork_key(self):
    with tempfile.TemporaryDirectory() as d:
      t = _mini_tree(Path(d), ["Other"], _IFACE_OK, _READER_OK)
      rc, out = _run(t, t)
      self.assertEqual(1, rc, out)
      self.assertIn("FAIL key-lint: fork key 'HyundaiFca11X'", out)

  def test_writer_lint_catches_a_read_field_without_a_writer(self):
    """Exactly the 0043 defect: a CarParamsSP field read by the controller, never written by setup_interfaces."""
    with tempfile.TemporaryDirectory() as d:
      reader = 'def on(CP_SP):\n  return bool(CP_SP.b)\n'
      t = _mini_tree(Path(d), ["HyundaiFca11X"], _IFACE_OK, reader)
      rc, out = _run(t, t)
      self.assertEqual(1, rc, out)
      self.assertIn("FAIL writer-lint: CarParamsSP.b is READ but has NO WRITER", out)
    with tempfile.TemporaryDirectory() as d:
      reader = 'def on(cp_sp):\n  return bool(getattr(cp_sp, "b", False))\n'      # a getattr read counts too
      t = _mini_tree(Path(d), ["HyundaiFca11X"], _IFACE_OK, reader)
      rc, out = _run(t, t)
      self.assertEqual(1, rc, out)

  def test_env_ban_catches_an_fca11_feature_flag(self):
    with tempfile.TemporaryDirectory() as d:
      reader = 'import os\ndef on(CP_SP):\n  return os.environ.get("FCA11_NEWFLAG", "") == "1" or bool(CP_SP.a)\n'
      t = _mini_tree(Path(d), ["HyundaiFca11X"], _IFACE_OK, reader)
      rc, out = _run(t, t)
      self.assertEqual(1, rc, out)
      self.assertIn("FAIL env-ban", out)
      self.assertIn("FCA11_NEWFLAG", out)

  def test_env_allowlist_is_operational_dirs_only(self):
    with tempfile.TemporaryDirectory() as d:
      reader = 'import os\nD = os.environ.get("FCA11_LOG_DIR", "/data")\ndef on(CP_SP):\n  return bool(CP_SP.a)\n'
      t = _mini_tree(Path(d), ["HyundaiFca11X"], _IFACE_OK, reader)
      rc, out = _run(t, t)
      self.assertEqual(0, rc, out)
    self.assertEqual({"FCA11_LOG_DIR", "FCA11_CAL_DIR", "FCA11_VALIDITY_CSV"}, set(CPR.ENV_ALLOWLIST))

  def test_clean_mini_tree_passes(self):
    with tempfile.TemporaryDirectory() as d:
      t = _mini_tree(Path(d), ["HyundaiFca11X"], _IFACE_OK, _READER_OK)
      rc, out = _run(t, t)
      self.assertEqual(0, rc, out)

  def test_writer_allowlist_is_minimal(self):
    self.assertEqual({}, CPR.WRITER_ALLOWLIST)

  def test_cli_exit_codes(self):
    with tempfile.TemporaryDirectory() as d:
      self.assertEqual(2, CPR.main(["--tree", d]))


if __name__ == "__main__":
  unittest.main()
