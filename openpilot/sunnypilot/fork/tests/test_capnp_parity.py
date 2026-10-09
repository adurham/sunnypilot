"""
Schema-parity guard for the SP capnp mirrors (regression for the 0041
fca11AffineGain card crash-loop, route 154, 2026-10-09).

``card`` serializes CarParamsSP / CarStateSP through helpers.convert_to_capnp at every
ignition; a dataclass field with no matching member in openpilot/cereal/custom.capnp
becomes ``KjException: struct has no such member`` at startup, card crash-loops, no
CarParams is ever published and the screen sits on "sunnypilot Unavailable / Waiting
to start" for the whole drive.  The static twin (check_capnp_parity.py) runs in
sync-upstream before the prebuilt tree is committed; this file is the in-tree test.
"""
import contextlib
import dataclasses
import io
import tempfile
import unittest
from pathlib import Path

from openpilot.cereal import custom
from openpilot.selfdrive.car import helpers
from opendbc.car import structs

from openpilot.sunnypilot.fork.tests import check_capnp_parity as CCP

REPO = Path(__file__).resolve().parents[4]


def _write_tree(root: Path, capnp: str, structs_py: str) -> Path:
  (root / "openpilot" / "cereal").mkdir(parents=True)
  (root / "opendbc" / "car").mkdir(parents=True)
  (root / "openpilot" / "cereal" / "custom.capnp").write_text(capnp, encoding="utf-8")
  (root / "opendbc" / "car" / "structs.py").write_text(structs_py, encoding="utf-8")
  return root


_CAPNP = "\n".join([
  "struct CarParamsSP @0x1 {",
  "  a @0 :Bool;",
  "}",
  "struct CarStateSP @0x2 {",
  "  b @0 :Bool;",
  "}",
  "struct CarControlSP @0x3 {",
  "  c @0 :Bool;",
  "}",
  "",
])
_STRUCTS = "\n".join([
  "class CarParamsSP:",
  "  a: bool = x",
  "",
  "@auto",
  "class CarStateSP:",
  "  b: int = x",
  "",
  "@auto",
  "class CarControlSP:",
  "  c: int = x",
  "",
])


class TestCapnpParity(unittest.TestCase):

  def test_dataclass_fields_all_have_schema_members(self):
    # the live tree (check_capnp_parity.py also gates the deploy on this invariant)
    for name in CCP.SP_STRUCTS:
      with self.subTest(struct=name):
        dfields = {f.name for f in dataclasses.fields(getattr(structs, name))}
        sfields = {f.name for f in getattr(custom, name).schema.node.struct.fields}
        missing = sorted(dfields - sfields)
        self.assertEqual([], missing, f"{name} dataclass fields missing from custom.capnp: {missing}")

  def test_convert_to_capnp_defaults_succeed(self):
    # the exact call card makes at ignition (card.py __init__)
    helpers.convert_to_capnp(structs.CarParamsSP())
    helpers.convert_to_capnp(structs.CarStateSP())

  def test_static_checker_passes_on_a_consistent_tree(self):
    with tempfile.TemporaryDirectory() as t:
      tree = _write_tree(Path(t), _CAPNP, _STRUCTS)
      with contextlib.redirect_stdout(io.StringIO()):
        self.assertEqual(0, CCP.run(str(tree)))

  def test_static_checker_catches_a_field_missing_from_the_schema(self):
    # this is the exact shape of the 0041 defect (dataclass grew, schema did not)
    with tempfile.TemporaryDirectory() as t:
      tree = _write_tree(Path(t), _CAPNP, _STRUCTS.replace("  a: bool = x", "  a: bool = x\n  a2: bool = x"))
      buf = io.StringIO()
      with contextlib.redirect_stdout(buf):
        rc = CCP.run(str(tree))
      self.assertEqual(1, rc, buf.getvalue())
      self.assertIn("a2", buf.getvalue())

  def test_static_checker_clean_on_the_real_tree(self):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
      rc = CCP.run(str(REPO))
    self.assertEqual(0, rc, buf.getvalue())
