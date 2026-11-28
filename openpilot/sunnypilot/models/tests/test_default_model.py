"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import os
import subprocess

from openpilot.sunnypilot import get_file_hash
import hashlib
from openpilot.common.test import OpenpilotTestCase


def _default_model():
  """`default_model` imports the openpilot UI state singleton, which builds a SubMaster('deviceState') at import
  time; that needs the per-test msgq directory OpenpilotTestCase.run() creates. Importing it at module scope runs
  before any test (collection time), where no msgq directory exists, and takes the whole module down with it."""
  from openpilot.sunnypilot.models import default_model
  return default_model


def _get_lfs_oid(path: str) -> str:
  """Extract the LFS OID (SHA256 of actual content) from git, works whether the file is smudged or not."""
  pointer = subprocess.check_output(["git", "show", f"HEAD:{path}"], text=True)
  for line in pointer.splitlines():
    if line.startswith("oid sha256:"):
      return line.split(":", 1)[1]
  raise ValueError(f"No LFS OID found for {path}")


class TestDefaultModel(OpenpilotTestCase):
  def test_compare_onnx_hashes(self):
    dm = _default_model()
    fields = dm._read_model_name_fields()
    supercombo_hash = get_file_hash(dm.SUPERCOMBO_ONNX_PATH)
    fingerprint = f"{supercombo_hash}:{fields.get('DEFAULT_MODEL', '')}:{fields.get('DEFAULT_MODEL_REF', '')}"
    combined_hash = hashlib.sha256(fingerprint.encode()).hexdigest()

    with open(dm.MODEL_HASH_PATH) as f:
      current_hash = f.read().strip()

    assert combined_hash == current_hash, "Run openpilot/sunnypilot/models/default_model.py to update the default model name and hash"

  def test_compare_big_pkl_hashes(self):
    dm = _default_model()
    fields = dm._read_model_name_fields()
    oid = _get_lfs_oid(os.path.relpath(dm.BIG_DRIVING_PKL_PATH, os.getcwd()))
    big_fingerprint = f"{oid}:{fields.get('DEFAULT_BIG_MODEL', '')}:{fields.get('DEFAULT_BIG_MODEL_REF', '')}"
    combined_hash = hashlib.sha256(big_fingerprint.encode()).hexdigest()

    with open(dm.BIG_MODEL_HASH_PATH) as f:
      current_hash = f.read().strip()

    assert combined_hash == current_hash, "Run openpilot/sunnypilot/models/default_model.py to update the default model name and hash"
