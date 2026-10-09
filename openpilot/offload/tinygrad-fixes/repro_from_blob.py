"""Minimal repro: Tensor.from_blob with a HOST address on METAL segfaults.

Run from the repo root (sunnypilot-offload):
  PYTHONPATH=$PWD/tinygrad_repo .venv/bin/python -u openpilot/offload/tinygrad-fixes/repro_from_blob.py

Expected on a fixed tinygrad:    RESULT: PASS
Expected on upstream master:     Fatal Python error: Segmentation fault
   (ops_metal.py _alloc -> objc.py msg send; canonical master crashes at mark_resident)
"""
import faulthandler

faulthandler.enable()

import numpy as np
import tinygrad

print("tinygrad:", tinygrad.__file__)

from tinygrad import Tensor

a = np.arange(1 << 20, dtype=np.uint8) % 251
print(f"ptr={hex(a.ctypes.data)} 16k-aligned={a.ctypes.data % 16384 == 0}")

t = Tensor.from_blob(a.ctypes.data, a.shape, dtype="uint8", device="METAL")
print("from_blob ok:", t.shape)

s = int(t.sum().item())
exp = int(a.sum())
print("sum:", s, "expect:", exp, "->", "CORRECT" if s == exp else "WRONG")

# aliasing contract: writes to the source must be visible through the tensor
a[:] = 7
s2 = int(t.sum().item())
assert s2 == 7 * a.size, f"aliasing broken: sum={s2}"
print("aliasing: OK")

print("RESULT: PASS" if s == exp else "RESULT: MISMATCH")
