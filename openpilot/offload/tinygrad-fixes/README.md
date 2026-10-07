# tinygrad fork fixes (offload track)

Patches applied to the `tinygrad_repo` submodule that are **not** upstream yet.
The offload CI cannot carry submodule changes, so the Mac worktree applies these
locally via `apply.sh` (idempotent; safe to re-run after `git submodule update`).

| Patch | Why | Upstream status |
|---|---|---|
| `0001-metal-external-ptr-zerocopy.patch` | `Tensor.from_blob(host_addr, device='METAL')` segfaults: `MetalAllocator._alloc` does `metal.MTLBuffer(options.external_ptr)`, which constructs a bogus Spec from an int; the later `.contents()` / `.release()` / `mark_resident()` message-sends to garbage. The offload pipeline needs exactly this call for zero-copy frame ingest (`model_adapters.copy_frames` → `Tensor.from_blob(ptr, device=WARP_DEV)`). | **Reported upstream** (tinygrad/tinygrad issue, 2026-10-07). Fix wraps the caller's memory with `newBufferWithBytesNoCopy:length:options:deallocator:` (aliasing preserved — required, since the frame tensor is cached per buffer address and the source is rewritten every frame). |

## Reproduce

```bash
cd tinygrad_repo && git status --short   # expect: M tinygrad/runtime/ops_metal.py after apply.sh
PYTHONPATH="$PWD/tinygrad_repo" ../.venv/bin/python -u ../openpilot/offload/tinygrad-fixes/repro_from_blob.py
# before fix: Fatal Python error: Segmentation fault (ops_metal.py _alloc → objc.py msg send)
# after fix:  RESULT: PASS
```

History: METAL host-pointer `from_blob` has never worked — on the Aug/Sept pins it
crashed later (kernel-bind, `setBuffer_offset_atIndex`), on the Oct pin and on
canonical master it crashes in allocation. The Oct 7 sync (`buffer: rework storage`
#18060 onwards) just moved the crash site; it did not introduce the defect.
