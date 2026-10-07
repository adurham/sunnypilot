# tinygrad fork fixes (offload track)

Patches applied to the `tinygrad_repo` submodule that are **fork-local and not upstream**.
The offload CI cannot carry submodule changes, so the Mac worktree applies these locally via
`apply.sh` (idempotent; safe to re-run after `git submodule update`). Re-run it whenever the
submodule pin moves — the patch is anchored on the pinned commit's source.

| Patch | Why | Status |
|---|---|---|
| `0001-metal-external-ptr-zerocopy.patch` | `Tensor.from_blob(host_addr, device='METAL')` segfaults on macOS. `MetalAllocator._alloc` does `metal.MTLBuffer(options.external_ptr)`, which constructs a bogus Spec from an int rather than doing an ObjC message send, so the later `.contents()` / `.release()` / `mark_resident()` calls message-send to garbage. The offload pipeline needs exactly this call for zero-copy frame ingest (`model_adapters.copy_frames` → `Tensor.from_blob(ptr, device=WARP_DEV)`). | **Fork-local, not upstream.** Wraps the caller's memory with `newBufferWithBytesNoCopy:length:options:deallocator:` (aliasing preserved — required, since the frame tensor is cached per buffer address and the source is rewritten every frame). Do NOT swap in `newBufferWithBytes:length:options:`: it copies, which silently freezes frames. |

## Scope and blast radius

- **The device is unaffected**: AGNOS compiles tinygrad with `DEV=QCOM`, so the METAL allocator
  is never exercised there. This matters only on the tethered Mac.
- **Upstream intends the current signature.** A tinygrad maintainer closed the report with
  "from_blob for METAL expects an MTLBuffer" — that path is only supported for buffers the caller
  obtained from the same METAL device. Our use (a host pointer produced by the VisionIPC frame
  buffer) is outside that contract, so the fix lives in this fork and is re-applied locally.
  If upstream ever changes the external-pointer contract, re-check this patch rather than
  assuming it still applies.

## Reproduce

```bash
cd tinygrad_repo && git status --short   # expect: M tinygrad/runtime/ops_metal.py after apply.sh
PYTHONPATH="$PWD/tinygrad_repo" ../.venv/bin/python -u ../openpilot/offload/tinygrad-fixes/repro_from_blob.py
# without the patch: Fatal Python error: Segmentation fault (ops_metal.py _alloc -> objc.py msg send)
# with the patch:    RESULT: PASS
```

The repro also asserts the aliasing property (`src[i] = x` must be visible through the tensor),
which is the reason the zero-copy constructor is required.

History: METAL host-pointer `from_blob` has never worked here — on the Aug/Sept pins it crashed
later (kernel-bind, `setBuffer_offset_atIndex`), on the Oct pin it crashes in allocation. The
Oct 7 sync (`buffer: rework storage` #18060 onwards) moved the crash site; it did not introduce
the defect relative to our usage.
