# tinygrad fork fixes (offload track)

The METAL `external_ptr` fix lives on **our tinygrad fork**, not upstream:

    https://github.com/adurham/tinygrad
    branch: offload-metal-external-ptr   (commit 16d93f68c)
    base:   fe5d3169b  == the exact commit the superproject pins

Upstream (`tinygrad/tinygrad`) keeps the current signature on purpose — a maintainer closed the
report with "from_blob for METAL expects an MTLBuffer" — so this is fork-local by design, and
**never open an upstream PR for it**.

## Why the fix is not in the superproject's gitlink

The prebuilt tree ships tinygrad flattened (real files, zero gitlinks/.gitmodules), and the sync
workflow hard-fails on any gitlink in that tree. A superproject pointer change would therefore be
either flattened (fine) or rejected (loudly) — but it would also churn CI for a Mac-only fix the
device never executes (`DEV=QCOM`). So the fork carries the commit and the Mac applies it locally.

## Applying it on the Mac

```bash
# 1. fetch the fork's branch and check it out in the submodule
cd tinygrad_repo
git fetch https://github.com/adurham/tinygrad.git offload-metal-external-ptr
git checkout FETCH_HEAD

# 2. or apply the patch form (idempotent; no fork fetch needed)
bash openpilot/offload/tinygrad-fixes/apply.sh
```

Both reach the same tree. `apply.sh` is the offline form (patch file in this dir, anchored on the
pinned commit's source) — re-run it after any `git submodule update`. If the submodule pin moves,
rebase the patch rather than forcing it (`apply.sh` fails closed rather than fuzz-applying).

## What it fixes

`Tensor.from_blob(host_addr, device='METAL')` segfaults: `MetalAllocator._alloc` passes the raw
address to `metal.MTLBuffer(...)`, which builds a Spec whose `.value` is that integer instead of
doing an ObjC message send, so the next `contents()` / `release()` / `mark_resident()` call
message-sends to a bogus pointer. The offload frame path needs exactly this call
(`model_adapters.copy_frames` → `Tensor.from_blob(ptr, device=WARP_DEV)`).

The fix wraps the caller's memory with `newBufferWithBytesNoCopy:length:options:deallocator:`
(zero-copy, `deallocator=None` so ownership stays with the caller). Zero-copy is **required**, not
stylistic: `from_blob` promises the tensor aliases the source and `test_tensor_from_blob` asserts
it, and the frame tensor is cached per buffer address while the source is rewritten every frame —
a copying constructor would silently freeze frame contents.

## Scope

- **Device unaffected**: AGNOS compiles tinygrad with `DEV=QCOM`; the METAL allocator never runs.
- The fix matters only on the tethered Mac (and any other macOS METAL user of host-pointer blobs).

## Reproduce

```bash
cd tinygrad_repo && git status --short    # M tinygrad/runtime/ops_metal.py once applied
PYTHONPATH="$PWD/tinygrad_repo" ../.venv/bin/python -u ../openpilot/offload/tinygrad-fixes/repro_from_blob.py
# unfixed: Fatal Python error: Segmentation fault (ops_metal.py _alloc -> objc.py msg send)
# fixed:   RESULT: PASS   (also asserts the aliasing property)
```

History: METAL host-pointer `from_blob` never worked here. On the Aug/Sept pins it crashed later
(kernel-bind, `setBuffer_offset_atIndex`); on the Oct pin it crashes in allocation. The Oct 7 sync
(`buffer: rework storage` #18060) moved the crash site; it did not introduce the defect.
