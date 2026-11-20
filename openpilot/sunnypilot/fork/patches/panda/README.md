# panda patch series (firmware `board/` sources)

Patches here apply to the **panda** submodule, not opendbc_repo. Run them from inside `panda/` at the pinned commit
(`git rev-parse HEAD:panda`, currently `8a16e089`) with plain `git apply` (no `--3way`, no whitespace fuzz):

    cd panda && for p in ../openpilot/sunnypilot/fork/patches/panda/*.patch; do git apply "$p"; done

* CI (`.github/workflows/sync-upstream.yaml`, step "Apply the fork's panda patch series") applies them before the H7
  firmware build. It refuses any path outside `board/`, then runs `fork/tests/check_panda_rx_order.py` on the patched
  tree. The patches are part of the `patches-sha256` fingerprint, so editing one forces a firmware rebuild.
* Nothing from here goes into the prebuilt tree as source. The prebuilt ships only the compiled
  `panda/board/obj/panda_h7.bin.signed`.
* Numbering uses the same `NNNN-` space as the opendbc series in the parent directory, so FORK.md can cite a single
  number. `check_patch_numbers.py` fails if a prefix is reused in either directory.
* If a panda pin bump touches a patched file, the apply step fails and the CI run goes red. That is intended. Re-cut the
  patch against the new pin, or drop it if upstream fixed the issue. `test_panda_rx_order.py` fails once the pinned
  upstream file no longer has the defect.

| patch | FORK.md | what |
|---|---|---|
| `0004-panda-safety-rx-hook-before-fwd.patch` | row 83 | `can_rx()` now runs `safety_rx_hook` before `safety_fwd_hook`, so the first braking-class camera 0x38D is forwarded |
