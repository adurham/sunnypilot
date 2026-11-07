#!/usr/bin/env python3
"""
check_panda_rx_order.py -- prove the panda RX path runs the safety layer BEFORE the forward decision.

Why this exists
---------------
In panda `board/drivers/fdcan.h::can_rx()` the upstream order is

    int bus_fwd_num = safety_fwd_hook(bus_number, to_push.addr);   // decides forwarding
    ...
    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;       // safety layer sees the frame

so the forward decision for a frame is made on state from the PREVIOUS frame. The fork's Hyundai FCA11 camera
hand-back forwards the camera's bus-2 0x38D only while `hyundai_fca11_long_cam_brake` is set, and that flag is set
by rx_hook from the frame itself, so the FIRST braking-class camera frame was never forwarded. The fix is the panda
patch in `openpilot/sunnypilot/fork/patches/panda/` (FORK.md row 83). This guard fails the moment the order in the
shipped source goes back.

What it checks (structurally, on the C source, not by grepping lines)
----------------------------------------------------------------------
Comments, string and char literals are blanked first, so a call that is only mentioned in a comment does not count.
Every C function body under `<panda>/board/` that CALLS `safety_fwd_hook(` is a forwarding path. For each one:
  * it calls `safety_rx_hook(` exactly ONCE (one safety-layer pass per frame, so `safety_rx_invalid` counts once);
  * that call comes BEFORE the first `safety_fwd_hook(` call in the function;
  * both calls sit directly in the SAME block (same innermost `{`), so the rx call cannot be skipped by a branch
    that the fwd call is not also inside;
  * neither call is inside a preprocessor conditional (`#if`/`#ifdef`) that the other is not.
At least one forwarding path must be found (an empty scan would otherwise "pass" after a refactor or a wrong path).

Usage
-----
    python check_panda_rx_order.py                 # checks <repo>/panda
    python check_panda_rx_order.py --panda DIR     # any panda checkout (e.g. a pristine pin: expected to FAIL)
    python check_panda_rx_order.py --file F.h ...  # explicit files instead of scanning board/

Exit 0 = every forwarding path is rx-before-fwd, 1 = a violation (or no forwarding path found), 2 = usage/IO error.
Stdlib only. Also importable: check_source(text, name) -> (paths_found, errors).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

RX_CALL = "safety_rx_hook"
FWD_CALL = "safety_fwd_hook"
# a function definition head: `<type words / pointers> name(<params>)` followed by `{` (after comment blanking)
FUNC_HEAD_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*\(([^;{}()]*)\)\s*\{")
CONTROL_WORDS = {"if", "for", "while", "switch", "return", "sizeof", "do", "else"}


def blank_comments_and_literals(src: str) -> str:
  """Replace comments and string/char literals with spaces, preserving every newline (line numbers stay exact)."""
  out = []
  i, n = 0, len(src)
  while i < n:
    c = src[i]
    nxt = src[i + 1] if i + 1 < n else ""
    if c == "/" and nxt == "/":
      j = src.find("\n", i)
      j = n if j < 0 else j
      out.append(" " * (j - i))
      i = j
    elif c == "/" and nxt == "*":
      j = src.find("*/", i + 2)
      j = n if j < 0 else j + 2
      out.append("".join(ch if ch == "\n" else " " for ch in src[i:j]))
      i = j
    elif c in "\"'":
      j = i + 1
      while j < n and src[j] != c:
        j += 2 if src[j] == "\\" else 1
      j = min(j + 1, n)
      out.append("".join(ch if ch == "\n" else " " for ch in src[i:j]))
      i = j
    else:
      out.append(c)
      i += 1
  return "".join(out)


def _preproc_stack_at(clean: str, pos: int) -> tuple[int, ...]:
  """Identity of the preprocessor-conditional nesting at `pos`: a tuple of the line numbers of the open #if*."""
  stack: list[int] = []
  for lineno, line in enumerate(clean[:pos].split("\n"), start=1):
    s = line.strip()
    if re.match(r"#\s*if", s):
      stack.append(lineno)
    elif re.match(r"#\s*endif", s):
      if stack:
        stack.pop()
  return tuple(stack)


def _functions(clean: str):
  """Yield (name, body_start, body_end) for each top-level function definition (brace-matched)."""
  depth = 0
  i = 0
  n = len(clean)
  while i < n:
    if depth == 0:
      m = FUNC_HEAD_RE.match(clean, i) if (clean[i].isalpha() or clean[i] == "_") and (i == 0 or not (clean[i - 1].isalnum() or clean[i - 1] == "_")) else None
      if m and m.group(1) not in CONTROL_WORDS:
        open_brace = m.end() - 1
        d = 0
        j = open_brace
        while j < n:
          if clean[j] == "{":
            d += 1
          elif clean[j] == "}":
            d -= 1
            if d == 0:
              break
          j += 1
        yield m.group(1), open_brace, j
        i = j + 1
        continue
    if clean[i] == "{":
      depth += 1
    elif clean[i] == "}":
      depth = max(0, depth - 1)
    i += 1


def _innermost_block(clean: str, start: int, pos: int) -> int:
  """Offset of the innermost `{` enclosing `pos` (searching from the function's opening brace `start`)."""
  stack: list[int] = []
  for k in range(start, pos):
    if clean[k] == "{":
      stack.append(k)
    elif clean[k] == "}":
      stack.pop()
  return stack[-1] if stack else start


def _calls(clean: str, name: str, lo: int, hi: int) -> list[int]:
  return [m.start() for m in re.finditer(rf"\b{re.escape(name)}\s*\(", clean[lo:hi])]


def check_source(src: str, label: str) -> tuple[list[str], list[str]]:
  """Return (forwarding paths found, errors) for one C source text."""
  clean = blank_comments_and_literals(src)
  line_of = lambda p: clean.count("\n", 0, p) + 1  # noqa: E731
  paths, errors = [], []
  for fname, lo, hi in _functions(clean):
    fwd = [lo + p for p in _calls(clean, FWD_CALL, lo, hi)]
    if not fwd:
      continue
    rx = [lo + p for p in _calls(clean, RX_CALL, lo, hi)]
    where = f"{label}:{fname}()"
    paths.append(f"{where} fwd@L{line_of(fwd[0])} rx@{','.join(f'L{line_of(p)}' for p in rx) or '-'}")
    if len(rx) != 1:
      errors.append(f"{where}: expected exactly one {RX_CALL}() call, found {len(rx)}")
      continue
    r, f = rx[0], fwd[0]
    if not r < f:
      errors.append(f"{where}: {FWD_CALL}() at line {line_of(f)} runs BEFORE {RX_CALL}() at line {line_of(r)} " +
                    "(the forward decision would use state from the previous frame)")
    if _innermost_block(clean, lo, r) != _innermost_block(clean, lo, f):
      errors.append(f"{where}: {RX_CALL}() (line {line_of(r)}) and {FWD_CALL}() (line {line_of(f)}) are not in the " +
                    "same block -- a branch could skip the safety layer for a forwarded frame")
    if _preproc_stack_at(clean, r) != _preproc_stack_at(clean, f):
      errors.append(f"{where}: {RX_CALL}() and {FWD_CALL}() sit under different #if conditions")
  return paths, errors


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
  default_panda = Path(__file__).resolve().parents[4] / "panda"
  ap.add_argument("--panda", type=Path, default=default_panda, help=f"panda checkout (default {default_panda})")
  ap.add_argument("--file", type=Path, action="append", default=[], help="check these files instead of <panda>/board")
  args = ap.parse_args(argv)

  files = args.file or sorted((args.panda / "board").rglob("*.[ch]"))
  if not files:
    print(f"error: no C sources found (panda={args.panda})", file=sys.stderr)
    return 2
  all_paths, all_errors = [], []
  for f in files:
    try:
      text = f.read_text(encoding="utf-8", errors="replace")
    except OSError as e:
      print(f"error: {e}", file=sys.stderr)
      return 2
    p, e = check_source(text, str(f))
    all_paths += p
    all_errors += e
  for p in all_paths:
    print(f"forwarding path: {p}")
  if not all_paths:
    all_errors.append(f"no function calling {FWD_CALL}() found under the scanned sources -- the guard is not looking at " +
                      "the RX path any more (moved/renamed?); refusing to pass vacuously")
  for e in all_errors:
    print(f"FAIL: {e}")
  if all_errors:
    return 1
  print(f"OK: {len(all_paths)} forwarding path(s), every one runs {RX_CALL}() before {FWD_CALL}()")
  return 0


if __name__ == "__main__":
  sys.exit(main())
