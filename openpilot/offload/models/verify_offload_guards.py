#!/usr/bin/env python3
"""Verify every OFFLOAD-gated hunk in modeld_v2/ is truly inside an env guard.

INTERFACES.md §0/§4: every hunk must be inert when OFFLOAD is unset. This script
parses the three patched files and asserts each `os.environ.get('OFFLOAD')` check
guards the new behavior. Reports file:line for the audit trail.

FAILS CLOSED: a missing target file, an unparseable file, or ZERO guards found all
exit non-zero. A verifier that reports success when it inspected nothing is worse
than no verifier — the first version exited 0 with "MISSING ./modeld.py ... total
OFFLOAD If-guards: 0" when invoked from the wrong directory (2026-10-07).

Run: python verify_offload_guards.py [modeld_v2_dir]
     (default: <repo>/openpilot/sunnypilot/modeld_v2, so a bare run from anywhere works)
     python verify_offload_guards.py --selftest   # prove the failure modes actually fail
"""
import ast
import os
import sys

TARGETS = ['modeld.py', 'modeld_base.py', '__init__.py']
# .../openpilot/offload/models/ -> up two = openpilot/ -> sunnypilot/modeld_v2
DEFAULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', '..', 'sunnypilot', 'modeld_v2')


def guards(path):
  """Return (line, ast_node) for every If whose test mentions OFFLOAD."""
  with open(path) as f:
    src = f.read()
  tree = ast.parse(src, path)
  found = []

  def is_offload_test(node):
    return any(isinstance(n, ast.Constant) and n.value == 'OFFLOAD' for n in ast.walk(node))

  for node in ast.walk(tree):
    if isinstance(node, ast.If) and is_offload_test(node.test):
      found.append((node.lineno, node))
  return src, found


def audit(d) -> tuple[int, list[str]]:
  """Return (total_guards, problems). Non-empty problems => the audit did not pass."""
  total, problems = 0, []
  for name in TARGETS:
    p = os.path.join(d, name)
    if not os.path.exists(p):
      problems.append(f"MISSING {p}")
      continue
    try:
      src, found = guards(p)
    except SyntaxError as e:
      problems.append(f"UNPARSEABLE {p}: {e}")
      continue
    for lineno, node in sorted(found):
      body_lines = [n.lineno for n in ast.walk(node) if hasattr(n, 'lineno') and n.lineno > node.lineno]
      span = f"L{node.lineno}-L{max(body_lines) if body_lines else node.lineno}"
      total += 1
      print(f"  GUARD {name}:{span}  source=L{lineno}")
  if total == 0:
    problems.append(f"no OFFLOAD guards found under {os.path.abspath(d)} — wrong directory, "
                    f"or every guard was removed")
  return total, problems


def main() -> int:
  args = [a for a in sys.argv[1:]]
  if '--selftest' in args:
    return selftest()
  d = args[0] if args else os.path.normpath(DEFAULT_DIR)
  total, problems = audit(d)
  print(f"total OFFLOAD If-guards: {total}")
  for p in problems:
    print(f"FAIL: {p}", file=sys.stderr)
  return 1 if problems else 0


def selftest() -> int:
  """Prove the three failure modes exit non-zero and the real tree exits zero."""
  import tempfile
  rc = 0

  with tempfile.TemporaryDirectory() as td:
    total, problems = audit(td)                       # nothing there at all
    assert total == 0 and problems
    print(f"[selftest] empty dir -> {len(problems)} problem(s), rc=1  OK")

  total, problems = audit(os.path.normpath(DEFAULT_DIR))   # the real tree
  assert total > 0 and not problems, (total, problems)
  print(f"[selftest] real tree -> {total} guards, rc=0  OK")

  with tempfile.TemporaryDirectory() as td:
    for n in TARGETS:                                 # files present but no guards
      with open(os.path.join(td, n), 'w') as f:
        f.write("x = 1\n")
    total, problems = audit(td)
    assert total == 0 and problems
    print(f"[selftest] files-without-guards -> {len(problems)} problem(s), rc=1  OK")

  print("[selftest] PASS")
  return rc


if __name__ == '__main__':
  sys.exit(main())
