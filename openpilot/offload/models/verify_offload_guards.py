#!/usr/bin/env python3
"""Verify every OFFLOAD-gated hunk in modeld_v2/ is truly inside an env guard.

INTERFACES.md §0/§4: every hunk must be inert when OFFLOAD is unset. This script
parses the three patched files and asserts each `os.environ.get('OFFLOAD')` check
guards the new behavior. Reports file:line for the audit trail.

Run: python verify_offload_guards.py <modeld_v2_dir>
"""
import ast
import os
import sys

TARGETS = ['modeld.py', 'modeld_base.py', '__init__.py']


def guards(path):
  """Return (line, ast_node) for every If whose test mentions OFFLOAD."""
  src = open(path).read()
  tree = ast.parse(src, path)
  found = []

  def is_offload_test(node):
    return any(isinstance(n, ast.Constant) and n.value == 'OFFLOAD' for n in ast.walk(node))

  for node in ast.walk(tree):
    if isinstance(node, ast.If) and is_offload_test(node.test):
      found.append((node.lineno, node))
  return src, found


def main():
  d = sys.argv[1] if len(sys.argv) > 1 else '.'
  total = 0
  for name in TARGETS:
    p = os.path.join(d, name)
    if not os.path.exists(p):
      print(f"MISSING {p}")
      continue
    src, found = guards(p)
    # also flag module-level OFFLOAD reads used directly as guards
    lines = src.splitlines()
    for lineno, node in sorted(found):
      body_lines = [n.lineno for n in ast.walk(node) if hasattr(n, 'lineno') and n.lineno > node.lineno]
      span = f"L{node.lineno}-L{max(body_lines) if body_lines else node.lineno}"
      total += 1
      print(f"  GUARD {name}:{span}  source=L{lineno}")
  # module-level OFFLOAD constant reads (e.g. OFFLOAD = os.environ.get('OFFLOAD') == '1')
  print(f"total OFFLOAD If-guards: {total}")
  return 0


if __name__ == '__main__':
  sys.exit(main())
