#!/usr/bin/env python3
"""
check_param_reachability.py -- static guard: every fork feature flag the car code READS must be REACHABLE on the car.

Why this exists
---------------
Patch 0041/0043 gated the FCA11 brake-shaping law on ``CP_SP.fca11AffineGain`` (param ``HyundaiFca11AffineGain``), but
(1) the key was never registered in ``openpilot/common/params_keys.h`` -- ``Params().put_bool`` raised UnknownKeyName on
the compiled registry -- and (2) nothing ever ASSIGNED ``CP_SP.fca11AffineGain``; the car code only READ it.  The sim
turned the law on through an env override (``FCA11_AFFINE``), which hid both gaps, and four reviews of the car-layer
Python never looked at the superproject wiring.  Same defect class as the 0041 capnp crash-loop (check_capnp_parity.py)
and the 0018 HyundaiFca11Brake plumbing fix.  Fixed by patch 0044; this check makes the class mechanical (Fable Ruling 2
(4)).  It runs in sync-upstream next to the capnp parity guard, on the prebuilt tree, before anything is committed.

What it checks (production code only -- ``tests/`` dirs and ``test_*.py`` are skipped)
--------------------------------------------------------------------------------------
(a) KEY LINT.  Every param key read or written with a string literal (or a same-function / module-level string
    constant) through ``params_dict[...]`` / ``params_dict.get(...)``, ``<params>.get*/put*/remove/check_key(...)`` on a
    params-like receiver (source text contains "params", or a name bound to ``Params(...)`` in the same function), or a
    UI ``param="..."`` keyword, under opendbc/sunnypilot, openpilot/sunnypilot and openpilot/selfdrive/ui/sunnypilot.
    FORK-OWNED keys (prefixes in FORK_KEY_PREFIXES) MUST be registered in params_keys.h.  Other keys: report-only.
    (The pytest twin, test_param_reachability.py, additionally asserts each fork key is accepted by the REAL compiled
    libparams via check_key.)
(b) WRITER LINT.  Every ``CarParamsSP`` dataclass field (opendbc/car/structs.py) that production code READS
    (``<CP_SP-like>.<field>`` in a load context, or ``getattr(<CP_SP-like>, "<field>")``) must have >= 1 WRITER: an
    assignment / augmented assignment to ``<CP_SP-like or ret>.<field>`` or a ``CarParamsSP(<field>=...)`` kwarg in a
    writer file (opendbc/sunnypilot/car/**/interfaces.py, opendbc/car/**/interface.py, openpilot/sunnypilot/selfdrive/
    car/**).  Intentionally default-only fields go in WRITER_ALLOWLIST with a reason.
(c) ENV-OVERRIDE BAN.  No ``os.environ`` / ``os.getenv`` read of an ``FCA11_*`` name in production code, except the
    operational directories in ENV_ALLOWLIST and the named refusal guard in ENV_GUARD_ALLOWLIST.

Usage
-----
    python check_param_reachability.py --tree <repo-or-prebuilt-tree-root> [--opendbc <dir containing opendbc/>]
--opendbc defaults to <tree>/opendbc_repo, then <tree> (a flattened prebuilt tree keeps opendbc under opendbc_repo/).
Exit 0 = OK, 1 = violation(s) (listed), 2 = usage/IO error.  Stdlib only -- runs in CI, on the Mac, and on the device.
"""
from __future__ import annotations

import argparse
import ast
import os
import re
import sys

# Keys whose names start with one of these are FORK-OWNED: they must be in params_keys.h. Extensible.
FORK_KEY_PREFIXES = ("HyundaiFca11", "HyundaiGasInterceptor", "DriveModePersonality",
                     "HyundaiFca11Shadow")  # 0045: named explicitly (also covered by HyundaiFca11) so a prefix edit can't drop it

# CarParamsSP fields that are read but intentionally never written by the fork's wiring. Keep minimal; reason each.
WRITER_ALLOWLIST: dict[str, str] = {}

# FCA11_* env names that are OPERATIONAL DIRECTORIES (where to write logs / read the cal plan), not feature flags.
ENV_ALLOWLIST = {
  "FCA11_LOG_DIR": "operational dir: the per-episode logger output dir ('0' disables I/O in tests); not a behaviour flag",
  "FCA11_CAL_DIR": ("operational dir: the cal-plan dir ('0' disables the sequencer in tests); the cal mode itself is " +
                    "file-gated under the HyundaiFca11Brake kill switch; not a behaviour flag"),
  "FCA11_VALIDITY_CSV": ("operational input path of the OFFLINE cal-runner CLI (fork/fca11_cal_runner.py --status): where " +
                         "the rep-validity analysis CSV lives; not in the actuation path, not a behaviour flag"),
}
# (file suffix, function) -> {env name: reason}: a read whose ONLY purpose is to REFUSE a removed override.
ENV_GUARD_ALLOWLIST = {
  ("opendbc/sunnypilot/car/hyundai/fca11_long.py", "_refuse_affine_env"): {
    "FCA11_AFFINE": "refusal guard (0044): raises RuntimeError when the removed override is set; never enables anything",
  },
  ("opendbc/sunnypilot/car/hyundai/fca11_long.py", "_refuse_shadow_env"): {
    "FCA11_SHADOW": "refusal guard (0045): raises RuntimeError when FCA11_SHADOW is set; Shadow is param-only",
  },
}

PARAM_METHODS = {"get", "get_bool", "put", "put_bool", "put_nonblocking", "put_bool_nonblocking", "remove", "check_key",
                 "get_default_value", "get_type"}
CP_SP_RE = re.compile(r"(?i)\bcp_?sp\b|carparamssp")
SKIP_DIRS = {"tests", "test", "__pycache__", ".git", "node_modules"}

KEY_SCAN = ("opendbc/sunnypilot", "openpilot/sunnypilot", "openpilot/selfdrive/ui/sunnypilot")
READ_SCAN = ("opendbc/sunnypilot", "opendbc/car", "openpilot/sunnypilot", "openpilot/selfdrive")
WRITER_GLOBS = (re.compile(r"^opendbc/sunnypilot/car/(.+/)?interfaces\.py$"),
                re.compile(r"^opendbc/car/(.+/)?interface\.py$"),
                re.compile(r"^openpilot/sunnypilot/selfdrive/car/.+\.py$"))


def _is_cp_sp(node) -> bool:
  """a CarParamsSP-like expression: a name/attribute whose LAST component is CP_SP / cp_sp / *CarParamsSP*."""
  last = _src(node).split(".")[-1]
  return bool(CP_SP_RE.fullmatch(last) or re.fullmatch(r"(?i)\w*carparamssp\w*", last))


def registry_keys(text: str) -> set[str]:
  return set(re.findall(r'^\s*\{"([A-Za-z0-9_]+)",\s*\{', text, re.M))


def dataclass_fields(text: str, cls: str):
  m = re.search(rf"(?ms)^class {re.escape(cls)}:\n(.*?)(?=^@|^class |\Z)", text)
  if not m:
    return None
  return set(re.findall(r"(?m)^  ([A-Za-z_][A-Za-z0-9_]*):", m.group(1)))


def _src(node) -> str:
  try:
    return ast.unparse(node)
  except Exception:
    return ""


def _is_test_path(rel: str) -> bool:
  parts = rel.split("/")
  return any(p in SKIP_DIRS for p in parts[:-1]) or parts[-1].startswith("test_") or parts[-1] == "conftest.py"


def _str_assigns(body) -> dict[str, str]:
  """name -> string constant, for simple `NAME = "literal"` assignments inside a function body."""
  out = {}
  for n in ast.walk(ast.Module(body=body, type_ignores=[])):
    if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant) and isinstance(n.value.value, str):
      for t in n.targets:
        if isinstance(t, ast.Name):
          out[t.id] = n.value.value
  return out


def _params_names(body) -> set[str]:
  """names bound to `Params(...)` inside a function body (so `p = Params(); p.get_bool("K")` is seen)."""
  out = set()
  for n in ast.walk(ast.Module(body=body, type_ignores=[])):
    if isinstance(n, ast.Assign) and isinstance(n.value, ast.Call) and _src(n.value.func).split(".")[-1] == "Params":
      for t in n.targets:
        if isinstance(t, ast.Name):
          out.add(t.id)
  return out


def _func_of(tree):
  """map id(node) -> innermost enclosing FunctionDef (or None)."""
  owner = {}

  def walk(node, fn):
    for ch in ast.iter_child_nodes(node):
      f = ch if isinstance(ch, (ast.FunctionDef, ast.AsyncFunctionDef)) else fn
      owner[id(ch)] = f
      walk(ch, f)
  walk(tree, None)
  return owner


def _module_consts(tree) -> dict[str, str]:
  out = {}
  for n in tree.body:
    if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant) and isinstance(n.value.value, str):
      for t in n.targets:
        if isinstance(t, ast.Name):
          out[t.id] = n.value.value
  return out


def _resolve(node, local: dict[str, str], module: dict[str, str]):
  if isinstance(node, ast.Constant) and isinstance(node.value, str):
    return node.value
  if isinstance(node, ast.Name):
    return local.get(node.id, module.get(node.id))
  return None


def scan_file(rel: str, path: str):
  """-> (key_uses[(key, rel, line, how)], cp_reads[(field, rel, line)], cp_writes[(field, rel, line)],
          env_reads[(name, rel, line, func)])"""
  with open(path, encoding="utf-8") as fh:
    text = fh.read()
  try:
    tree = ast.parse(text, filename=path)
  except SyntaxError as e:
    raise SystemExit(f"ERROR: cannot parse {rel}: {e}") from e
  owner = _func_of(tree)
  module = _module_consts(tree)
  fn_cache: dict[int, tuple[dict[str, str], set[str]]] = {}

  def scope(node):
    fn = owner.get(id(node))
    if fn is None:
      return {}, set()
    if id(fn) not in fn_cache:
      fn_cache[id(fn)] = (_str_assigns(fn.body), _params_names(fn.body))
    return fn_cache[id(fn)]

  keys, reads, writes, envs = [], [], [], []
  for n in ast.walk(tree):
    # ---- (a) key uses ----
    if isinstance(n, ast.Subscript) and isinstance(n.value, ast.Name) and n.value.id == "params_dict":
      local, _ = scope(n)
      k = _resolve(n.slice, local, module)
      if k:
        keys.append((k, rel, n.lineno, "params_dict[...]"))
    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
      local, pnames = scope(n)
      recv = n.func.value
      recv_src = _src(recv)
      if recv_src == "params_dict" and n.func.attr == "get" and n.args:
        k = _resolve(n.args[0], local, module)
        if k:
          keys.append((k, rel, n.lineno, "params_dict.get"))
      elif n.func.attr in PARAM_METHODS and n.args and (
          ("params" in recv_src.lower() and recv_src != "params_dict") or
          (isinstance(recv, ast.Name) and recv.id in pnames) or
          (isinstance(recv, ast.Call) and _src(recv.func).split(".")[-1] == "Params")):
        k = _resolve(n.args[0], local, module)
        if k:
          keys.append((k, rel, n.lineno, f"{recv_src}.{n.func.attr}"))
    if isinstance(n, ast.Call):
      for kw in n.keywords:
        if kw.arg == "param":
          local, _ = scope(n)
          k = _resolve(kw.value, local, module)
          if k:
            keys.append((k, rel, n.lineno, "param="))
    # ---- (b) CarParamsSP reads / writes ----
    if isinstance(n, ast.Attribute) and _is_cp_sp(n.value) and isinstance(n.ctx, ast.Load):
      reads.append((n.attr, rel, n.lineno))
    if isinstance(n, ast.Call) and _src(n.func) == "getattr" and len(n.args) >= 2 and CP_SP_RE.search(_src(n.args[0])):
      local, _ = scope(n)
      f = _resolve(n.args[1], local, module)
      if f:
        reads.append((f, rel, n.lineno))
    targets = []
    if isinstance(n, ast.Assign):
      targets = n.targets
    elif isinstance(n, (ast.AugAssign, ast.AnnAssign)):
      targets = [n.target]
    for t in targets:
      # the field is the attribute directly on the CP_SP-like base; nested sub-struct writes
      # (CP_SP.neuralNetworkLateralControl.model.path = ...) count as writes of the top-level field.
      while isinstance(t, ast.Attribute) and isinstance(t.value, ast.Attribute) and not _is_cp_sp(t.value):
        t = t.value
      if isinstance(t, ast.Attribute) and (_is_cp_sp(t.value) or _src(t.value) == "ret"):
        writes.append((t.attr, rel, n.lineno))
    if isinstance(n, ast.Call) and _src(n.func).split(".")[-1] == "CarParamsSP":
      for kw in n.keywords:
        if kw.arg:
          writes.append((kw.arg, rel, n.lineno))
    # ---- (c) env reads ----
    name = None
    if isinstance(n, ast.Call):
      fs = _src(n.func)
      if fs in ("os.environ.get", "os.getenv", "environ.get", "getenv", "os.environ.pop", "os.environ.setdefault") and n.args:
        local, _ = scope(n)
        name = _resolve(n.args[0], local, module)
    elif isinstance(n, ast.Subscript) and _src(n.value) in ("os.environ", "environ"):
      local, _ = scope(n)
      name = _resolve(n.slice, local, module)
    elif isinstance(n, ast.Compare) and len(n.comparators) == 1 and _src(n.comparators[0]) in ("os.environ", "environ") \
        and isinstance(n.ops[0], (ast.In, ast.NotIn)):
      local, _ = scope(n)
      name = _resolve(n.left, local, module)
    if name and name.startswith("FCA11_"):
      fn = owner.get(id(n))
      envs.append((name, rel, n.lineno, fn.name if fn is not None else "<module>"))
  return keys, reads, writes, envs


def run(tree: str, opendbc: str | None = None, verbose: bool = False) -> int:
  tree = os.path.abspath(tree)
  reg_path = os.path.join(tree, "openpilot", "common", "params_keys.h")
  if not os.path.isfile(reg_path):
    print(f"ERROR: missing {reg_path}")
    return 2
  if opendbc is None:
    opendbc = next((c for c in (os.path.join(tree, "opendbc_repo"), tree) if os.path.isfile(os.path.join(c, "opendbc", "car", "structs.py"))), None)
  if opendbc is None or not os.path.isfile(os.path.join(opendbc, "opendbc", "car", "structs.py")):
    print(f"ERROR: no opendbc/car/structs.py under {opendbc or tree} (pass --opendbc)")
    return 2
  opendbc = os.path.abspath(opendbc)
  with open(reg_path, encoding="utf-8") as fh:
    registry = registry_keys(fh.read())
  with open(os.path.join(opendbc, "opendbc", "car", "structs.py"), encoding="utf-8") as fh:
    sp_fields = dataclass_fields(fh.read(), "CarParamsSP")
  if not registry or not sp_fields:
    print(f"ERROR: could not parse the registry ({len(registry)} keys) or CarParamsSP ({sp_fields})")
    return 2

  files = []
  for scan in (KEY_SCAN, READ_SCAN):
    for s in scan:
      root = opendbc if s.startswith("opendbc/") else tree
      base = os.path.join(root, s)
      if not os.path.isdir(base):
        continue
      for d, dirs, fs in os.walk(base):
        dirs[:] = sorted(x for x in dirs if x not in SKIP_DIRS)
        for f in sorted(fs):
          if f.endswith(".py"):
            ap = os.path.join(d, f)
            rel = os.path.relpath(ap, root).replace(os.sep, "/")
            if not _is_test_path(rel):
              files.append((rel, ap))
  files = sorted(set(files))

  keys, reads, writes, envs = [], [], [], []
  for rel, ap in files:
    k, r, w, e = scan_file(rel, ap)
    if any(rel.startswith(s + "/") for s in KEY_SCAN):
      keys += k
      envs += e
    if any(rel.startswith(s + "/") for s in READ_SCAN):
      reads += r
    if any(g.match(rel) for g in WRITER_GLOBS):
      writes += w
  rc = 0

  # (a) key lint
  fork_keys = sorted({k for k, *_ in keys if k.startswith(FORK_KEY_PREFIXES)})
  missing = sorted({k for k in fork_keys if k not in registry})
  other_unknown = sorted({k for k, *_ in keys if not k.startswith(FORK_KEY_PREFIXES) and k not in registry})
  for k in missing:
    rc = 1
    where = ", ".join(f"{r}:{ln} ({how})" for kk, r, ln, how in keys if kk == k)
    print(f"FAIL key-lint: fork key {k!r} is used but NOT registered in openpilot/common/params_keys.h -- {where}")
  print(f"{'ok  ' if not missing else 'FAIL'} key-lint: {len(fork_keys)} fork-owned key(s) used, {len(missing)} unregistered: {fork_keys}")
  if other_unknown:
    print(f"info key-lint (report-only, non-fork): {len(other_unknown)} key literal(s) not in params_keys.h: {other_unknown}")

  # (b) writer lint
  read_fields = sorted({f for f, *_ in reads if f in sp_fields})
  written = {f for f, *_ in writes}
  unwritten = [f for f in read_fields if f not in written and f not in WRITER_ALLOWLIST]
  for f in unwritten:
    rc = 1
    where = ", ".join(f"{r}:{ln}" for ff, r, ln in reads if ff == f)
    print(f"FAIL writer-lint: CarParamsSP.{f} is READ but has NO WRITER in setup_interfaces / brand interfaces -- read at {where}")
  tag = "ok  " if not unwritten else "FAIL"
  print(f"{tag} writer-lint: {len(read_fields)} CarParamsSP field(s) read {read_fields}; unwritten (not allowlisted): {unwritten}")
  if verbose:
    for f in read_fields:
      print(f"     {f}: writers {sorted({f'{r}:{ln}' for ff, r, ln in writes if ff == f})}")

  # (c) env-override ban
  bad = []
  for name, rel, ln, fn in envs:
    if name in ENV_ALLOWLIST:
      continue
    guard = next((v for (suffix, gfn), v in ENV_GUARD_ALLOWLIST.items() if rel.endswith(suffix) and gfn == fn), {})
    if name in guard:
      continue
    bad.append((name, rel, ln, fn))
  for name, rel, ln, fn in bad:
    rc = 1
    print(f"FAIL env-ban: production code reads FCA11 feature-flag env {name!r} at {rel}:{ln} (in {fn}) -- " +
          "feature flags must come from a registered param via setup_interfaces, never os.environ")
  allowed = sorted({n for n, *_ in envs if n in ENV_ALLOWLIST})
  guarded = sorted({e[0] for e in envs if e not in bad and e[0] not in ENV_ALLOWLIST})
  tag = "ok  " if not bad else "FAIL"
  print(f"{tag} env-ban: {len(envs)} FCA11_* env read(s), {len(bad)} banned; allowlisted: {allowed} + guards {guarded}")
  print(f"scanned {len(files)} production file(s); registry {len(registry)} keys; CarParamsSP {len(sp_fields)} fields")
  return rc


def main(argv=None):
  ap = argparse.ArgumentParser(description="fork param reachability guard (key lint, CarParamsSP writer lint, FCA11 env ban)")
  ap.add_argument("--tree", required=True, help="repo or prebuilt-tree root (has openpilot/common/params_keys.h)")
  ap.add_argument("--opendbc", default=None, help="dir containing opendbc/ (default: <tree>/opendbc_repo, then <tree>)")
  ap.add_argument("-v", "--verbose", action="store_true")
  args = ap.parse_args(argv)
  return run(args.tree, args.opendbc, args.verbose)


if __name__ == "__main__":
  sys.exit(main())
