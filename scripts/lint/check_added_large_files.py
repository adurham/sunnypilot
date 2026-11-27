#!/usr/bin/env python3
import argparse
import fnmatch
import math
import os
import subprocess


def lfs_files(filenames: list[str]) -> set[str]:
  if not filenames:
    return set()

  result = subprocess.run(
    ("git", "check-attr", "filter", "-z", "--stdin"),
    input="\0".join(filenames),
    check=True,
    capture_output=True,
    text=True,
  )
  fields = result.stdout.rstrip("\0").split("\0") if result.stdout else []
  return {fields[i] for i in range(0, len(fields), 3) if fields[i + 2] == "lfs"}


def excluded_patterns(path: str | None) -> list[str]:
  """One glob per line; a line may carry a reason after the glob. Blank lines and # comments are skipped."""
  if path is None or not os.path.exists(path):
    return []
  patterns = []
  with open(path) as f:
    for line in f:
      line = line.strip()
      if not line or line.startswith("#"):
        continue
      patterns.append(line.split("#", 1)[0].strip())
  return patterns


def check_added_large_files(filenames: list[str], max_kb: int, exclude_from: str | None = None) -> int:
  failed = False
  ignored = lfs_files(filenames)
  excluded = excluded_patterns(exclude_from)
  for filename in filenames:
    if filename in ignored:
      continue
    if any(fnmatch.fnmatch(filename, pattern) for pattern in excluded):
      continue

    size_kb = math.ceil(os.stat(filename).st_size / 1024)
    if size_kb > max_kb:
      print(f"{filename} ({size_kb} KB) exceeds {max_kb} KB.")
      failed = True

  return int(failed)


def main() -> int:
  parser = argparse.ArgumentParser(description="Check that tracked files do not exceed a size limit.")
  parser.add_argument("filenames", nargs="*")
  parser.add_argument("--maxkb", type=int, default=500, help="maximum allowable size in KiB")
  parser.add_argument("--exclude-from", default=None, help="file of glob patterns to allow (one per line, optional reason)")
  args = parser.parse_args()
  return check_added_large_files(args.filenames, args.maxkb, args.exclude_from)


if __name__ == "__main__":
  raise SystemExit(main())
