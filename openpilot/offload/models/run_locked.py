#!/usr/bin/env python3
"""GPU-serialization helper for macOS (flock(1) is absent on Darwin).

Acquires an exclusive fcntl.flock on /tmp/offload.gpu.lock, holds it for the
lifetime of the child command, then releases.  Stands in for
`flock /tmp/offload.gpu.lock -c "<cmd>"` required by INTERFACES.md §0.
"""
import fcntl
import subprocess
import sys

LOCK = "/tmp/offload.gpu.lock"


def main() -> int:
    with open(LOCK, "w") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        return subprocess.call(sys.argv[1:])


if __name__ == "__main__":
    sys.exit(main())
