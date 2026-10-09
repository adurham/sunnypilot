# check_device.sh — read-only device audit

Run **from the Mac**:

```bash
bash openpilot/offload/device/check_device.sh comma@comma-b203ed6e.local
bash openpilot/offload/device/check_device.sh comma@comma-b203ed6e.local --no-retry
```

Safe to run **while driving**: it performs no writes, starts/stops no processes, and never
touches `/data`. The only executable it launches is the bridge binary under `timeout 2` (it
loops by design); nothing is mutated.

## What it asserts and prints

One `PASS` / `WARN` / `FAIL` line per check, then a `SUMMARY` block and a verdict. Exit 0 if
no `FAIL`.

| # | Check | Expected | Severity if not |
|---|---|---|---|
| 0 | ssh reachability (retried up to 3×) | `SSH_OK` | FAIL (aborts, `NOT AUDITED`) |
| 1 | device python | `/usr/local/venv/bin/python3` = Python 3.12 | WARN |
| 2 | `capnp` import | ok | FAIL |
| 3 | `pyzmq` import | ok (libzmq version shown) | FAIL |
| 4 | bridge binary + no-args run | `/data/openpilot/.../bridge` executable; `rc=124` under `timeout 2` | FAIL |
| 5 | UDC state | `a600000.dwc3` present, state shown | WARN |
| 6 | `AdbEnabled` + `adbd` service | param shown; service inactive by default | WARN (informational) |
| 7 | tether interface | `usb0` present **after** `adbd` is started | WARN (expected absent pre-bring-up) |
| 8 | `/data` free space | < 80% used | WARN ≥80 / FAIL ≥90 |
| 9 | running modeld flavor | `modeld_v2` (tinygrad) vs stock | WARN if unclear/absent |
| 10 | live cereal service list + ZMQ ports | names+ports for FORWARD/RETURN (from `contract.py`) | WARN on any missing |
| 11 | `OffloadMode` param | **absent** (expected) | WARN if present (offloadd would be live) |
| 12 | tree branch/rev | branch + short SHA shown | PASS |

## User-gated tether command (the script does NOT run this)

```
ssh comma@comma-b203ed6e.local
# set AdbEnabled=1, then:
sudo systemctl start adbd          # builds NCM+FunctionFS gadget on UDC a600000.dwc3
```

`usb0` will not exist until `adbd` runs; before that, check #7 is a WARN by design.
