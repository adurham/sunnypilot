"""Off-thread params access for hardwared's 2 Hz deviceState publisher.

2026-11-04 fork (routes 189/18b/193): hardwared's deviceState went silent 3.8-8.0 s while every other service kept its
rate, and the 193 stall disengaged (commIssue). Forensics showed the publishing thread blocked in the kernel at a
DIFFERENT point of its loop each time, and the only things it touches at those points are files on /data (ext4, 90 %
full, `discard`, loggerd writing continuously, the deleter unlinking whole routes). Per iteration it did ~20 params file
reads, a non-blocking put or two, and - unconditionally - `set_offroad_alert("Offroad_TiciSupport", False)`, which is
Params().remove() = flock + unlink + fsync(/data/params/d). Every 60 s and on every engage edge it also did
`put(block=True)` = write + fsync(file) + rename + fsync(dir). When ext4 stalls its journal commit, any of those waits
seconds, and with them the deviceState frame.

`ParamsWorker` makes the publisher never touch /data:
  * reads: every key the publisher needs is registered up front and refreshed on the worker thread; `get()` /
    `get_bool()` return the cached value (a dict lookup). A stalled refresh costs freshness, never a frame.
  * writes/removes: queued per key, latest value wins, persisted on the worker with block=True semantics. The cache
    is updated immediately, so the publisher reads its own writes, and a refresh that raced with a write cannot
    resurrect the old value.
  * offroad alerts: `set_alert()` is idempotent per alert (no op if (show, extra) is unchanged) and the
    `set_offroad_alert` call itself runs on the worker.
  * probes: other filesystem checks (e.g. "is the big model on disk") registered as callables, refreshed the same way.
Any single worker op over SLOW_OP_S is cloudlogged with the key, so a future /data stall names itself.
"""
import threading
import time
from collections.abc import Callable, Iterable
from typing import Any

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert

SLOW_OP_S = 1.0

_PUT, _REMOVE, _ALERT = "put", "remove", "alert"


class ParamsWorker:
  def __init__(self, read_keys: Iterable[str], probes: dict[str, Callable[[], Any]] | None = None,
               refresh_period: float = 0.5, params: Params | None = None, start: bool = True):
    # Construction runs on the caller (hardwared startup, before the publish loop): one synchronous fill so the first
    # iteration sees real values, exactly as before.
    self.params = params if params is not None else Params()
    self.refresh_period = refresh_period
    self._probes = dict(probes or {})
    self._read_keys = list(dict.fromkeys(read_keys))

    self._lock = threading.Lock()
    self._wake = threading.Event()
    self._end = threading.Event()
    self._cache: dict[str, Any] = {}
    self._probe_values: dict[str, Any] = {}
    self._gen: dict[str, int] = {}            # bumped on every local write; a refresh started before it is discarded
    self._pending: dict[str, tuple] = {}      # key -> latest op (insertion order kept for fairness)
    self._alerts: dict[str, tuple[bool, str | None]] = {}
    self._last_put: dict[str, Any] = {}       # for put(..., dedupe=True)
    self._idle = threading.Event()
    self._idle.set()

    self._refresh()
    self._thread = threading.Thread(target=self._run, name="hardwared-params", daemon=True)
    if start:
      self._thread.start()

  # ---------------- publisher side: never blocks on the filesystem ----------------

  def get(self, key: str, default: Any = None) -> Any:
    with self._lock:
      if key not in self._cache:
        raise KeyError(f"param {key} not registered with ParamsWorker")
      value = self._cache[key]
    return default if value is None else value

  def get_bool(self, key: str) -> bool:
    return bool(self.get(key))

  def probe(self, name: str) -> Any:
    with self._lock:
      return self._probe_values[name]

  def put(self, key: str, value: Any, block: bool = False, dedupe: bool = False) -> None:
    """Queue a write. `block` is accepted for drop-in compatibility; every write is persisted on the worker with fsync.
    dedupe=True skips the write when the value equals the last one queued by this worker (per-iteration status bits)."""
    with self._lock:
      if dedupe and key in self._last_put and self._last_put[key] == value:
        return
      self._last_put[key] = value
      if key in self._cache:
        self._cache[key] = value
      self._gen[key] = self._gen.get(key, 0) + 1
      self._pending.pop(key, None)
      self._pending[key] = (_PUT, value)
      self._idle.clear()
    self._wake.set()

  def put_bool(self, key: str, value: bool, block: bool = False, dedupe: bool = False) -> None:
    self.put(key, bool(value), block=block, dedupe=dedupe)

  def remove(self, key: str) -> None:
    with self._lock:
      self._last_put.pop(key, None)
      if key in self._cache:
        self._cache[key] = None
      self._gen[key] = self._gen.get(key, 0) + 1
      self._pending.pop(key, None)
      self._pending[key] = (_REMOVE,)
      self._idle.clear()
    self._wake.set()

  def set_alert(self, alert: str, show: bool, extra_text: str | None = None) -> None:
    """Idempotent offroad alert. Same contract as hardwared's old set_offroad_alert_if_changed (first call always
    writes, so a stale alert left on disk by a previous run is cleared once), but the write happens off-thread."""
    with self._lock:
      if self._alerts.get(alert) == (show, extra_text):
        return
      self._alerts[alert] = (show, extra_text)
      self._gen[alert] = self._gen.get(alert, 0) + 1
      self._pending.pop(alert, None)
      self._pending[alert] = (_ALERT, show, extra_text)
      self._idle.clear()
    self._wake.set()

  # ---------------- lifecycle ----------------

  def flush(self, timeout: float | None = None) -> bool:
    """Wait until every queued op is on disk. For shutdown and tests only - never call from the publish loop."""
    self._wake.set()
    return self._idle.wait(timeout)

  def stop(self, timeout: float = 5.0) -> None:
    self.flush(timeout)
    self._end.set()
    self._wake.set()
    if self._thread.is_alive():
      self._thread.join(timeout)

  # ---------------- worker side ----------------

  def _run(self) -> None:
    next_refresh = time.monotonic() + self.refresh_period
    while not self._end.is_set():
      self._wake.wait(max(0., next_refresh - time.monotonic()))
      self._wake.clear()
      self._drain()
      if time.monotonic() >= next_refresh:
        self._refresh()
        next_refresh = time.monotonic() + self.refresh_period

  def _drain(self) -> None:
    while True:
      with self._lock:
        if not self._pending:
          self._idle.set()
          return
        key = next(iter(self._pending))
        op = self._pending.pop(key)
      self._timed(f"{op[0]} {key}", self._apply, key, op)

  def _apply(self, key: str, op: tuple) -> None:
    try:
      if op[0] == _PUT:
        self.params.put(key, op[1], block=True)
      elif op[0] == _REMOVE:
        self.params.remove(key)
      else:
        set_offroad_alert(key, op[1], extra_text=op[2])
    except Exception:
      cloudlog.exception(f"hardwared params worker: {op[0]} {key} failed")

  def _refresh(self) -> None:
    for key in self._read_keys:
      with self._lock:
        gen = self._gen.get(key, 0)
      try:
        value = self._timed(f"get {key}", self.params.get, key)
      except Exception:
        cloudlog.exception(f"hardwared params worker: get {key} failed")
        continue
      with self._lock:
        # a write queued (or applied) after this read started is newer than what we read: keep it
        if self._gen.get(key, 0) == gen and key not in self._pending:
          self._cache[key] = value
    for name, fn in self._probes.items():
      try:
        value = self._timed(f"probe {name}", fn)
      except Exception:
        cloudlog.exception(f"hardwared params worker: probe {name} failed")
        continue
      with self._lock:
        self._probe_values[name] = value

  @staticmethod
  def _timed(what: str, fn, *args):
    t0 = time.monotonic()
    try:
      return fn(*args)
    finally:
      dt = time.monotonic() - t0
      if dt > SLOW_OP_S:
        cloudlog.event("hardwared params op slow", op=what, duration=round(dt, 3), error=True)
