"""
Fork: ESC (HECU) security-access PROBE -- request the 0x27 seed and write the CURRENT 0x0103 variant-coding
value straight back (a no-op by construction), inside openpilot's own fingerprint window.
(adurham/sunnypilot; car-features/esc-probe-0027-report.md)

Why this exists
---------------
Question to answer on the car: does WRITING the ESC's variant-coding DID 0x0103 require UDS security access
(0x27) at all? The one-minute probe is deliberately the smallest possible write: read 0x0103 as it is, ask for
the 0x27 seed, then write those exact bytes back with 0x2E. Writing a value onto itself is a no-op for the ESC's
configuration; what is informative is the *reply*:

  * a refused seed + a refused write  -> the write path is gated by security access;
  * a refused seed + an ACCEPTED write -> the ESC takes 0x2E without an unlock (the gate is elsewhere/absent);
  * an all-zero seed                 -> "already unlocked", the write is attempted anyway to see if it lands.

Phase 1 never sends the key (0x27 sub 0x02) -- asking is the whole probe.

Vendor-confirmed write flow (GIT VariantCodingTable, decoded 2026-10-05; CN7N ESC block SecuritySupported=0)
------------------------------------------------------------------------------------------------------------
The vendor's own variant-coding table for THIS car's ESC (GIT VariantCodingTable_HY.git.xml, the CN7N/"Elantra N"
block -- HECU 58910-IB000, CAN IDs 0x7D1/0x7D9) exposes the authoritative write sequence as plain request hex:

    <Backup requestvalue="07D103220103">          -> 0x7D1: 03 22 01 03      read the current 0x0103 value
    <Input  requestvalue="07D1021003">            -> 0x7D1: 02 10 03         ENTER EXTENDED SESSION
    <Input  requestvalue="07D1072E0103$01$$02$$03$$04$"> -> 0x7D1: 07 2E 01 03 <4 code bytes>   write

and `<Security SecuritySupported="0" Securityindex="0" CANID="07D1"/>` -- the vendor tool never sends 0x27 for this
ECU (the 27 01 templates exist only for the Kia CONTI/MANDO entries with security indexes 26300/270100). This probe
still ASKS for the 0x27 seed (that contrast is informative), but mirrors the vendor exactly at the write: read 0x0103
-> 10 03 extended session -> 2E 0103 no-op.

Phase 1 result / Phase 2 rationale
----------------------------------
Phase 1 ran on the car (2026-10-05 21:45Z; result
``car-features/esc-software/probe-results/20261005T214537Z-result.json``): the 0x0103 read answered (``62 01 03 90 06 03
50``), the ``10 03`` session answered (``50 03 00 32 01 F4``), the ``27 01`` seed request answered POSITIVELY with an
8-byte, static-looking seed (``67 01 5A B0 5A B0 5A B0 5A B0`` x4), the no-op ``2E`` write was REFUSED (``7F 2E 33`` =
securityAccessDenied), the re-read was unchanged, the multiplexer was restored, and nothing errored (0.425 s). That
refusal alone cannot distinguish (a) a write that genuinely requires a completed ``27`` unlock, (b) phase 1's OWN
pre-write ``27 01`` confounding it (a pending seed, or a vendor tool that sends no ``27`` for this ECU at all), or
(c) the Hyundai/Kia Security Gateway gating write services for unauthorized testers (its documented refusal is also
NRC ``0x33``).

Phase 2 discriminates between them by repeating the write in the EXACT vendor order with NO seed request before the
write (``READ 22 0103 -> 10 03 -> 2E no-op -> re-read``), then sending ONE ``27 01`` after the write purely as a data
point (is the seed still static across runs?). An accepted write (``6E 0103``) means the pre-write seed was the
confounder and the write path works without an unlock; ``7F 2E 33`` again means the gate is real and needs the key
algorithm / gateway work. The phase is selected by the ``phase`` key in ``state.json`` (1 = default, 2 =
vendor-exact).

Phase 2 result / Phase 3 rationale
----------------------------------
Phase 2 ran on the car (2026-10-05 22:12Z; result
``car-features/esc-software/probe-results/20261005T221227Z-result.json``): the vendor-exact order (read -> 10 03 ->
2E no-op -> re-read -> 27 01 sample) ALSO got ``7F 2E 33`` on the no-op write, and the post-write ``27 01`` again
returned a fresh-looking 8-byte seed. So the write refusal is NOT phase 1's own pre-write seed. What remains is: (a)
the ESC itself requires a completed ``27`` unlock, (b) the Hyundai/Kia Security Gateway (SGW) intercepts write
services from an unauthorized tester (its documented refusal is ALSO NRC ``0x33``), or (c) a blanket filter that
rejects the service outright in any session.

Phase 3 is a single-ignition "battery" (state ``{"probe_enabled": true, "phase": 3}``) that discriminates those:
  1. read 0x0103 (must parse, else the battery aborts with ``battery: no current value``);
  2. a 6-address ``27 01`` seed sweep -- ESC 0x7D1, CLU 0x7C6, TCU 0x7E1, EPS 0x7D4, CAM 0x7C4, CR 0x7B7 (300 ms
     each). Identical static seeds from several no-security modules would point at a gateway answering on the
     modules' behalf (an SGW that relays/relays-blocking would produce uniform replies);
  3. the no-op ``2E 0103`` in the DEFAULT session (no ``10 03``) -- same idempotent write;
  4. ``29 01`` (Authentication) to the ESC -- a live UDS auth service changes the picture;
  5. ``31 01 0000`` (RoutineControl start, routine 0x0000) -- ``7F 31 31`` (requestOutOfRange) means the module's own
     session logic answered, whereas ``33`` would mean the same blanket filter as the write;
  6. re-read 0x0103 (confirm unchanged);
  7. ``10 03`` extended session;
  8. the SAME no-op ``2E`` again in extended -- ``7F 2E 33`` in BOTH sessions is the signature of a blanket filter;
  9. LAST and exactly ONCE: ``27 02`` sendKey with the IDENTITY key = the exact 8 seed bytes the ESC returned in step
     2 (the cheapest candidate algorithm; if the ESC's key == seed this unlocks). It is ISO-TP multi-frame (10-byte
     payload: first frame + flow control + one consecutive frame). Mechanically enforced: the guard admits a ``27 02``
     only when it is the first of the run AND the key bytes equal the step-2 ESC seed -- any other key or a second
     attempt raises ``SafetyViolation``. ``67 02`` -> step 10; ``7F 27 xx`` -> STOP (no more keys, no write);
  10. only if step 9 returned ``67 02``: ``10 03`` -> the no-op ``2E`` -> re-read (still the no-op by construction).

Phase 3 uses a 30 s budget (``RUN_BUDGET_S_PHASE3``); phases 1/2 keep 15 s. Per-address silent-abort counting is
unchanged. Phases 1/2 remain byte-identical: the 0x29/0x31/0x27-0x02 service/frame shapes are admitted only when
``phase == 3``, and the client only walks extra request addresses in the battery.

Phase 4 result / rationale
--------------------------
Phase 3 ran on the car (2026-10-05 23:21Z; result
``car-features/esc-software/probe-results/20261005T232115Z-result.json``): in the DEFAULT session the ESC refused EVERY
probe -- ``27 01`` -> ``7F 27 7F`` (serviceNotSupportedInActiveSession), ``29 01`` -> ``7F 29 7F``, ``31 01 0000`` ->
``7F 31 33``, and the no-op ``2E`` -> ``7F 2E 33`` in BOTH the default and the extended session. The seed is
SESSION-GATED: it only answers in the EXTENDED session (phase 1/2 proved ``67 01`` there). The one thing never tried is
the key itself: ``27 02`` (sendKey). Phase 4 completes that unlock:

  1. read ``22 0103`` (default; the extended-retry rules are the phase-1/2 ones; no usable read -> abort, no key,
     no write);
  2. ensure the EXTENDED session (``10 03``; skipped only if the read's retry already entered it). Phase 4 REQUIRES a
     POSITIVE ``50 03``: silence OR a negative -> abort (recorded), no key, no write (stricter than phase 3's
     "any answer continues", because seeds never answer outside the extended session);
  3. ``27 01`` requestSeed IN EXTENDED. REQUIRES a positive ``67 01`` with >=2 seed bytes; otherwise abort (recorded);
  4. wait ~500 ms (the vendor ``delaytime``);
  5. resolve the candidate key bytes from the seed: ``identity2`` = seed[:2], ``identity4`` = seed[:4], ``identity8`` =
     the whole seed, ``algo`` = ``openpilot.sunnypilot.fork.esc_probe_seedkey.key_for(seed, state["algo"])`` (the
     recovered G-scan CalKeyAlgorithm_* family, selectable by Securityindex 27100..27400; a missing/failing module, an
     unknown ``algo``, or a zero-byte seed that the vendor path bails on aborts, recorded), ``hex`` =
     ``bytes.fromhex(state["key_hex"])``. ONLY lengths 2, 4, 8 are admissible. The 8-byte CONSTRUCTION modes wrap the
     same resolvers into exactly 8 wire bytes, because the car proved ``27 02`` wants an 8-byte key (a 4-byte key drew
     ``7F 27 13`` incorrect-length; the 8-byte seed-as-key drew ``7F 27 35`` invalidKey -- so 8 bytes is the accepted
     LENGTH and VALUE iteration begins): ``algo8`` = the algo key repeated to fill 8 (2-byte k -> k*4, 4-byte -> k*2);
     ``algo8w`` = the algo key's FIRST TWO BYTES repeated to fill 8 (``k[:2]*4`` = ``[lo,hi]x4`` -- the wire shape of
     the on-car seed, in case the layer repeats the underlying 2-byte value rather than the whole 4-byte render);
     ``algo8p`` = the algo key + zero padding (2-byte -> k+6 zeros, 4-byte -> k+4 zeros); ``repeat8`` = the seed's own
     2-byte value repeated (``seed[:2]*4``); ``hex8`` = exactly 8 bytes from ``state["key_hex"]`` (any other length
     aborts -- stricter than ``hex``);
  6. send ``27 02`` ONCE: 2-byte key -> ``04 27 02 K1 K2``; 4-byte -> ``06 27 02 K1..K4``; 8-byte -> the ISO-TP
     multi-frame ``10 0A 27 02 K1..K4`` + one consecutive frame ``21 K5..K8`` (the phase-3 machinery);
  7. on ``67 02``: the no-op ``2E 0103 <readback>`` (the SAME readback guard as always) then a re-read -- the payoff.
     ``7F 27 35/36/37`` (invalidKey / exceeded attempts / required time delay) or silence -> recorded, STOP (no write).

The key_mode is the ``key_mode`` state key (default ``identity2`` when absent). A phase-4 ``27 02`` frame is admissible
ONLY when ``phase == 4``, ONLY as the first key of the run (the same counter phase 3 uses), and ONLY when its key bytes
equal the RESOLVED candidate (``guard_frame`` takes the resolved bytes and re-checks them at the single TX site). Phase 3
keeps its own identity-key path unchanged. A malformed ``key_mode`` is inert (``skip: unknown key_mode``).

Phase 5 -- the READ-ONLY capability battery (state ``{"probe_enabled": true, "phase": 5}``)
-------------------------------------------------------------------------------------------
Phase 4 iterates KEY VALUES; phase 5 instead maps what the ESC will even TALK ABOUT, parked, without ever reaching a key
or a write. It is a FIXED, read-only frame list, one ignition, budget ``RUN_BUDGET_S_PHASE5`` = 40 s, and it NEVER
aborts on a refusal -- it records every frame's request/response/NRC/timeout/latency and moves on:

  1. ``fp_canary``: ``22 F1 00`` -- the functional-read canary DID;
  2. ``read_esc``: ``22 01 03`` -- the current 0x0103 value (start);
  3. ``session``: ``10 03`` -- enter the extended session;
  4. ``seed1`` / ``seed2``: two ``27 01`` requestSeed samples;
  5. ten 2-byte sub-probes ``27 03/05/07/09/0B/0D/0F/11/41/61`` (the other requestSeed-shaped sub-functions);
  6. seven 1-byte service probes ``23/29/31/34/35/36/37`` (bare, no sub-function);
  7. ``seed3``: a third ``27 01`` sample;
  8. ``read_esc_end``: re-read ``22 01 03`` (confirm unchanged);
  9. two extra-address peeks: a single ``10 03`` to request addr 0x770 (listen 0x778) and to 0x7A0 (listen 0x7A8) --
     do OTHER OBD addresses answer, or is the ESC's own 0x7D1 the only talker?

The summary adds: ``seed1``/``seed2``/``seed3`` hex, ``seed_stable_12`` (seed1 == seed2), ``seed_stable_all`` (all
three), ``fp_canary_hex``, ``value_start``, ``value_end``. Guards, mechanically enforced for phase 5 (``SafetyViolation``
raised BEFORE any frame is built):

  * ONLY the exact frames above are admissible; anything else raises;
  * ``27``: ONLY sub 0x01 (2-byte frame) and the exact ten sub-probes (2-byte frames); NO ``27 02`` EVER in phase 5;
  * ``10``: only sub 0x03; the extra request addresses are ONLY {0x770, 0x7A0} and ONLY for the ``10 03`` frame;
  * ``22``: only DIDs {F100, 0103};
  * services 23/29/31/34/35/36/37 only as the bare 1-byte frame;
  * NO ``2E``, no ``31`` sub 0x01, no ``29`` sub 0x01, and NO multi-frame sends at all (RX multi-frame is fine);
  * phases 1-4 stay byte-identical (the phase-5 shapes are admitted only when ``phase == 5``);
  * phase 5 respects ``done_ignition`` (never runs twice per ignition).

Phase 5 adds no new services to the base allowlist; it is a strict, self-contained subset.

Phase 6 -- the SECURITY-POLICY MATRIX + DID SWEEP (state ``{"probe_enabled": true, "phase": 6}``)
-----------------------------------------------------------------------------------------------
Phase 5 maps what the ESC will TALK ABOUT read-only; phase 6 maps its ATTEMPT/SESSION policy: how many ``27 02`` key
attempts it tolerates, whether a session reset (``10 01`` -> ``10 03``) clears an exhaustion lockout, whether the seed
changes after a failed key, and what the identification DIDs say (the SW-ID/supplier/serial strings that index Hyundai's
security DB). One ignition, budget ``RUN_BUDGET_S_PHASE6`` = 60 s, every frame to the ESC (0x7D1 -> 0x7D9):

  1. ``22 01 03`` -> ``value_start`` (the only abort gate: no usable read stops the run);
  2. ``10 03`` extended session (record; if refused/silent the key part is skipped but the DID sweep still runs);
  3. ``27 01`` -> S1;  4. ``27 01`` -> S2  (seed stability before any key);
  5. ``27 02`` + ZERO key (``00``x8) -> R1; a ``0x36``/``0x37`` NRC sets ``lockout_seen`` and jumps to step 8;
  6. (adaptive) ``27 01`` -> S3; ``27 02`` zero key -> R2;  7. (adaptive) ``27 01`` -> S4; ``27 02`` zero key -> R3
     -- at most 3 pre-cycle key attempts;
  8. ALWAYS: ``10 01``; ``10 03``; ``27 01`` -> S5; ``27 02`` zero key -> R4  (the cycle-reset test; the ONLY 4th attempt);
  9. DID sweep (extended): ``22 F186``; ``22 F187``; ``22 F190``; ``22 F199``; ``22 F18A``; ``22 F18C``; ``22 F191``;
     ``22 F195``;
  10. ``10 02`` programming-session probe -> record; if POSITIVE, ``27 01`` -> S6 (NO key in the programming session);
  11. ``19 02 A5`` -> record (is a security DTC visible?);
  12. clean leave ``10 01``; then ``22 01 03`` -> ``value_end``.

Every frame is recorded (req hex, resp hex or silence, nrc, timeout, ms). Summary fields: ``S1``..``S6``, ``R1``..``R4``
(nrc + resp + positive), ``seed_stable_pre`` (S1 == S2), ``seed_after_fail`` (S3 equals S1/S2, or ``new:...``),
``lockout_seen``, ``cycle_reset`` (R4 not 0x36/0x37 after a lockout was seen), ``dids`` (six-hex -> resp),
``prog_session_1002``, ``dtc_19_02_a5``, ``value_start``, ``value_end``. Guards, mechanically enforced for phase 6
(``SafetyViolation`` raised BEFORE any frame is built):

  * ONLY the frames above are admissible; ``0x2E`` and ``0x3E`` are NOT in the phase-6 allowlist; no ``29``/``31`` ever;
  * ``27 02`` is admissible ONLY in phase 6, ONLY with key == the ZERO key (``00``x8), ONLY when a ``27 01`` requestSeed
    immediately preceded it (``guard_frame``'s ``seed == PHASE6_KEY`` sentinel / the client's ``_p6_seed_precedes`` flag),
    and at most ``PHASE6_MAX_KEY_ATTEMPTS`` (4) times per run -- the ONE 4th attempt being the step-8 cycle-reset;
  * ``10``: only sub ``01``/``02``/``03``;  ``22``: only DIDs {0103 + the exact eight-DID sweep};  ``19``: only sub
    ``02`` with mask ``A5``;  ``23``/``31``/``34``/``35``/``36``/``37`` are not admissible at all;
  * phase 6 NEVER writes (no ``2E``) and never runs ``31`` sub ``01`` / ``29`` sub ``01``;
  * phases 1-4 stay byte-identical and phase 5 keeps its own closed read-only allowlist (still NO ``27 02``).

In ``algo``/``algo8``/``algo8w``/``algo8p`` mode the ``algo`` state key (an optional string) selects the recovered algorithm: one of
``"27100"``/``"26300"``/``"26400"``/``"26700"``/``"26800"``/``"27400"`` (absent -> the vendor-exact ``27100`` default).
The resolved candidate hex is recorded as ``key_bytes`` (and the selector as ``algo``) in the result JSON and the
summary/cloudlog, BEFORE the ``27 02`` is sent, so a refused/silent key is still fully attributable. Every recovered
output is UNVERIFIED against hardware (no ground-truth seed->key pair exists); a wrong key is expected to draw NRC 0x35.

Safety, mechanically enforced here (tests: fork/tests/test_esc_probe_0027.py, mutation-proven)
----------------------------------------------------------------------------------------------
* Enabled only by a state file: ``/data/esc-probe-0027/state.json`` with ``{"probe_enabled": true}``. Missing file
  or flag -> ``run()`` returns ``{"skip": "not enabled"}`` and nothing happens. ``touch .../DISABLE`` forces off.
* At most ONE probe per ignition cycle: ``done_ignition`` is written once the standstill pre-check passes, BEFORE
  the multiplexer or any TX, so neither a crash nor a killed process can lose the cycle.
* Allowlist (checked twice: ``guard_service`` before a frame is built, ``guard_frame`` at the single TX site):
  0x22 ReadDataByIdentifier, 0x3E TesterPresent, 0x10 sub 0x03 (extended: retry a refused read, or the
  vendor-confirmed session step before the write),
  0x27 sub 0x01 ONLY in phases 1/2 (sendKey 0x02 raises there; phases 3/4 admit the single pinned key attempt),
  0x2E with DID 0x0103 ONLY -- and the 0x2E payload bytes MUST equal the bytes read back from 0x0103 in step 1 (an
  argument to the request, asserted at the TX site too).
* Only when stationary: the SAME definition as esc_diag -- gear Park (LVR12) and the wheels at standstill
  (WHL_SPD11; ``esc_diag.wheels_moving``), from fresh bus-0 frames. ``VehicleGate`` is IMPORTED, not copied, so
  there is exactly one standstill definition in the tree. Re-checked before every frame and while waiting for
  every answer; any violation aborts at once.
* The write happens ONLY if step 1 returned exactly ``62 01 03 <4 bytes>``; otherwise nothing is written.
* Never interferes: no car safety mode, no controls; OBD multiplexing is switched off in ``finally`` (SIGTERM too)
  and verified from pandaStates. Errors never propagate into card. Hard budget ``RUN_BUDGET_S``.

Output: ``/data/esc-probe-0027/<UTC>-result.json`` (raw request/response hex, TX log, duration, aborted/error,
mux_restored) + a compact ``esc_probe_0027`` cloudlog event, so the result is also in that drive's rlog.
"""
import json
import os
import signal
import time
from collections.abc import Callable

from opendbc.car.can_definitions import CanData

# The standstill definition lives in exactly one place: esc_diag. Import it, do not copy it.
from openpilot.sunnypilot.fork.esc_diag import Abort, FLOW_CONTROL_FRAME, SafetyViolation, VehicleGate, \
  build_single_frame, wheels_moving
# The DTC service constant also comes from esc_diag (one definition): phase 6's 19 02 A5 probe reuses it.
from openpilot.sunnypilot.fork.esc_diag import SVC_READ_DTC_INFORMATION

# ---------------------------------------------------------------------------------------------------------------------
# Probe allowlist -- a strict superset of esc_diag's read set (0x27/0x2E added) and nothing else.
# ---------------------------------------------------------------------------------------------------------------------
SVC_READ_DATA_BY_IDENTIFIER = 0x22
SVC_DIAGNOSTIC_SESSION_CONTROL = 0x10
SVC_TESTER_PRESENT = 0x3E
SVC_SECURITY_ACCESS = 0x27
SVC_WRITE_DATA_BY_IDENTIFIER = 0x2E
# Phase-3-only services. They stay OUT of ALLOWED_SERVICES so phases 1/2 are byte-identical: the guards admit them
# only when the caller passes phase == 3, and the tests drive both phases through the same guard.
SVC_AUTHENTICATION = 0x29
SVC_ROUTINE_CONTROL = 0x31
ALLOWED_SERVICES = frozenset({SVC_READ_DATA_BY_IDENTIFIER, SVC_DIAGNOSTIC_SESSION_CONTROL, SVC_TESTER_PRESENT,
                              SVC_SECURITY_ACCESS, SVC_WRITE_DATA_BY_IDENTIFIER})
ALLOWED_PHASE3_SERVICES = frozenset({SVC_AUTHENTICATION, SVC_ROUTINE_CONTROL})
ALLOWED_SESSION_SUBFUNC = 0x03   # extended only (used to retry a refused 0x0103 read)
ALLOWED_SEC_SUBFUNC = 0x01       # requestSeed; sendKey (0x02) is admitted ONLY in the phase-3 battery (single attempt)
ALLOWED_SEC_SUBFUNC_KEY = 0x02   # sendKey -- phase 3 (identity key == the step-2 seed) and phase 4 (resolved candidate)
ALLOWED_KEY_LENGTHS = (2, 4, 8)  # phase 4: the ONLY admissible candidate-key sizes (2/4 -> single frame, 8 -> ISO-TP)
# phase-4 candidate-key derivation modes. The 8-byte CONSTRUCTION modes ("algo8"/"algo8p"/"repeat8"/"hex8") were added
# after the on-car run proved 27 02 wants an 8-byte key: a 4-byte key drew 7F 27 13 (incorrect length) and the
# 8-byte seed-as-key (identity8) drew 7F 27 35 (invalidKey) -- so 8 bytes is the accepted LENGTH and VALUE iteration
# can begin. Each of the new modes resolves to exactly 8 wire bytes (see resolve_key).
KEY_MODES = ("identity2", "identity4", "identity8", "algo", "hex",
             "algo8", "algo8w", "algo8p", "repeat8", "hex8")   # phase-4 candidate-key derivation modes
ALLOWED_AUTH_SUBFUNC = 0x01      # Authentication (0x29) start -- phase 3 only
ALLOWED_ROUTINE_SUBFUNC = 0x01   # RoutineControl (0x31) start -- phase 3 only
ROUTINE_CONTROL_ID = 0x0000      # the "is any routine even answered" probe
DID_VARIANT_CODING = 0x0103      # the ESC's variant-coding DID; the ONLY DID this module may write
WRITE_DID = DID_VARIANT_CODING

ESC_REQ_ADDR = 0x7D1
ESC_RSP_ADDR = 0x7D9
ESC_BUS = 1                      # OBD port (needs OBD multiplexing); the ESC never answers on bus 0

# Phase 3 walks the same OBD bus to five more modules (request addr -> response addr). Every one is reached with OBD
# multiplexing on, bus 1, 8-byte ISO-TP frames; the request allowlist stays closed over exactly these pairs.
ADDR_PAIRS = ((0x7D1, 0x7D9),   # ESC  (HECU)   -- the write target
              (0x7C6, 0x7CE),   # CLU  (cluster)
              (0x7E1, 0x7E9),   # TCU  (transmission)
              (0x7D4, 0x7DC),   # EPS  (steering)
              (0x7C4, 0x7CC),   # CAM  (front camera / LKAS)
              (0x7B7, 0x7BF))   # CR   (cruise radar / SCC)
ALLOWED_REQ_ADDRS = frozenset(req for req, _ in ADDR_PAIRS)
ALL_RESP_ADDRS = frozenset(rsp for _, rsp in ADDR_PAIRS)
RESP_OF_REQ = dict(ADDR_PAIRS)
REQ_OF_RESP = {rsp: req for req, rsp in ADDR_PAIRS}
SEED_SWEEP_ADDRS = tuple(req for req, _ in ADDR_PAIRS)

# ---------------------------------------------------------------------------------------------------------------------
# Phase 5 -- the READ-ONLY capability battery. It answers "what does this ESC actually support / how far can a parked
# tester get" WITHOUT ever reaching a key or a write. One ignition, a FIXED scripted frame list to the ESC (0x7D1) plus
# two single-frame peeks at OTHER request addresses (0x770, 0x7A0) that stay strictly bounded to the `10 03` frame.
# The admissible set is closed HERE so the guards can enforce it mechanically (no 27 02, no 2E, no multi-frame TX).
# ---------------------------------------------------------------------------------------------------------------------
PHASE5_DID_FP_CANARY = 0xF100     # functional-read canary DID
PHASE5_RESP_DEFAULT = RESP_OF_REQ[ESC_REQ_ADDR]
# the ten 27 03/05/07/09/0B/0D/0F/11/41/61 sub-probes (2-byte frames, requestSeed-shaped sub-functions)
PHASE5_SEC_SUB_PROBES = (0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x11, 0x41, 0x61)
PHASE5_SVC_PROBES = (0x23, 0x29, 0x31, 0x34, 0x35, 0x36, 0x37)   # bare 1-byte service probes
PHASE5_EXTRA_ADDRS = ((0x770, 0x778), (0x7A0, 0x7A8))           # extra request addr -> listen addr (10 03 only)
PHASE5_EXTRA_REQ_ADDRS = frozenset(req for req, _ in PHASE5_EXTRA_ADDRS)
PHASE5_EXTRA_RESP_OF_REQ = dict(PHASE5_EXTRA_ADDRS)
# The phase-5 27 sub-functions: requestSeed (0x01) plus the ten 2-byte sub-probes. sendKey (0x02) is NEVER admissible.
PHASE5_SEC_SUBFUNCS = frozenset({ALLOWED_SEC_SUBFUNC, *PHASE5_SEC_SUB_PROBES})
# The phase-5 DIDs: the canary functional read and the variant-coding read, nothing else.
PHASE5_DIDS = frozenset({PHASE5_DID_FP_CANARY, DID_VARIANT_CODING})
# The phase-5 service allowlist: a strict SUBSET of the base set (note: 0x2E is absent, and so is 0x3E) plus the bare
# 1-byte probes. Kept as its own frozenset so phases 1-4 stay byte-identical.
ALLOWED_PHASE5_SERVICES = frozenset({SVC_READ_DATA_BY_IDENTIFIER, SVC_DIAGNOSTIC_SESSION_CONTROL,
                                     SVC_SECURITY_ACCESS, *PHASE5_SVC_PROBES})
# Every phase-5 exchange is a single ISO-TP frame (1- or 2-byte payload) -- there is NO multi-frame TX in phase 5.
PHASE5_MAX_FRAMES = 1

# ---------------------------------------------------------------------------------------------------------------------
# Phase 6 -- the SECURITY-POLICY MATRIX + DID SWEEP battery. One ignition, all frames to the ESC (0x7D1 -> 0x7D9),
# budget RUN_BUDGET_S_PHASE6. It maps the ESC's ATTEMPT/SESSION policy with a ZERO key (8x00) and then sweeps the
# identification DIDs (the string that indexes Hyundai's security DB: SW-ID / supplier / serial, Mando vs Conti).
# The zero key is mechanically pinned: 27 02 is admissible ONLY in phase 6, ONLY with key == 8x00, ONLY immediately
# after a 27 01 requestSeed, and at most PHASE6_MAX_KEY_ATTEMPTS (4) times per run. The ONE 4th attempt is always the
# cycle-reset step (10 01 -> 10 03 -> 27 01 -> 27 02), so the matrix can never exceed four keys, ever.
# ---------------------------------------------------------------------------------------------------------------------
SESSION_SUBFUNC_DEFAULT = 0x01        # 10 01 default session (the cycle-reset's leave/enter, and the clean leave)
SESSION_SUBFUNC_PROGRAMMING = 0x02    # 10 02 programming session (probe only -- no key in it)
PHASE6_SESSION_SUBFUNCS = frozenset({SESSION_SUBFUNC_DEFAULT, SESSION_SUBFUNC_PROGRAMMING, ALLOWED_SESSION_SUBFUNC})
PHASE6_KEY = bytes(8)                 # the zero key the phase-6 matrix sends: exactly 8 zero bytes (00 x8)
PHASE6_MAX_KEY_ATTEMPTS = 4           # hard cap on 27 02 per run; the ONE 4th attempt is always the cycle-reset step
PHASE6_LOCKOUT_NRCS = (0x36, 0x37)    # exceededNumberOfAttempts / requiredTimeDelayNotExpired -> lockout flag
PHASE6_SWEEP_DIDS = (0xF186, 0xF187, 0xF190, 0xF199, 0xF18A, 0xF18C, 0xF191, 0xF195)   # SW-ID / supplier / serial
PHASE6_DIDS = frozenset({DID_VARIANT_CODING, *PHASE6_SWEEP_DIDS})   # 0x0103 (value_start/end) + the exact sweep
PHASE6_DTC_SUBFUNC = 0x02             # ReadDTCByStatusMask; the ONLY 0x19 sub phase 6 may send
PHASE6_DTC_MASK = 0xA5                # the ONLY status mask phase 6 may send
# The phase-6 service allowlist: a STRICT subset -- 0x2E (write) and 0x3E (tester present) are absent, and there is no
# 0x29/0x31 anywhere. Kept as its own frozenset so phases 1-5 stay byte-identical.
ALLOWED_PHASE6_SERVICES = frozenset({SVC_READ_DATA_BY_IDENTIFIER, SVC_DIAGNOSTIC_SESSION_CONTROL,
                                     SVC_SECURITY_ACCESS, SVC_READ_DTC_INFORMATION})

RESP_TIMEOUT_S = 0.25            # first answer frame
SEED_SWEEP_TIMEOUT_S = 0.3       # the 6-address 27 01 sweep waits up to 300 ms per module
PENDING_TIMEOUT_S = 2.0          # after NRC 0x78 responsePending
CF_TIMEOUT_S = 0.5               # between consecutive frames
RUN_BUDGET_S = 15.0              # hard cap on phases 1/2
RUN_BUDGET_S_PHASE3 = 30.0       # hard cap on the phase-3 battery (6-address sweep + one sendKey)
RUN_BUDGET_S_PHASE5 = 40.0       # hard cap on the phase-5 READ-ONLY capability battery (26 frames + 2 extra addrs)
RUN_BUDGET_S_PHASE6 = 60.0       # hard cap on the phase-6 policy matrix + DID sweep (~25 frames, incl. 4 zero keys)
SILENT_ABORT_N = 3               # consecutive requests with no answer at all = module not reachable, stop
STATE_KNOWN_WAIT_S = 0.5         # wait this long for the first fresh gear/wheel frames before the pre-check
KEEP_FILES = 60

OUT_DIR = "/data/esc-probe-0027"
TARGET_FINGERPRINTS = ("HYUNDAI_ELANTRA_2022_NON_SCC",)


def guard_service(service: int, subfunc: int | None, did: int | None = None, phase: int = 1) -> None:
  """Layer 1: which (service, sub-function, DID) triples may even be turned into a frame.

  Phase 3 additionally admits the two discriminating services and the single 27 02 sendKey. Phases 1/2 are byte-identical:
  with ``phase != 3`` the allowlist is exactly the original, so 0x29/0x31/0x27 sub 0x02 raise exactly as before.

  Phase 4 admits 0x27 sub 0x02 too (its whole point), under the same ``phase in (3, 4)`` gate.

  Phase 5 is READ-ONLY and STRICTER than everything else: a closed subset of services (0x2E and 0x3E are NOT admissible),
  0x27 restricted to requestSeed + the exact ten 2-byte sub-probes (sendKey 0x02 NEVER), 0x10 only sub 0x03, 0x22 only
  DIDs {F100, 0103}, and services 23/29/31/34/35/36/37 only as the bare sub-function-absent frame.

  Phase 6 is the SECURITY-POLICY MATRIX + DID SWEEP: 0x22 only DIDs {0103 + the exact eight-DID sweep}, 0x10 only subs
  {01, 02, 03}, 0x27 only requestSeed (0x01) / the phase-6 zero-key sendKey (0x02), and 0x19 only sub 0x02 (A5 mask).
  0x2E and 0x3E are NOT admissible; there is no 0x29/0x31 anywhere.
  """
  if phase == 6:
    if service not in ALLOWED_PHASE6_SERVICES:
      raise SafetyViolation(f"phase6: service 0x{service:02X} is not in the policy-matrix allowlist")
    if service == SVC_DIAGNOSTIC_SESSION_CONTROL and subfunc not in PHASE6_SESSION_SUBFUNCS:
      raise SafetyViolation(f"phase6: session control sub-function {subfunc!r} refused (only 0x01/0x02/0x03)")
    if service == SVC_SECURITY_ACCESS and subfunc not in (ALLOWED_SEC_SUBFUNC, ALLOWED_SEC_SUBFUNC_KEY):
      raise SafetyViolation(f"phase6: security-access sub {subfunc!r} refused (only requestSeed 0x01 / zero-key sendKey 0x02)")
    if service == SVC_READ_DATA_BY_IDENTIFIER and did not in PHASE6_DIDS:
      raise SafetyViolation(f"phase6: read of DID {did!r} refused (0x0103 + the exact F186/F187/F190/F199/F18A/F18C/F191/F195 sweep)")
    if service == SVC_READ_DTC_INFORMATION and subfunc != PHASE6_DTC_SUBFUNC:
      raise SafetyViolation(f"phase6: DTC sub-function {subfunc!r} refused (only 0x02 ReadDTCByStatusMask)")
    return
  if phase == 5:
    if service not in ALLOWED_PHASE5_SERVICES:
      raise SafetyViolation(f"phase5: service 0x{service:02X} is not in the read-only battery allowlist")
    if service == SVC_DIAGNOSTIC_SESSION_CONTROL and subfunc != ALLOWED_SESSION_SUBFUNC:
      raise SafetyViolation(f"phase5: session control sub-function {subfunc!r} refused (only 0x03 extended)")
    if service == SVC_SECURITY_ACCESS and subfunc not in PHASE5_SEC_SUBFUNCS:
      raise SafetyViolation(f"phase5: security-access sub {subfunc!r} refused (requestSeed 0x01 + the 27 sub-probes; NO 27 02)")
    if service == SVC_READ_DATA_BY_IDENTIFIER and did not in PHASE5_DIDS:
      raise SafetyViolation(f"phase5: read of DID {did!r} refused (only 0xF100 canary and 0x0103)")
    if service in PHASE5_SVC_PROBES and subfunc is not None:
      raise SafetyViolation(f"phase5: service 0x{service:02X} is admissible only as the bare 1-byte frame, not sub {subfunc!r}")
    return
  if service not in ALLOWED_SERVICES and not (phase in (3, 4) and service in ALLOWED_PHASE3_SERVICES):
    raise SafetyViolation(f"service 0x{service:02X} is not in the probe allowlist")
  if service == SVC_DIAGNOSTIC_SESSION_CONTROL and subfunc != ALLOWED_SESSION_SUBFUNC:
    raise SafetyViolation(f"session control sub-function {subfunc!r} refused (only 0x03 extended)")
  if service == SVC_SECURITY_ACCESS:
    allowed_sec = (ALLOWED_SEC_SUBFUNC, ALLOWED_SEC_SUBFUNC_KEY) if phase in (3, 4) else (ALLOWED_SEC_SUBFUNC,)
    if subfunc not in allowed_sec:
      extra = "; 0x02 sendKey in phases 3/4" if phase in (3, 4) else ""
      raise SafetyViolation(f"security-access sub-function {subfunc!r} refused (only 0x01 requestSeed{extra})")
  if service == SVC_AUTHENTICATION and subfunc != ALLOWED_AUTH_SUBFUNC:
    raise SafetyViolation(f"authentication sub-function {subfunc!r} refused (only 0x01 start)")
  if service == SVC_ROUTINE_CONTROL and subfunc != ALLOWED_ROUTINE_SUBFUNC:
    raise SafetyViolation(f"routine-control sub-function {subfunc!r} refused (only 0x01 start)")
  if service == SVC_WRITE_DATA_BY_IDENTIFIER and did != WRITE_DID:
    raise SafetyViolation(f"write to DID {did!r} refused (only 0x0103)")


def guard_frame(addr: int, dat: bytes, bus: int, readback: bytes | None = None, *, phase: int = 1,
                seed: bytes | None = None, key_attempts: int = 0, key: bytes | None = None,
                p6_seed: bool = False) -> None:
  """Layer 2: the exact frame shapes this module may emit, checked immediately before can_send.

  ``readback`` is the 4 bytes read from 0x0103 in step 1: a 0x2E frame is admitted only when its data bytes are
  byte-identical to it -- the write is a no-op by construction.

  Phase 3 adds three shapes, each mechanically enforced:
  * request addresses are the six OBD pairs in ``ALLOWED_REQ_ADDRS`` (still bus 1, 8 bytes);
  * ``29 01`` and ``31 01 00 00`` (the auth / routine probes);
  * exactly ONE ``27 02`` sendKey: its 8 key bytes MUST equal ``seed`` (the step-2 ESC seed) and it MUST be the first
    (``key_attempts == 0``) -- any second attempt or any other key raises. It is sent as ISO-TP multi-frame: the first
    frame ``10 0A 27 02 <4 key bytes>`` and the one consecutive frame ``21 <4 key bytes>``.

  Phase 4 adds the candidate-key sendKey: ``key`` is the RESOLVED key bytes (2/4/8), and a phase-4 ``27 02`` frame is
  admitted ONLY when ``key_attempts == 0`` and its key bytes equal ``key`` exactly. A 2- or 4-byte key is a single
  frame (``04 27 02 K1K2`` / ``06 27 02 K1..K4``); an 8-byte key is the ISO-TP multi-frame shape. The phase-3
  multi-frame path stays reachable (a phase-4 run may also resolve an 8-byte candidate that must equal ``seed``).
  """
  dat = bytes(dat)
  if phase == 5:
    # ---- PHASE 5: the READ-ONLY battery. Single frames only; the fixed flow-control frame is the one exemption (it
    # completes an RX multi-frame answer). No 27 02, no 2E, no multi-frame TX. Request addresses are the ESC (0x7D1)
    # plus the two bounded extra peek addresses, which admit ONLY the 10 03 session frame.
    if bus != ESC_BUS or len(dat) != 8:
      raise SafetyViolation(f"phase5: frame 0x{addr:X} bus {bus} len {len(dat)} refused")
    if dat == FLOW_CONTROL_FRAME:
      return
    if addr != ESC_REQ_ADDR:
      if addr in PHASE5_EXTRA_REQ_ADDRS and dat[:3] == bytes([2, SVC_DIAGNOSTIC_SESSION_CONTROL, ALLOWED_SESSION_SUBFUNC]):
        return
      raise SafetyViolation(f"phase5: frame 0x{addr:X} refused (only 0x7D1, and 0x770/0x7A0 for 10 03): {dat.hex()}")
    if dat[0] >> 4 != 0 or not 1 <= dat[0] <= 7:
      raise SafetyViolation(f"phase5: only ISO-TP single frames may be sent: {dat.hex()}")
    ln, svc = dat[0], dat[1]
    if svc == SVC_READ_DATA_BY_IDENTIFIER and ln == 3:
      did = (dat[2] << 8) | dat[3]
      if did not in PHASE5_DIDS:
        raise SafetyViolation(f"phase5: read of DID 0x{did:04X} refused (only 0xF100 and 0x0103)")
      return
    if svc == SVC_DIAGNOSTIC_SESSION_CONTROL and ln == 2 and dat[2] == ALLOWED_SESSION_SUBFUNC:
      return
    if svc == SVC_SECURITY_ACCESS and ln == 2 and dat[2] in PHASE5_SEC_SUBFUNCS:
      return
    if ln == 1 and svc in PHASE5_SVC_PROBES:
      return
    raise SafetyViolation(f"phase5: frame {dat.hex()} is not an allowlisted read-only battery request")
  if phase == 6:
    # ---- PHASE 6: the security-policy matrix + DID sweep. All frames to the ESC; single frames for everything except
    # the zero-key sendKey's ISO-TP multi-frame. No 2E/3E/29/31. The 27 02 is pinned to the zero key AND to a
    # requestSeed immediately before it: ``p6_seed`` is the ordering sentinel the caller passes right after a 27 01, so
    # a 27 02 with no preceding 27 01 raises HERE. The attempt counter is capped HERE too; the ONE 0x21 consecutive
    # frame must carry the zero-key tail.
    if addr != ESC_REQ_ADDR or bus != ESC_BUS or len(dat) != 8:
      raise SafetyViolation(f"phase6: frame 0x{addr:X} bus {bus} len {len(dat)} refused (only 0x7D1 on bus 1)")
    if dat == FLOW_CONTROL_FRAME:
      return
    # --- the zero-key sendKey's ISO-TP first frame (its byte 1 is a length, not a service) ---
    if dat[0] == 0x10 and dat[1] == 0x0A and dat[2] == SVC_SECURITY_ACCESS and dat[3] == ALLOWED_SEC_SUBFUNC_KEY:
      if not p6_seed:
        raise SafetyViolation("phase6: 27 02 sendKey without the required immediately-preceding 27 01 (zero key pinned)")
      if key_attempts >= PHASE6_MAX_KEY_ATTEMPTS:
        raise SafetyViolation(f"phase6: 27 02 refused: {key_attempts} key attempts already (max {PHASE6_MAX_KEY_ATTEMPTS})")
      if dat[4:8] != PHASE6_KEY[:4]:
        raise SafetyViolation(f"phase6: 27 02 first frame key {dat[4:8].hex()} != zero key {PHASE6_KEY[:4].hex()}")
      return
    if dat[0] == 0x21:
      if dat[1:5] != PHASE6_KEY[4:8]:
        raise SafetyViolation(f"phase6: 27 02 consecutive frame key {dat[1:5].hex()} != zero-key tail {PHASE6_KEY[4:8].hex()}")
      return
    if dat[0] >> 4 != 0 or not 1 <= dat[0] <= 7:
      raise SafetyViolation(f"phase6: only ISO-TP single frames may be sent: {dat.hex()}")
    ln, svc = dat[0], dat[1]
    if svc == SVC_READ_DATA_BY_IDENTIFIER and ln == 3:
      did = (dat[2] << 8) | dat[3]
      if did not in PHASE6_DIDS:
        raise SafetyViolation(f"phase6: read of DID 0x{did:04X} refused (0x0103 + the exact DidSweep set)")
      return
    if svc == SVC_DIAGNOSTIC_SESSION_CONTROL and ln == 2 and dat[2] in PHASE6_SESSION_SUBFUNCS:
      return
    if svc == SVC_SECURITY_ACCESS and ln == 2 and dat[2] == ALLOWED_SEC_SUBFUNC:
      return
    if svc == SVC_READ_DTC_INFORMATION and ln == 3 and dat[2] == PHASE6_DTC_SUBFUNC and dat[3] == PHASE6_DTC_MASK:
      return
    raise SafetyViolation(f"phase6: frame {dat.hex()} is not an allowlisted policy-matrix request")
  if addr not in ALLOWED_REQ_ADDRS or bus != ESC_BUS or len(dat) != 8:
    raise SafetyViolation(f"frame 0x{addr:X} bus {bus} len {len(dat)} refused")
  if dat == FLOW_CONTROL_FRAME:
    return
  # --- phase-3/4 multi-frame sendKey (8-byte key) first: it is NOT a single frame ---
  if phase in (3, 4) and dat[0] == 0x10 and dat[1] == 0x0A and dat[2] == SVC_SECURITY_ACCESS and dat[3] == ALLOWED_SEC_SUBFUNC_KEY:
    if seed is None:
      raise SafetyViolation("27 02 sendKey without the step-2 seed")
    if key_attempts != 0:
      raise SafetyViolation(f"27 02 sendKey refused: already attempted this run ({key_attempts})")
    if dat[4:8] != bytes(seed)[:4]:
      raise SafetyViolation(f"27 02 first frame key {dat[4:8].hex()} != step-2 seed {bytes(seed)[:4].hex()}")
    return
  if phase in (3, 4) and dat[0] == 0x21:
    if seed is None:
      raise SafetyViolation("27 02 consecutive frame without the step-2 seed")
    if key_attempts != 1:
      raise SafetyViolation(f"27 02 consecutive frame refused: no consumed sendKey attempt ({key_attempts})")
    if dat[1:5] != bytes(seed)[4:8]:
      raise SafetyViolation(f"27 02 consecutive frame key {dat[1:5].hex()} != step-2 seed tail {bytes(seed)[4:8].hex()}")
    return
  if dat[0] >> 4 != 0 or not 1 <= dat[0] <= 7:
    raise SafetyViolation(f"only ISO-TP single frames may be sent: {dat.hex()}")
  ln, svc = dat[0], dat[1]
  if svc == SVC_READ_DATA_BY_IDENTIFIER and ln == 3:
    return
  if svc == SVC_TESTER_PRESENT and ln == 2 and dat[2] in (0x00, 0x80):
    return
  if svc == SVC_DIAGNOSTIC_SESSION_CONTROL and ln == 2 and dat[2] == ALLOWED_SESSION_SUBFUNC:
    return
  if svc == SVC_SECURITY_ACCESS and ln == 2 and dat[2] == ALLOWED_SEC_SUBFUNC:
    return
  if phase == 4 and svc == SVC_SECURITY_ACCESS and dat[2] == ALLOWED_SEC_SUBFUNC_KEY:
    # phase-4 sendKey: a 2- or 4-byte candidate key as a single frame. Pinned to the RESOLVED key bytes.
    if key is None:
      raise SafetyViolation("27 02 sendKey without the resolved candidate key")
    if key_attempts != 0:
      raise SafetyViolation(f"27 02 single-frame sendKey refused: already attempted this run ({key_attempts})")
    if ln not in (4, 6) or len(key) != ln - 2:
      raise SafetyViolation(f"27 02 frame len {ln} does not match a {len(key)}-byte candidate key")
    if dat[3:1 + ln] != bytes(key):
      raise SafetyViolation(f"27 02 key {dat[3:1 + ln].hex()} != resolved candidate {bytes(key).hex()}")
    return
  if phase == 3 and svc == SVC_AUTHENTICATION and ln == 2 and dat[2] == ALLOWED_AUTH_SUBFUNC:
    return
  if phase == 3 and svc == SVC_ROUTINE_CONTROL and ln == 4 and dat[2] == ALLOWED_ROUTINE_SUBFUNC \
     and dat[3:5] == ROUTINE_CONTROL_ID.to_bytes(2, "big"):
    return
  if svc == SVC_WRITE_DATA_BY_IDENTIFIER and ln == 7 and dat[2:4] == WRITE_DID.to_bytes(2, "big"):
    if readback is None:
      raise SafetyViolation("0x2E write without the step-1 read-back value")
    if dat[4:8] != bytes(readback):
      raise SafetyViolation(f"0x2E payload {dat[4:8].hex()} != step-1 read-back {bytes(readback).hex()}")
    return
  raise SafetyViolation(f"frame {dat.hex()} is not an allowlisted probe request")


def parse_read_did(resp_hex: str | None, did: int) -> bytes | None:
  """A usable positive 0x22 answer is exactly ``62 <did hi> <did lo>`` + 4 data bytes (the ESC's 0x0103 shape)."""
  if not resp_hex:
    return None
  try:
    body = bytes.fromhex(resp_hex)
  except ValueError:
    return None
  if len(body) == 7 and body[:3] == bytes([0x62, did >> 8, did & 0xFF]):
    return body[3:7]
  return None


def _resolve_algo_key(seed: bytes, algo: str | None) -> bytes:
  """Shared algo resolution for the ``algo``/``algo8``/``algo8p`` modes: ``key_for(seed, algo)``.

  Aborts (recorded) on a missing/failing module, an unknown algo name, or the vendor zero-byte bail (``key_for`` ->
  None) -- identical semantics in all three modes, so a zero-byte seed can never reach the bus unpinned.
  """
  try:
    import openpilot.sunnypilot.fork.esc_probe_seedkey as sk
    resolved = sk.key_for(bytes(seed), algo)          # None = the vendor zero-byte bail, not an error
  except Exception as e:  # missing module, import error, unknown algo, or the algorithm raising -> recorded abort
    raise Abort(f"algo module missing/failed: {e!r}") from e
  if resolved is None:
    raise Abort("algo returned no key for this seed (zero-byte bail)")
  return bytes(resolved)


def resolve_key(seed: bytes, key_mode: str, key_hex: str | None = None, algo: str | None = None) -> bytes:
  """Phase 4 step 5: derive the candidate key bytes from the step-3 seed.

  identity2 = the seed's first 2 bytes; identity4 = first 4; identity8 = the whole seed; algo = the separate
  ``esc_probe_seedkey`` module's ``key_for(seed, algo)`` (``algo`` names a recovered G-scan algorithm, None -> its
  vendor-exact 27100 default); hex = the operator-supplied ``key_hex``. A missing/failing algo module, an unknown algo
  name, a zero-byte seed the vendor path bails on (``key_for`` -> None), an unparseable hex string, or any length other
  than 2/4/8 aborts the run (recorded).

  The 8-byte CONSTRUCTION modes wrap the same resolvers into exactly 8 wire bytes (the car proved 27 02 wants an 8-byte
  key -- 4 bytes drew NRC 0x13, the full seed drew 0x35):
    * algo8  = the algo key repeated to fill 8 (2-byte k -> k*4; 4-byte k -> k*2);
    * algo8w = the algo key's FIRST TWO BYTES repeated to fill 8 (``k[:2] * 4`` = ``[lo,hi]x4``);
    * algo8p = the algo key + zero padding (2-byte -> k + 6 zeros; 4-byte -> k + 4 zeros);
    * repeat8 = the seed's own 2-byte value repeated (seed[:2] * 4) -- same bytes as identity2-as-8B, kept distinct for
      construction clarity;
    * hex8   = exactly 8 bytes from ``key_hex`` (any other length aborts -- stricter than ``hex``).
  """
  if key_mode == "identity2":
    key = bytes(seed)[:2]
  elif key_mode == "identity4":
    key = bytes(seed)[:4]
  elif key_mode == "identity8":
    key = bytes(seed)
  elif key_mode == "algo":
    key = _resolve_algo_key(seed, algo)
  elif key_mode == "algo8":
    k = _resolve_algo_key(seed, algo)
    if len(k) not in (2, 4):
      raise Abort(f"algo8: algo key length {len(k)} is not 2 or 4")
    key = (k * 4)[:8] if len(k) == 2 else k * 2
  elif key_mode == "algo8w":
    k = _resolve_algo_key(seed, algo)
    if len(k) not in (2, 4):
      raise Abort(f"algo8w: algo key length {len(k)} is not 2 or 4")
    key = k[:2] * 4                    # first two key bytes repeated to fill 8: the seed's own [v0,v1]x4 wire shape
  elif key_mode == "algo8p":
    k = _resolve_algo_key(seed, algo)
    if len(k) not in (2, 4):
      raise Abort(f"algo8p: algo key length {len(k)} is not 2 or 4")
    key = k.ljust(8, b"\x00")
  elif key_mode == "repeat8":
    key = bytes(seed)[:2] * 4
  elif key_mode == "hex":
    try:
      key = bytes.fromhex((key_hex or "").strip())
    except ValueError as e:
      raise Abort(f"hex key_mode: bad key_hex {key_hex!r}") from e
  elif key_mode == "hex8":
    try:
      key = bytes.fromhex((key_hex or "").strip())
    except ValueError as e:
      raise Abort(f"hex8 key_mode: bad key_hex {key_hex!r}") from e
    if len(key) != 8:
      raise Abort(f"hex8 key_mode: key_hex must be exactly 8 bytes, got {len(key)}")
  else:
    raise Abort(f"unknown key_mode {key_mode!r}")
  if len(key) not in ALLOWED_KEY_LENGTHS:
    raise Abort(f"resolved key length {len(key)} not in {ALLOWED_KEY_LENGTHS}")
  return key


# The phase-4 sequence (one key attempt, mechanically pinned). Kept here so the demo / tests can reference it verbatim.
KEY_MODES_PHASE4 = KEY_MODES


class EscProbeClient:
  """Minimal UDS client for the probe. ``_tx`` is the ONLY place a frame is handed to can_send.

  Multi-address (phase 3): a request names its own expected response address; ``_rx_frames`` collects frames for ALL
  known response addresses into ``self.rx`` so a response that arrives with an unexpected latency (the signature of a
  gateway answering on the module's behalf) is still timestamped and visible.
  """

  def __init__(self, can_send, can_recv, gate, now: Callable[[], float], deadline: float, phase: int = 1):
    self._can_send = can_send
    self._can_recv = can_recv
    self.gate = gate
    self.now = now
    self.deadline = deadline
    self.phase = phase
    self.tx_log: list[str] = []
    self.silent = 0
    self.key_attempts = 0            # 27 02 sendKey: hard single-attempt counter (guard_frame enforces it)
    self._p6_seed_precedes = False   # phase 6: True for exactly one frame after a 27 01; guards the 27 02 ordering
    self.rx: dict[int, list[tuple[float, bytes]]] = {a: [] for a in ALL_RESP_ADDRS}
    if phase == 5:
      # phase 5 also listens on the two extra peek addresses (0x778/0x7A8) so a response OR silence is timestamped
      for _req, rsp in PHASE5_EXTRA_ADDRS:
        self.rx.setdefault(rsp, [])

  def _tx(self, dat: bytes, readback: bytes | None = None, *, seed: bytes | None = None,
          key: bytes | None = None, p6_seed: bool = False) -> None:
    self.gate.check()
    if self.now() > self.deadline:
      raise Abort("time budget exhausted")
    guard_frame(ESC_REQ_ADDR, dat, ESC_BUS, readback, phase=self.phase, seed=seed, key_attempts=self.key_attempts,
                key=key, p6_seed=p6_seed)
    self._can_send([CanData(ESC_REQ_ADDR, bytes(dat), ESC_BUS)])
    self.tx_log.append(bytes(dat).hex())

  def _tx_at(self, addr: int, dat: bytes) -> None:
    """Phase-3 helper: the single TX site for the non-ESC modules. Same gate + guard_frame + can_send path as _tx."""
    self.gate.check()
    if self.now() > self.deadline:
      raise Abort("time budget exhausted")
    guard_frame(addr, dat, ESC_BUS, phase=self.phase, key_attempts=self.key_attempts)
    self._can_send([CanData(addr, bytes(dat), ESC_BUS)])
    self.tx_log.append(bytes(dat).hex())

  def _rx_frames(self) -> list[bytes]:
    """Collect every known response address's frames (timestamped), so a request can match its own."""
    out = []
    for packet in self._can_recv(wait_for_one=True):
      for msg in packet:
        self.gate.feed(msg)
        if msg.src == ESC_BUS and msg.address in self.rx:
          dat = bytes(msg.dat)
          self.rx[msg.address].append((self.now(), dat))
          out.append(dat)
    self.gate.check()
    return out

  def _rx_for(self, rsp_addr: int) -> list[bytes]:
    """Take the frames received for ``rsp_addr`` and clear them (a second answer for the same addr is a new exchange)."""
    frames = [d for _, d in self.rx.get(rsp_addr, [])]
    self.rx[rsp_addr] = []
    return frames

  def drain(self) -> None:
    self._rx_frames()

  def wait(self, seconds: float) -> None:
    """Phase-4 step 4: honour the vendor ``delaytime`` (~500 ms) before the key. Pumping the bus keeps the state fresh."""
    t_end = self.now() + seconds
    while self.now() < t_end:
      self._rx_frames()

  def request(self, service, subfunc, payload=b"", did=None, readback=None, *, addr=ESC_REQ_ADDR, rsp=None,
              timeout=None, key=None):
    rsp_addr = rsp if rsp is not None else RESP_OF_REQ[addr]
    guard_service(service, subfunc, did, phase=self.phase)
    req = bytes([service]) + (bytes([subfunc]) if subfunc is not None else b"") + bytes(payload)
    frame = build_single_frame(req)
    self.drain()
    if addr == ESC_REQ_ADDR:
      self._tx(frame, readback, key=key)
    else:
      self._tx_at(addr, frame)
    if key is not None:
      self.key_attempts = 1     # phase-4 single-frame sendKey consumed: a second 27 02 can never reach the bus
    t_tx = self.now()
    res: dict = {"req": frame.hex(), "addr": addr, "rsp_addr": rsp_addr, "frames": [], "frames_t_ms": []}
    seen: set[str] = set()
    data = b""
    expect = None
    t_end = t_tx + (timeout if timeout is not None else RESP_TIMEOUT_S)
    while self.now() < t_end:
      took = False
      for f in self._rx_for(rsp_addr):
        took = True
        kind = f[0] >> 4
        if expect is not None and kind == 2:
          if f.hex() not in seen:
            seen.add(f.hex())
            res["frames"].append(f.hex())
            res["frames_t_ms"].append(round((self.now() - t_tx) * 1000, 3))
          data += f[1:8]
          t_end = self.now() + CF_TIMEOUT_S
          if len(data) >= expect:
            self.silent = 0
            return self._finish(res, service, data[:expect])
          continue
        if f.hex() not in seen:
          seen.add(f.hex())
          res["frames"].append(f.hex())
          res["frames_t_ms"].append(round((self.now() - t_tx) * 1000, 3))
        if expect is None and kind == 0:
          body = f[1:1 + (f[0] & 0xF)]
          if len(body) >= 3 and body[0] == 0x7F and body[2] == 0x78:
            t_end = self.now() + PENDING_TIMEOUT_S   # responsePending: keep waiting
            continue
          self.silent = 0
          return self._finish(res, service, body)
        if expect is None and kind == 1:
          expect = ((f[0] & 0xF) << 8) | f[1]
          data = f[2:8]
          self._tx(FLOW_CONTROL_FRAME)
          t_end = self.now() + CF_TIMEOUT_S
      if not took:
        self._rx_frames()                # pump the bus (and the clock); new frames are matched next pass
    res["no_response" if not res["frames"] else "incomplete"] = True
    self.silent = self.silent + 1 if not res["frames"] else 0
    if self.silent >= SILENT_ABORT_N:
      raise Abort(f"module 0x{rsp_addr:X} silent for {self.silent} requests")
    return res

  @staticmethod
  def _finish(res: dict, service: int, body: bytes) -> dict:
    res["resp"] = body.hex()
    if body[:1] == b"\x7f":
      res["nrc"] = body[2] if len(body) > 2 else None
    elif body[:1] == bytes([service + 0x40]):
      res["positive"] = True
    return res

  def read_did(self, did: int) -> dict:
    return self.request(SVC_READ_DATA_BY_IDENTIFIER, None, bytes([did >> 8, did & 0xFF]), did=did)

  def extended_session(self) -> dict:
    return self.request(SVC_DIAGNOSTIC_SESSION_CONTROL, ALLOWED_SESSION_SUBFUNC)

  def request_probe(self, service, subfunc=None, payload=b"", did=None, *, addr=ESC_REQ_ADDR, rsp=None, timeout=None) -> dict:
    """Phase 5: one bare read-only probe frame (1- or 2-byte) to the ESC or to a bounded extra peek address.

    ``request()``'s default response address comes from ``RESP_OF_REQ`` (the ESC's six known modules); the two phase-5
    peek addresses (0x770/0x7A0) are not in that table, so their listen address is supplied explicitly here.
    """
    rsp_addr = rsp if rsp is not None else (PHASE5_EXTRA_RESP_OF_REQ[addr] if addr in PHASE5_EXTRA_RESP_OF_REQ
                                            else RESP_OF_REQ[addr])
    return self.request(service, subfunc, payload, did=did, addr=addr, rsp=rsp_addr, timeout=timeout)

  def request_seed(self) -> dict:
    r = self.request(SVC_SECURITY_ACCESS, ALLOWED_SEC_SUBFUNC, did=None)
    if self.phase == 6:
      # phase 6: a requestSeed was just sent, so a 27 02 may immediately follow it (the zero-key ordering sentinel)
      self._p6_seed_precedes = True
    return r

  def request_seed_at(self, addr: int) -> dict:
    """Phase 3: request the 0x27 seed from any of the six modules, waiting up to SEED_SWEEP_TIMEOUT_S."""
    return self.request(SVC_SECURITY_ACCESS, ALLOWED_SEC_SUBFUNC, did=None, addr=addr, timeout=SEED_SWEEP_TIMEOUT_S)

  def seed_sweep(self) -> list[dict]:
    """Phase 3 step 2: the 6-address 27 01 sweep. Benign (seeds only), per-address timeout, never raises."""
    out = []
    for addr in SEED_SWEEP_ADDRS:
      r = {**self.request_seed_at(addr), "name": f"seed_{addr:X}"}
      out.append(r)
    return out

  def auth_probe(self) -> dict:
    """Phase 3 step 4: 29 01 (Authentication start) to the ESC -- is the UDS auth service live?"""
    return self.request(SVC_AUTHENTICATION, ALLOWED_AUTH_SUBFUNC, did=None)

  def routine_probe(self) -> dict:
    """Phase 3 step 5: 31 01 0000 (RoutineControl start, routine 0x0000) -- session logic answered vs blanket filter?"""
    return self.request(SVC_ROUTINE_CONTROL, ALLOWED_ROUTINE_SUBFUNC, ROUTINE_CONTROL_ID.to_bytes(2, "big"))

  def send_key(self, seed: bytes) -> dict:
    """Phase 3 step 9 (LAST, once): 27 02 sendKey with EXACTLY the step-2 ESC seed, as ISO-TP multi-frame.

    Mechanically pinned: ``seed`` must be the 8 bytes the ESC returned in step 2, and this is the first 27 02 of the
    run (``key_attempts == 0``). The single ``_tx`` site sends the first frame, then waits for the ESC flow-control
    frame (0x30), then sends the ONE consecutive frame. The guard admits only those two frames with that key.
    """
    seed = bytes(seed)
    if len(seed) != 8:
      raise SafetyViolation(f"sendKey seed must be the 8-byte step-2 seed, got {len(seed)}")
    if self.key_attempts != 0:
      raise SafetyViolation("sendKey refused: already attempted this run")
    req = bytes([SVC_SECURITY_ACCESS, ALLOWED_SEC_SUBFUNC_KEY]) + seed   # 10 bytes -> multi-frame
    first = bytes([0x10, len(req)]) + req[:6]                # FF: 10 0A 27 02 <k0..k3> (8-byte CAN frame)
    cf = (bytes([0x21]) + seed[4:8]).ljust(8, b"\x00")    # CF: 21 <k4..k7> + pad (8-byte CAN frame)
    self.drain()
    self.rx[ESC_RSP_ADDR] = []
    t_tx = self.now()
    self._tx(first, seed=seed)                                # guard_frame(seed=seed, key_attempts=0) admits it
    self.key_attempts = 1                                     # consumed: the consecutive frame requires ==1; a 2nd key raises
    res: dict = {"req": first.hex(), "addr": ESC_REQ_ADDR, "rsp_addr": ESC_RSP_ADDR, "frames": [first.hex()],
                 "frames_t_ms": [0.0]}
    seen: set[str] = set(res["frames"])
    cf_sent = False
    data = b""
    expect = None
    t_end = t_tx + CF_TIMEOUT_S
    while self.now() < t_end:
      took = False
      for f in self._rx_for(ESC_RSP_ADDR):
        took = True
        if f.hex() not in seen:
          seen.add(f.hex())
          res["frames"].append(f.hex())
          res["frames_t_ms"].append(round((self.now() - t_tx) * 1000, 3))
        kind = f[0] >> 4
        if kind == 3:                                        # ESC flow control -> send the ONE consecutive frame
          if not cf_sent:
            self._tx(cf, seed=seed)
            cf_sent = True
            seen.add(cf.hex())
            res["frames"].append(cf.hex())
            res["frames_t_ms"].append(round((self.now() - t_tx) * 1000, 3))
            t_end = self.now() + RESP_TIMEOUT_S
          continue
        if kind == 0:
          body = f[1:1 + (f[0] & 0xF)]
          return self._finish(res, SVC_SECURITY_ACCESS, body)
        if kind == 1:
          expect = ((f[0] & 0xF) << 8) | f[1]
          data = f[2:8]
          self._tx(FLOW_CONTROL_FRAME)
          t_end = self.now() + CF_TIMEOUT_S
        elif expect is not None and kind == 2:
          data += f[1:8]
          if len(data) >= expect:
            return self._finish(res, SVC_SECURITY_ACCESS, data[:expect])
      if not took:
        self._rx_frames()
    res["no_response" if not cf_sent else "incomplete"] = True
    return res

  def send_key_candidate(self, key: bytes) -> dict:
    """Phase 4 step 6 (ONCE): 27 02 sendKey with the RESOLVED candidate key (2/4/8 bytes).

    Mechanically pinned exactly like phase 3: ``key`` must be the resolved candidate bytes and this is the first 27 02
    of the run. A 2- or 4-byte key is a single frame; an 8-byte key is the ISO-TP multi-frame shape (first frame +
    the ESC's flow-control frame + one consecutive frame). The guard admits ONLY frames whose key bytes equal ``key``.
    """
    key = bytes(key)
    if len(key) not in ALLOWED_KEY_LENGTHS:
      raise SafetyViolation(f"sendKey key must be 2, 4 or 8 bytes, got {len(key)}")
    if self.key_attempts != 0:
      raise SafetyViolation("sendKey refused: already attempted this run")
    if len(key) in (2, 4):
      return self.request(SVC_SECURITY_ACCESS, ALLOWED_SEC_SUBFUNC_KEY, key, did=None, key=key)
    # 8-byte key: the phase-3 multi-frame machinery (first frame, ESC flow control, one consecutive frame)
    req = bytes([SVC_SECURITY_ACCESS, ALLOWED_SEC_SUBFUNC_KEY]) + key   # 10 bytes -> multi-frame
    first = bytes([0x10, len(req)]) + req[:6]                # FF: 10 0A 27 02 <k0..k3> (8-byte CAN frame)
    cf = (bytes([0x21]) + key[4:8]).ljust(8, b"\x00")        # CF: 21 <k4..k7> + pad (8-byte CAN frame)
    self.drain()
    self.rx[ESC_RSP_ADDR] = []
    t_tx = self.now()
    self._tx(first, seed=key)                                # guard_frame(seed=key, key_attempts=0) admits it
    self.key_attempts = 1                                    # consumed: the consecutive frame requires ==1; a 2nd key raises
    res: dict = {"req": first.hex(), "addr": ESC_REQ_ADDR, "rsp_addr": ESC_RSP_ADDR, "frames": [first.hex()],
                 "frames_t_ms": [0.0]}
    seen: set[str] = set(res["frames"])
    cf_sent = False
    data = b""
    expect = None
    t_end = t_tx + CF_TIMEOUT_S
    while self.now() < t_end:
      took = False
      for f in self._rx_for(ESC_RSP_ADDR):
        took = True
        if f.hex() not in seen:
          seen.add(f.hex())
          res["frames"].append(f.hex())
          res["frames_t_ms"].append(round((self.now() - t_tx) * 1000, 3))
        kind = f[0] >> 4
        if kind == 3:                                        # ESC flow control -> send the ONE consecutive frame
          if not cf_sent:
            self._tx(cf, seed=key)
            cf_sent = True
            seen.add(cf.hex())
            res["frames"].append(cf.hex())
            res["frames_t_ms"].append(round((self.now() - t_tx) * 1000, 3))
            t_end = self.now() + RESP_TIMEOUT_S
          continue
        if kind == 0:
          body = f[1:1 + (f[0] & 0xF)]
          return self._finish(res, SVC_SECURITY_ACCESS, body)
        if kind == 1:
          expect = ((f[0] & 0xF) << 8) | f[1]
          data = f[2:8]
          self._tx(FLOW_CONTROL_FRAME)
          t_end = self.now() + CF_TIMEOUT_S
        elif expect is not None and kind == 2:
          data += f[1:8]
          if len(data) >= expect:
            return self._finish(res, SVC_SECURITY_ACCESS, data[:expect])
      if not took:
        self._rx_frames()
    res["no_response" if not cf_sent else "incomplete"] = True
    return res

  def _tx_phase6_key(self, key: bytes, rsp_addr: int) -> dict:
    """Phase 6: the ONE zero-key ``27 02`` sendKey, as ISO-TP (an 8-byte key: FF ``10 0A 27 02 00 00 00 00`` + one CF).

    This method only consumes the ordering sentinel and forwards it to ``_tx``/``guard_frame``; the ENFORCEMENT of
    "zero key only / immediately after a 27 01 / max ``PHASE6_MAX_KEY_ATTEMPTS`` attempts" lives solely in
    ``guard_frame``. The attempt counter is incremented by the caller (``send_key_phase6``) after ``_tx`` returns.
    """
    p6_seed = self._p6_seed_precedes          # the ordering sentinel: True only if a 27 01 immediately preceded
    self._p6_seed_precedes = False            # consume it; guard_frame is the SOLE enforcer (raises if False)
    key = bytes(key)
    req = bytes([SVC_SECURITY_ACCESS, ALLOWED_SEC_SUBFUNC_KEY]) + key   # 10 bytes -> multi-frame
    first = bytes([0x10, len(req)]) + req[:6]                          # FF: 10 0A 27 02 <k0..k3>
    cf = (bytes([0x21]) + key[4:8]).ljust(8, b"\x00")                  # CF: 21 <k4..k7> + pad
    self.drain()
    self.rx[rsp_addr] = []
    t_tx = self.now()
    self._tx(first, seed=PHASE6_KEY, p6_seed=p6_seed)
    res: dict = {"req": first.hex(), "addr": ESC_REQ_ADDR, "rsp_addr": rsp_addr, "frames": [first.hex()],
                 "frames_t_ms": [0.0]}
    seen: set[str] = set(res["frames"])
    cf_sent = False
    data = b""
    expect = None
    t_end = t_tx + CF_TIMEOUT_S
    while self.now() < t_end:
      took = False
      for f in self._rx_for(rsp_addr):
        took = True
        if f.hex() not in seen:
          seen.add(f.hex())
          res["frames"].append(f.hex())
          res["frames_t_ms"].append(round((self.now() - t_tx) * 1000, 3))
        kind = f[0] >> 4
        if kind == 3:                                                  # ESC flow control -> the ONE consecutive frame
          if not cf_sent:
            self._tx(cf, seed=PHASE6_KEY)
            cf_sent = True
            seen.add(cf.hex())
            res["frames"].append(cf.hex())
            res["frames_t_ms"].append(round((self.now() - t_tx) * 1000, 3))
            t_end = self.now() + RESP_TIMEOUT_S
          continue
        if kind == 0:
          return self._finish(res, SVC_SECURITY_ACCESS, f[1:1 + (f[0] & 0xF)])
        if kind == 1:
          expect = ((f[0] & 0xF) << 8) | f[1]
          data = f[2:8]
          self._tx(FLOW_CONTROL_FRAME)
          t_end = self.now() + CF_TIMEOUT_S
        elif expect is not None and kind == 2:
          data += f[1:8]
          if len(data) >= expect:
            return self._finish(res, SVC_SECURITY_ACCESS, data[:expect])
      if not took:
        self._rx_frames()
    res["no_response" if not cf_sent else "incomplete"] = True
    return res

  def send_key_phase6(self, key: bytes) -> dict:
    """The ONE documented phase-6 public entry to a zero-key ``27 02``. It consumes the ordering sentinel and forwards
    it to ``guard_frame`` (the SOLE enforcer of "zero key only / immediately after a 27 01 / max 4 attempts"), then
    increments the attempt counter ONLY after the frame reached the bus. Tests drive THIS (never ``_tx_phase6_key``
    alone) to prove a 5th attempt is refused and a 27 02 without a preceding 27 01 is refused."""
    if bytes(key) != PHASE6_KEY:
      raise SafetyViolation(f"phase6: send_key_phase6 key must be the zero key {PHASE6_KEY.hex()}")
    res = self._tx_phase6_key(PHASE6_KEY, ESC_RSP_ADDR)
    self.key_attempts += 1            # consume the attempt only once the zero key actually reached the bus
    return res

  def write_did(self, did: int, value: bytes, readback: bytes) -> dict:
    """The ONLY write this module can make. ``value`` is mechanically forced to equal the step-1 read-back."""
    if did != WRITE_DID:
      raise SafetyViolation(f"write to DID {did!r} refused (only 0x0103)")
    if len(value) != 4:
      raise SafetyViolation(f"refusing to write {value.hex()}: 0x0103 is a 4-byte DID")
    if value != readback:
      raise SafetyViolation(f"refusing to write {value.hex()}: it must equal the step-1 read-back {readback.hex()}")
    return self.request(SVC_WRITE_DATA_BY_IDENTIFIER, None, did.to_bytes(2, "big") + value, did=did, readback=readback)


# ---------------------------------------------------------------------------------------------------------------------
# State + result files
# ---------------------------------------------------------------------------------------------------------------------
def _load_state(out_dir: str) -> dict:
  try:
    with open(os.path.join(out_dir, "state.json")) as f:
      return json.load(f)
  except (OSError, ValueError):
    return {}


def _save_state(out_dir: str, state: dict) -> None:
  path = os.path.join(out_dir, "state.json")
  tmp = path + ".tmp"
  with open(tmp, "w") as f:
    json.dump(state, f, indent=1, sort_keys=True)
  os.replace(tmp, path)


def _write_result(out_dir: str, kind: str, doc: dict, wall: float) -> str:
  name = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(wall)) + f"-{kind}.json"
  path = os.path.join(out_dir, name)
  with open(path, "w") as f:
    json.dump(doc, f, indent=1, sort_keys=True)
  files = sorted(p for p in os.listdir(out_dir) if p.endswith(".json") and p != "state.json")
  for old in files[:-KEEP_FILES]:
    try:
      os.remove(os.path.join(out_dir, old))
    except OSError:
      pass
  return path


def plan(state: dict, ignition_key: str) -> bool:
  """Pure: may this ignition be probed? Enabled by the operator, and not already probed in this cycle."""
  if not state.get("probe_enabled"):
    return False
  if state.get("done_ignition") == ignition_key:
    return False
  return True


def _raise_keyboard_interrupt(signum, frame):
  raise KeyboardInterrupt


# ---------------------------------------------------------------------------------------------------------------------
# Phase-3 battery -- ONE ignition, a scripted discriminating sequence (see the module docstring / report "Update 4")
# ---------------------------------------------------------------------------------------------------------------------
def _run_phase3_battery(client: "EscProbeClient", doc: dict) -> None:
  """The exact phase-3 sequence. Raises Abort on a hard stop; records every step in ``doc['steps']``."""

  def step(name: str, addr: int, req: str, res: dict) -> None:
    doc["steps"].append({"name": name, "addr": addr, "req": req,
                         "frames": list(res.get("frames", [])), "frames_t_ms": list(res.get("frames_t_ms", [])),
                         "resp": res.get("resp"), "nrc": res.get("nrc"),
                         "positive": bool(res.get("positive")),
                         "no_response": bool(res.get("no_response")), "timeout": bool(res.get("no_response")),
                         "incomplete": bool(res.get("incomplete"))})

  # ---- 1. read the current 0x0103 value (default session) --------------------------------------------------------
  r1 = client.read_did(DID_VARIANT_CODING)
  doc["read"] = r1
  step("read_esc", ESC_REQ_ADDR, r1["req"], r1)
  current = parse_read_did(r1.get("resp"), DID_VARIANT_CODING)
  doc["current_value"] = current.hex() if current is not None else None
  if current is None:
    raise Abort("battery: no current value")     # no usable read -> the no-op write cannot be formed

  # ---- 2. 27 01 seed sweep across the six modules ---------------------------------------------------------------
  sweep = client.seed_sweep()
  doc["seed_sweep"] = sweep
  for r in sweep:
    step(f"seed_{r['addr']:X}", r["addr"], r["req"], r)
  esc_seed = None
  for r in sweep:
    if r["addr"] == ESC_REQ_ADDR and r.get("positive") and (r.get("resp") or "").startswith("6701"):
      esc_seed = bytes.fromhex(r["resp"])[2:]      # 67 01 + 8 seed bytes
  doc["key"] = None

  # ---- 3. the no-op 0x2E write in the DEFAULT session (idempotent) ----------------------------------------------
  doc["write_attempted"] = True
  r3 = client.write_did(DID_VARIANT_CODING, current, current)
  doc["write_default"] = r3
  doc["write"] = r3
  step("write_default", ESC_REQ_ADDR, r3["req"], r3)

  # ---- 4. 29 01 Authentication start to the ESC -----------------------------------------------------------------
  r4 = client.auth_probe()
  doc["auth_probe"] = r4
  step("auth_probe", ESC_REQ_ADDR, r4["req"], r4)

  # ---- 5. 31 01 0000 RoutineControl start to the ESC -------------------------------------------------------------
  r5 = client.routine_probe()
  doc["routine_probe"] = r5
  step("routine_probe", ESC_REQ_ADDR, r5["req"], r5)

  # ---- 6. re-read 0x0103 (confirm unchanged) ---------------------------------------------------------------------
  r6 = client.read_did(DID_VARIANT_CODING)
  doc["reread"] = r6
  reread = parse_read_did(r6.get("resp"), DID_VARIANT_CODING)
  doc["reread_value"] = reread.hex() if reread is not None else None
  doc["value_changed"] = (reread is not None and reread != current)
  step("reread_esc", ESC_REQ_ADDR, r6["req"], r6)

  # ---- 7. enter the extended session (10 03) ---------------------------------------------------------------------
  r7 = client.extended_session()
  doc["session"] = r7
  doc["session_before_write"] = r7
  step("session", ESC_REQ_ADDR, r7["req"], r7)

  # ---- 8. the SAME no-op 0x2E write in the EXTENDED session ------------------------------------------------------
  r8 = client.write_did(DID_VARIANT_CODING, current, current)
  doc["write_extended"] = r8
  step("write_extended", ESC_REQ_ADDR, r8["req"], r8)

  # ---- 9. THE ONE sendKey attempt (LAST). Identity key == the exact step-2 ESC seed. ----------------------------
  if esc_seed is None:
    doc["key_note"] = "no positive ESC seed in step 2; sendKey not attempted"
    return
  doc["key_attempted"] = True
  doc["key"] = esc_seed.hex()
  r9 = client.send_key(esc_seed)
  doc["key_result"] = r9
  doc["key_frames"] = list(r9.get("frames", []))
  doc["key_positive"] = bool(r9.get("positive"))
  doc["key_nrc"] = r9.get("nrc")
  doc["unlocked"] = bool(r9.get("positive"))
  step("key_attempt", ESC_REQ_ADDR, r9["req"], r9)

  # ---- 10. IF (and only if) step 9 returned 67 02 (unlocked): the no-op write, then re-read ----------------------
  if r9.get("positive"):
    r10a = client.extended_session()
    doc["unlock_session"] = r10a
    step("unlock_session", ESC_REQ_ADDR, r10a["req"], r10a)
    r10b = client.write_did(DID_VARIANT_CODING, current, current)
    doc["write_unlocked"] = r10b
    step("write_unlocked", ESC_REQ_ADDR, r10b["req"], r10b)
    r10c = client.read_did(DID_VARIANT_CODING)
    doc["reread_unlocked"] = r10c
    step("reread_unlocked", ESC_REQ_ADDR, r10c["req"], r10c)
  else:
    doc["key_note"] = "sendKey refused: no further keys, no write after a failed key"


# ---------------------------------------------------------------------------------------------------------------------
# Phase-4 sequence -- ONE extended-session seed -> ONE candidate sendKey -> conditional no-op write (see docstring)
# ---------------------------------------------------------------------------------------------------------------------
def _run_phase4_sequence(client: "EscProbeClient", doc: dict, state: dict, current: bytes) -> None:
  """The exact phase-4 sequence. Raises Abort on a hard stop; records every step in ``doc``.

  Stricter than phase 3: seeds only answer in the extended session, so the extended session (10 03) and the 27 01 seed
  MUST both answer POSITIVELY. Silence or a negative at either stops the run before any key.
  """

  def step(name: str, res: dict) -> None:
    doc["steps"].append({"name": name, "addr": ESC_REQ_ADDR, "req": res.get("req"),
                         "frames": list(res.get("frames", [])), "frames_t_ms": list(res.get("frames_t_ms", [])),
                         "resp": res.get("resp"), "nrc": res.get("nrc"), "positive": bool(res.get("positive")),
                         "no_response": bool(res.get("no_response")), "timeout": bool(res.get("no_response")),
                         "incomplete": bool(res.get("incomplete"))})

  # ---- 1. read the current 0x0103 value (default session; extended retry unchanged) -------------------------------
  step("read_esc", doc["read"])

  # ---- 2. ensure the EXTENDED session (10 03). Positive 50 03 REQUIRED. -----------------------------------------
  if not doc["extended_retry"]:
    sess = client.extended_session()
    doc["session_before_write"] = sess
    step("session", sess)
  if not (doc["session_before_write"] or {}).get("positive"):
    raise Abort("phase4: extended session (10 03) not positive; no key, no write")

  # ---- 3. 27 01 requestSeed IN THE EXTENDED SESSION. Positive 67 01 with >=2 seed bytes REQUIRED. -----------------
  seed_res = client.request_seed()
  doc["seed_request"] = seed_res
  pos_seed = None
  if seed_res.get("positive") and (seed_res.get("resp") or "").startswith("6701"):
    pos_seed = bytes.fromhex(seed_res["resp"])[2:]
  doc["seed"] = pos_seed.hex() if pos_seed is not None else None
  step("seed", seed_res)
  if pos_seed is None or len(pos_seed) < 2:
    raise Abort("phase4: 27 01 seed not positive with >=2 bytes; no key, no write")

  # ---- 4. vendor delaytime: wait ~500 ms before the key -----------------------------------------------------------
  client.wait(0.5)

  # ---- 5. resolve the candidate key bytes from the seed -----------------------------------------------------------
  key_mode = state.get("key_mode") or "identity2"
  doc["key_mode"] = key_mode
  algo = state.get("algo") if key_mode in ("algo", "algo8", "algo8w", "algo8p") else None   # selector (None -> key_for default)
  key = resolve_key(pos_seed, key_mode, state.get("key_hex"), algo)   # Abort (recorded) on bad mode/hex/algo/length/None
  doc["algo"] = algo
  doc["key"] = key.hex()
  doc["key_bytes"] = key.hex()          # the computed candidate hex, recorded BEFORE the 27 02 reaches the bus
  doc["key_attempted"] = True

  # ---- 6. send 27 02 ONCE, with EXACTLY those bytes ---------------------------------------------------------------
  r6 = client.send_key_candidate(key)
  doc["key_result"] = r6
  doc["key_frames"] = list(r6.get("frames", []))
  doc["key_positive"] = bool(r6.get("positive"))
  doc["key_nrc"] = r6.get("nrc")
  doc["unlocked"] = bool(r6.get("positive"))
  step("key_attempt", r6)

  # ---- 7. on 67 02 (unlocked): the no-op write + re-read. Any 7F 27 xx / silence: STOP. ---------------------------
  if not r6.get("positive"):
    doc["key_note"] = "sendKey refused/silent: no further keys, no write"
    return
  r7 = client.write_did(DID_VARIANT_CODING, current, current)
  doc["write_unlocked"] = r7
  step("write_unlocked", r7)
  r8 = client.read_did(DID_VARIANT_CODING)
  doc["reread_unlocked"] = r8
  reread = parse_read_did(r8.get("resp"), DID_VARIANT_CODING)
  doc["reread_unlocked_value"] = reread.hex() if reread is not None else None
  step("reread_unlocked", r8)


# ---------------------------------------------------------------------------------------------------------------------
# Phase-5 battery -- ONE ignition, a FIXED read-only frame list (see the module docstring / report "Update 8").
# ---------------------------------------------------------------------------------------------------------------------
def _seed_hex(resp: str | None) -> str | None:
  """The seed bytes of a positive 67 01 answer (everything after the first two response bytes), else None."""
  if not resp or not resp.startswith("6701"):
    return None
  return resp[4:]


def _p6_r(res: dict | None) -> dict | None:
  """Phase 6: a compact R1..R4 record (nrc + resp + positive) for the summary/cloudlog, else None."""
  if not res:
    return None
  return {"nrc": res.get("nrc"), "resp": res.get("resp"), "positive": bool(res.get("positive")),
          "no_response": bool(res.get("no_response"))}


def _run_phase5_battery(client: "EscProbeClient", doc: dict) -> None:
  """The exact phase-5 sequence. Read-only: records every frame's req/resp/nrc/timeout/ms; never aborts on a refusal.

  The frame list is FIXED (the guards enforce it): fp_canary -> read_esc -> session -> seed1 -> seed2 -> ten 2-byte
  sub-probes -> seven bare service probes -> seed3 -> read_esc_end -> the two extra-address `10 03` peeks. No keys, no
  writes (there is no 27 02 and no 2E anywhere in this phase), and NO multi-frame TX.
  """

  def step(name: str, res: dict) -> None:
    doc["steps"].append({"name": name, "addr": res.get("addr"), "resp_addr": res.get("rsp_addr"),
                         "req": res.get("req"), "frames": list(res.get("frames", [])),
                         "frames_t_ms": list(res.get("frames_t_ms", [])), "resp": res.get("resp"),
                         "nrc": res.get("nrc"), "positive": bool(res.get("positive")),
                         "no_response": bool(res.get("no_response")), "timeout": bool(res.get("no_response")),
                         "incomplete": bool(res.get("incomplete"))})

  # ---- 1. fp_canary: a functional-style read of 0xF100 (the ESC's functional-read canary) ---------------------------
  r_canary = client.request_probe(SVC_READ_DATA_BY_IDENTIFIER, None,
                                  bytes([PHASE5_DID_FP_CANARY >> 8, PHASE5_DID_FP_CANARY & 0xFF]),
                                  did=PHASE5_DID_FP_CANARY)
  doc["fp_canary"] = r_canary
  doc["fp_canary_hex"] = r_canary.get("resp")
  step("fp_canary", r_canary)

  # ---- 2. read_esc: the current 0x0103 variant-coding value (start) ------------------------------------------------
  r_read = client.read_did(DID_VARIANT_CODING)
  doc["read"] = r_read
  value_start = parse_read_did(r_read.get("resp"), DID_VARIANT_CODING)
  doc["value_start"] = value_start.hex() if value_start is not None else None
  doc["current_value"] = doc["value_start"]
  step("read_esc", r_read)

  # ---- 3. session: 10 03 extended (the vendor step; any answer continues) ------------------------------------------
  r_sess = client.extended_session()
  doc["session"] = r_sess
  doc["session_before_write"] = r_sess
  step("session", r_sess)

  # ---- 4/5. seed1, seed2: two 27 01 requestSeed samples (is the seed stable across two asks?) ----------------------
  r_seed1 = client.request_seed()
  doc["seed1"] = r_seed1
  step("seed1", r_seed1)
  r_seed2 = client.request_seed()
  doc["seed2"] = r_seed2
  step("seed2", r_seed2)

  # ---- 6. the ten 2-byte sub-probes 27 03/05/07/09/0B/0D/0F/11/41/61 ----------------------------------------------
  for sub in PHASE5_SEC_SUB_PROBES:
    r = client.request_probe(SVC_SECURITY_ACCESS, sub)
    doc["sub_probes"].append({"sub": sub, **r})
    step(f"sub_{sub:02X}", r)

  # ---- 7. the seven bare 1-byte service probes 23/29/31/34/35/36/37 -----------------------------------------------
  for svc in PHASE5_SVC_PROBES:
    r = client.request_probe(svc)
    doc["svc_probes"].append({"svc": svc, **r})
    step(f"svc_{svc:02X}", r)

  # ---- 8. seed3: a third 27 01 sample -----------------------------------------------------------------------------
  r_seed3 = client.request_seed()
  doc["seed3"] = r_seed3
  step("seed3", r_seed3)

  # ---- 9. read_esc_end: re-read 0x0103 (confirm unchanged) --------------------------------------------------------
  r_end = client.read_did(DID_VARIANT_CODING)
  doc["read_end"] = r_end
  value_end = parse_read_did(r_end.get("resp"), DID_VARIANT_CODING)
  doc["value_end"] = value_end.hex() if value_end is not None else None
  doc["reread"] = r_end
  doc["reread_value"] = doc["value_end"]
  doc["value_changed"] = (value_start is not None and value_end is not None and value_end != value_start)
  step("read_esc_end", r_end)

  # ---- 10. extra-address probes: a single 10 03 to req 0x770 (listen 0x778) and req 0x7A0 (listen 0x7A8) -----------
  for req, rsp in PHASE5_EXTRA_ADDRS:
    r = client.request_probe(SVC_DIAGNOSTIC_SESSION_CONTROL, ALLOWED_SESSION_SUBFUNC, addr=req, rsp=rsp, timeout=RESP_TIMEOUT_S)
    doc["extra_probes"].append({"req_addr": req, "rsp_addr": rsp, **r})
    step(f"extra_{req:03X}", r)

  # ---- summary fields ---------------------------------------------------------------------------------------------
  s1, s2, s3 = (_seed_hex(r_seed1.get("resp")), _seed_hex(r_seed2.get("resp")), _seed_hex(r_seed3.get("resp")))
  doc["seed1_hex"], doc["seed2_hex"], doc["seed3_hex"] = s1, s2, s3
  doc["seed_stable_12"] = (s1 is not None and s1 == s2)
  doc["seed_stable_all"] = (s1 is not None and s1 == s2 == s3)
  doc["key_attempted"] = False            # phase 5 never attempts a key
  doc["write_attempted"] = False          # phase 5 never writes


# ---------------------------------------------------------------------------------------------------------------------
# Phase-6 battery -- ONE ignition, the security-policy matrix + DID sweep (see the module docstring / report "Update 9").
# ---------------------------------------------------------------------------------------------------------------------
def _run_phase6_battery(client: "EscProbeClient", doc: dict) -> None:
  """The exact phase-6 sequence. Records every frame in ``doc['steps']``; NEVER aborts on a refusal (except the
  step-1 0x0103 read gate). The zero-key ``27 02`` is mechanically pinned (phase 6 only / zero key only / immediately
  after a ``27 01`` / at most ``PHASE6_MAX_KEY_ATTEMPTS`` total, the ONE 4th being the cycle-reset step)."""
  key_attempts = 0        # this local mirror + client.key_attempts are the enforced cap; the ONE 4th attempt is the reset

  def step(name: str, res: dict) -> None:
    doc["steps"].append({"name": name, "addr": res.get("addr", ESC_REQ_ADDR),
                         "resp_addr": res.get("rsp_addr", ESC_RSP_ADDR), "req": res.get("req"),
                         "frames": list(res.get("frames", [])), "frames_t_ms": list(res.get("frames_t_ms", [])),
                         "resp": res.get("resp"), "nrc": res.get("nrc"), "positive": bool(res.get("positive")),
                         "no_response": bool(res.get("no_response")), "timeout": bool(res.get("no_response")),
                         "incomplete": bool(res.get("incomplete"))})

  def record_seed(tag: str, res: dict) -> dict:
    """Record a 27 01 sample under ``tag`` and flag that a requestSeed now immediately precedes a possible 27 02."""
    doc[tag] = res
    client._p6_seed_precedes = True
    step(tag, res)
    return res

  def record_key(tag: str, res: dict) -> dict:
    doc[tag] = res
    step(tag, res)
    return res

  def seed_hex_of(res: dict) -> str | None:
    return _seed_hex(res.get("resp"))

  def nrc_in_lockout(res: dict) -> bool:
    return res.get("nrc") in PHASE6_LOCKOUT_NRCS

  # ---- 1. read 0x0103 -> value_start (the value_start gate: no usable read -> abort, nothing else runs) -------------
  r_read = client.read_did(DID_VARIANT_CODING)
  doc["read"] = r_read
  value_start = parse_read_did(r_read.get("resp"), DID_VARIANT_CODING)
  doc["value_start"] = value_start.hex() if value_start is not None else None
  doc["current_value"] = doc["value_start"]
  step("read_esc", r_read)
  if value_start is None:
    raise Abort("phase6: step-1 read of 0x0103 refused/missing; nothing else runs")

  # ---- 2. 10 03 extended session (record; if refused/silent skip the key part, still run the DID sweep) ------------
  r_sess = client.extended_session()
  doc["session_before_write"] = r_sess
  step("session", r_sess)
  extended_ok = bool(r_sess.get("positive"))

  # ---- 3/4. two 27 01 samples -> S1, S2 (seed stability) ----------------------------------------------------------
  s1 = record_seed("S1", client.request_seed())
  s2 = record_seed("S2", client.request_seed())
  seed_stable_pre = (seed_hex_of(s1) is not None and seed_hex_of(s1) == seed_hex_of(s2))
  doc["seed_stable_pre"] = seed_stable_pre

  lockout_seen = False
  cycle_reset = False
  key_attempted = False

  # ---- 5. 27 02 zero key -> R1; a 0x36/0x37 NRC marks lockout and skips straight to the cycle-reset (step 8) ------
  if extended_ok and client.key_attempts < PHASE6_MAX_KEY_ATTEMPTS:
    r1 = record_key("R1", client.send_key_phase6(PHASE6_KEY))
    key_attempts += 1
    key_attempted = True
    if nrc_in_lockout(r1):
      lockout_seen = True
      doc["lockout_seen"] = True

  # ---- 6/7. adaptive: 27 01 -> S3 -> R2, then 27 01 -> S4 -> R3 (only while no lockout; <= 3 pre-cycle attempts) ---
  if extended_ok and not lockout_seen:
    if client.key_attempts < PHASE6_MAX_KEY_ATTEMPTS:
      s3 = record_seed("S3", client.request_seed())
      doc["seed_after_fail"] = None
      s3_hex = seed_hex_of(s3)
      if s3_hex is not None:
        if s3_hex == seed_hex_of(s2) or s3_hex == seed_hex_of(s1):
          doc["seed_after_fail"] = s3_hex
        else:
          doc["seed_after_fail"] = "new:" + s3_hex
      record_key("R2", client.send_key_phase6(PHASE6_KEY))
      key_attempts += 1
    if client.key_attempts < PHASE6_MAX_KEY_ATTEMPTS:
      record_seed("S4", client.request_seed())
      record_key("R3", client.send_key_phase6(PHASE6_KEY))
      key_attempts += 1

  # ---- 8. ALWAYS: the cycle-reset test -- 10 01; 10 03; 27 01 -> S5; 27 02 zero key -> R4 (the ONLY 4th key, ever) -
  if client.key_attempts < PHASE6_MAX_KEY_ATTEMPTS:
    doc["reset_session_default"] = client.request(SVC_DIAGNOSTIC_SESSION_CONTROL, SESSION_SUBFUNC_DEFAULT)
    step("reset_session_default", doc["reset_session_default"])
    doc["reset_session_extended"] = client.extended_session()
    step("reset_session_extended", doc["reset_session_extended"])
    record_seed("S5", client.request_seed())
    r4 = record_key("R4", client.send_key_phase6(PHASE6_KEY))
    key_attempts += 1
    if not nrc_in_lockout(r4):
      cycle_reset = True
  doc["cycle_reset"] = cycle_reset
  doc["lockout_seen"] = lockout_seen

  # ---- 9. DID sweep (the extended session; each recorded; refusals are fine) ---------------------------------------
  for did in PHASE6_SWEEP_DIDS:
    tag = f"did_{did:04X}"
    r = client.read_did(did)
    doc["dids"][f"{did:04X}"] = {"resp": r.get("resp"), "nrc": r.get("nrc"), "positive": bool(r.get("positive")),
                                 "no_response": bool(r.get("no_response"))}
    step(tag, r)

  # ---- 10. 10 02 programming-session probe -> record; if positive, 27 01 -> S6 (NO key in the programming session) -
  r_prog = client.request(SVC_DIAGNOSTIC_SESSION_CONTROL, SESSION_SUBFUNC_PROGRAMMING)
  doc["prog_session_1002"] = r_prog
  step("prog_session_1002", r_prog)
  if r_prog.get("positive"):
    record_seed("S6", client.request_seed())

  # ---- 11. 19 02 A5 (security DTC visible?) ----------------------------------------------------------------------
  r_dtc = client.request(SVC_READ_DTC_INFORMATION, PHASE6_DTC_SUBFUNC, bytes([PHASE6_DTC_MASK]))
  doc["dtc_19_02_a5"] = r_dtc
  step("dtc_19_02_a5", r_dtc)

  # ---- 12. clean leave: 10 01, then 22 0103 -> value_end -----------------------------------------------------------
  r_leave = client.request(SVC_DIAGNOSTIC_SESSION_CONTROL, SESSION_SUBFUNC_DEFAULT)
  doc["leave_session"] = r_leave
  step("leave_session", r_leave)
  r_end = client.read_did(DID_VARIANT_CODING)
  doc["read_end"] = r_end
  value_end = parse_read_did(r_end.get("resp"), DID_VARIANT_CODING)
  doc["value_end"] = value_end.hex() if value_end is not None else None
  doc["reread"] = r_end
  doc["reread_value"] = doc["value_end"]
  doc["value_changed"] = (value_start is not None and value_end is not None and value_end != value_start)
  step("read_esc_end", r_end)

  # ---- summary fields --------------------------------------------------------------------------------------------
  doc["key_attempted"] = key_attempted or client.key_attempts > 0
  doc["write_attempted"] = False          # phase 6 NEVER writes (no 2E anywhere)
  doc["unlocked"] = bool(doc.get("R4", {}) and doc["R4"].get("positive")) if doc.get("R4") else False


def _run_phase4_read(client: "EscProbeClient", doc: dict) -> bytes:
  """Phase-4 step 1: read 0x0103 (extended-retry rules unchanged). No usable read -> Abort; no key, no write."""
  r1 = client.read_did(DID_VARIANT_CODING)
  doc["read"] = r1
  current = parse_read_did(r1.get("resp"), DID_VARIANT_CODING)
  if current is None and r1.get("nrc") in (0x31, 0x7F):
    ext = client.extended_session()
    doc["extended_session"] = ext
    if ext.get("positive"):
      doc["extended_retry"] = True
      r1 = client.read_did(DID_VARIANT_CODING)
      doc["read"] = r1
      current = parse_read_did(r1.get("resp"), DID_VARIANT_CODING)
  doc["current_value"] = current.hex() if current is not None else None
  if current is None:
    raise Abort("phase4: step-1 read of 0x0103 refused/missing; refusing to write")
  return current


# ---------------------------------------------------------------------------------------------------------------------
# Core (dependency-injected, so the tests drive the real code path)
# ---------------------------------------------------------------------------------------------------------------------
def run(can_send, can_recv, set_obd_multiplexing, *, fingerprint: str, car_fw=None, ignition_key: str | None,
        ignition_on: bool, out_dir: str = OUT_DIR, now: Callable[[], float] = time.monotonic,
        wall: Callable[[], float] = time.time,  # noqa: TID251 - wall clock wanted (file names)
        verify_mux_off: Callable[[], bool] | None = None,
        log_event: Callable[..., None] | None = None) -> dict:
  """Core probe. Returns a summary dict; never raises into card. All errors are recorded in the result JSON."""
  summary: dict = {"ran": False}
  if fingerprint not in TARGET_FINGERPRINTS:
    return {**summary, "skip": "not the target car"}
  if os.path.exists(os.path.join(out_dir, "DISABLE")):
    return {**summary, "skip": "disabled"}
  if not ignition_on:
    return {**summary, "skip": "ignition off"}
  if not ignition_key:
    return {**summary, "skip": "ignition cycle unknown"}
  os.makedirs(out_dir, exist_ok=True)
  state = _load_state(out_dir)
  if not state.get("probe_enabled"):
    return {**summary, "skip": "not enabled"}
  if not plan(state, ignition_key):
    return {**summary, "skip": "already done (ignition)"}

  # Phase selector: 1 (default) = phase 1 (pre-write seed), 2 = vendor-exact (no pre-write seed, post-write seed
  # sample), 3 = the discriminating battery, 4 = sendKey (extended seed -> ONE candidate key -> conditional no-op
  # write). Anything else is inert: nothing is sent and the ignition is not consumed.
  try:
    phase = int(state.get("phase", 1) or 1)
  except (TypeError, ValueError):
    phase = 0
  key_mode = state.get("key_mode")
  if key_mode is None:
    key_mode = "identity2"
  if phase == 4 and key_mode not in KEY_MODES:
    return {**summary, "skip": f"unknown key_mode {key_mode!r}"}
  if phase not in (1, 2, 3, 4, 5, 6):
    return {**summary, "skip": "unknown phase"}

  gate = VehicleGate(now)
  t0 = now()
  budget = (RUN_BUDGET_S_PHASE6 if phase == 6 else
            RUN_BUDGET_S_PHASE5 if phase == 5 else
            (RUN_BUDGET_S_PHASE3 if phase in (3, 4) else RUN_BUDGET_S))
  client = EscProbeClient(can_send, can_recv, gate, now, t0 + budget, phase=phase)
  doc: dict = {"ignition_key": ignition_key, "fingerprint": fingerprint, "car_fw_abs": None, "phase": phase,
              "read": None, "extended_session": None, "extended_retry": False, "current_value": None,
              "session_before_write": None,
              "seed_request": None, "seed": None, "already_unlocked": None, "seed_post": None,
              "write": None, "write_attempted": False, "reread": None, "reread_value": None,
              "value_changed": None, "tx": [], "duration_s": None, "aborted": None, "error": None,
              "mux_restored": None, "gate_speeds_moving": None, "current": None,
              # phase-3 battery only (kept None/empty for phases 1/2, so those results are unchanged)
              "steps": [], "seed_sweep": None, "write_default": None, "auth_probe": None, "routine_probe": None,
              "session": None, "write_extended": None, "key_attempted": False, "key": None, "unlocked": None,
              "unlock_session": None, "write_unlocked": None, "reread_unlocked": None, "key_note": None,
              # phase-4 only
              "key_mode": None, "key_result": None, "key_frames": None, "key_positive": None, "key_nrc": None,
              "algo": None, "key_bytes": None, "reread_unlocked_value": None,
              # phase-5 only (kept None/empty for phases 1-4, so those results are unchanged)
              "fp_canary": None, "fp_canary_hex": None, "value_start": None, "value_end": None,
              "seed1": None, "seed2": None, "seed3": None, "seed1_hex": None, "seed2_hex": None, "seed3_hex": None,
              "seed_stable_12": None, "seed_stable_all": None, "sub_probes": [], "svc_probes": [],
              "extra_probes": [], "read_end": None,
              # phase-6 only (kept None/empty for phases 1-5, so those results are unchanged)
              "S1": None, "S2": None, "S3": None, "S4": None, "S5": None, "S6": None,
              "R1": None, "R2": None, "R3": None, "R4": None, "seed_stable_pre": None,
              "seed_after_fail": None, "lockout_seen": None, "cycle_reset": None, "dids": {},
              "prog_session_1002": None, "dtc_19_02_a5": None, "reset_session_default": None,
              "reset_session_extended": None, "leave_session": None}
  for fw in car_fw or []:
    if str(getattr(fw, "ecu", "")) == "abs":
      doc["car_fw_abs"] = bytes(fw.fwVersion).hex()

  # The pre-check and the probe body share one try/finally: any failure (including a CAN receive error) is recorded
  # in the result JSON, and the multiplexer is only ever turned off if it was turned on.
  precheck_failed = False
  mux_on = False
  old_sigterm = None
  current = None            # phase 3 sets this inside the battery; phases 1/2 inside their branch -- sentinel for except
  summary["ran"] = True
  try:
    try:
      old_sigterm = signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    except ValueError:
      old_sigterm = None  # not the main thread: finally still restores the multiplexer

    # Pre-check from live frames with the SAME definition the per-frame checks use (esc_diag.VehicleGate). Wait a
    # moment for the state to be KNOWN, then require parked + standstill; a violation does NOT consume the ignition.
    known_by = now() + STATE_KNOWN_WAIT_S
    while True:
      v = gate.violation()
      if "stale" in v and now() < known_by:
        for packet in can_recv(wait_for_one=True):
          for msg in packet:
            gate.feed(msg)
        continue
      break
    pre = gate.violation()
    doc["gate_speeds_moving"] = wheels_moving(gate.speeds) if gate.speeds is not None else None
    if pre:
      precheck_failed = True
      doc["aborted"] = "precheck: " + pre
      raise Abort(doc["aborted"])

    # Pre-check passed: mark the ignition BEFORE touching the multiplexer or sending anything, so a crash mid-probe
    # can never cause a second probe in this cycle.
    state["done_ignition"] = ignition_key
    _save_state(out_dir, state)

    mux_on = True
    set_obd_multiplexing(True)
    client.drain()

    if phase == 3:
      # ---- PHASE 3: the single-ignition discriminating battery (exact scripted order) -----------------------------
      _run_phase3_battery(client, doc)
    elif phase == 4:
      # ---- PHASE 4: read -> extended seed (REQUIRED positive) -> ONE candidate sendKey -> conditional no-op write --
      current = _run_phase4_read(client, doc)
      doc["current"] = current.hex()
      _run_phase4_sequence(client, doc, state, current)
    elif phase == 5:
      # ---- PHASE 5: the READ-ONLY capability battery (fixed frame list; no keys, no writes, no multi-frame TX) ----
      _run_phase5_battery(client, doc)
    elif phase == 6:
      # ---- PHASE 6: the security-policy matrix + DID sweep (zero key ONLY; pinned 27 02 ordering; never writes) ----
      _run_phase6_battery(client, doc)
    else:
      # ---- step 1: read the CURRENT 0x0103 value (default session; extended retry only if refused) -------------
      r1 = client.read_did(DID_VARIANT_CODING)
      doc["read"] = r1
      current = parse_read_did(r1.get("resp"), DID_VARIANT_CODING)
      if current is None and r1.get("nrc") in (0x31, 0x7F):
        ext = client.extended_session()
        doc["extended_session"] = ext
        if ext.get("positive"):
          doc["extended_retry"] = True
          r1 = client.read_did(DID_VARIANT_CODING)
          doc["read"] = r1
          current = parse_read_did(r1.get("resp"), DID_VARIANT_CODING)
      doc["current_value"] = current.hex() if current is not None else None
      if current is None:
        # No usable read -> nothing is written. (The no-op write is only meaningful against the value we just read.)
        raise Abort("step-1 read of 0x0103 refused/missing; refusing to write")

      # ---- step 1.5: enter extended session (10 03) before the write --------------------------------------------
      # The vendor VariantCodingTable for this ECU writes as READ 22 0103 -> 10 03 -> 2E 0103. The read above ran in the
      # default session; if the retry path already entered extended (extended_retry), we are already there -- do not send
      # 10 03 twice. A 10 03 that gets ANY answer (positive 50 03 or negative 7F 10 ..) continues; true silence (no frame
      # at all) means the ESC will not hold a write session, so abort before the write.
      if not doc["extended_retry"]:
        sess = client.extended_session()
        doc["session_before_write"] = sess
        if not sess.get("frames"):
          raise Abort("10 03 session got no response; not attempting the write")

      # ---- step 2: request the 0x27 seed ONLY (never the key) -- PHASE 1 ONLY -----------------------------------
      # Phase 1's contrast is "refused seed + refused/accepted write". Phase 2 deliberately sends NO seed before the
      # write (the exact vendor order); it samples the seed once after the write instead (step 5).
      if phase == 1:
        r2 = client.request_seed()
        doc["seed_request"] = r2
        seed_resp = bytes.fromhex(r2.get("resp", "")) if r2.get("resp") else None
        doc["seed"] = seed_resp.hex() if seed_resp is not None else None
        if r2.get("positive") and seed_resp is not None and seed_resp[:2] == b"\x67\x01":
          doc["already_unlocked"] = seed_resp[2:] == b"\x00\x00\x00\x00"
        if seed_resp is None:
          # Silence (no frame at all): the ESC is not in a state to talk. Do NOT attempt the write.
          raise Abort("0x27 seed request got no response; not attempting the write")
      else:
        # Phase 2: no pre-write seed -- the write is the vendor-exact sequence with nothing in front of it.
        doc["seed_request"] = None

      # ---- step 3: the no-op write -- 0x2E 0x0103 + the exact bytes read in step 1 -------------------------------
      doc["write_attempted"] = True
      r3 = client.write_did(DID_VARIANT_CODING, current, current)
      doc["write"] = r3

      # ---- step 4: re-read and record whether the value changed (it must not) -----------------------------------
      r4 = client.read_did(DID_VARIANT_CODING)
      doc["reread"] = r4
      reread = parse_read_did(r4.get("resp"), DID_VARIANT_CODING)
      doc["reread_value"] = reread.hex() if reread is not None else None
      doc["value_changed"] = (reread is not None and reread != current)

      # ---- step 5 (PHASE 2 ONLY): ONE post-write 0x27 01 sample, after the write actually happened -------------
      # Purely a data point (is the seed static across runs?). The write already happened, so silence or a refusal is
      # recorded and never aborts. 27 02 (sendKey) is still never sent.
      if phase == 2:
        r5 = client.request_seed()
        doc["seed_post"] = r5
  except Abort as e:
    doc["aborted"] = str(e)
  except SafetyViolation as e:
    doc["error"] = f"SafetyViolation: {e}"
  except Exception as e:  # never let this take card down
    doc["error"] = repr(e)
  finally:
    if mux_on:
      try:
        set_obd_multiplexing(False)
        doc["mux_restored"] = verify_mux_off() if verify_mux_off is not None else True
      except BaseException as e:
        doc["mux_restored"] = False
        doc["error"] = (doc["error"] or "") + f" | mux restore failed: {e!r}"
    if old_sigterm is not None:
      signal.signal(signal.SIGTERM, old_sigterm)

  doc["tx"] = client.tx_log
  doc["duration_s"] = round(now() - t0, 3)
  doc["final_gate"] = gate.violation() or "parked+stationary"
  if precheck_failed:
    summary.update(ran=False, skip=doc["aborted"])
  path = _write_result(out_dir, "result", doc, wall())
  summary.update(path=path, current_value=doc["current_value"], session_before_write=doc["session_before_write"],
                 session_before_write_positive=bool((doc["session_before_write"] or {}).get("positive")),
                 phase=doc["phase"], seed=doc["seed"],
                 seed_positive=bool((doc["seed_request"] or {}).get("positive")),
                 seed_post=doc["seed_post"], seed_post_positive=bool((doc["seed_post"] or {}).get("positive")),
                 write_attempted=doc["write_attempted"], write_positive=bool((doc["write"] or {}).get("positive")),
                 write_nrc=(doc["write"] or {}).get("nrc"), reread_value=doc["reread_value"],
                 value_changed=doc["value_changed"], aborted=doc["aborted"], error=doc["error"],
                 tx=len(client.tx_log), mux_restored=doc["mux_restored"], duration_s=doc["duration_s"],
                 steps=len(doc["steps"]), write_default_nrc=(doc["write_default"] or {}).get("nrc"),
                 write_extended_nrc=(doc["write_extended"] or {}).get("nrc"),
                 auth_nrc=(doc["auth_probe"] or {}).get("nrc"), routine_nrc=(doc["routine_probe"] or {}).get("nrc"),
                 seed_sweep=[{"addr": r["addr"], "resp": r.get("resp"), "nrc": r.get("nrc"),
                              "positive": bool(r.get("positive")), "no_response": bool(r.get("no_response"))}
                             for r in (doc["seed_sweep"] or [])],
                 key_attempted=doc["key_attempted"], key=doc["key"], unlocked=doc["unlocked"],
                 key_nrc=doc.get("key_nrc"), key_note=doc["key_note"], key_mode=doc.get("key_mode"),
                 algo=doc.get("algo"), key_bytes=doc.get("key_bytes"),
                 write_unlocked_nrc=(doc["write_unlocked"] or {}).get("nrc"),
                 write_unlocked_positive=bool((doc["write_unlocked"] or {}).get("positive")),
                 reread_unlocked_value=doc.get("reread_unlocked_value"),
                 # phase-5 (read-only battery)
                 fp_canary_hex=doc.get("fp_canary_hex"), value_start=doc.get("value_start"), value_end=doc.get("value_end"),
                 seed1=doc.get("seed1_hex"), seed2=doc.get("seed2_hex"), seed3=doc.get("seed3_hex"),
                 seed_stable_12=doc.get("seed_stable_12"), seed_stable_all=doc.get("seed_stable_all"),
                 sub_probes=[{"sub": r["sub"], "resp": r.get("resp"), "nrc": r.get("nrc"),
                              "no_response": bool(r.get("no_response"))} for r in (doc.get("sub_probes") or [])],
                 svc_probes=[{"svc": r["svc"], "resp": r.get("resp"), "nrc": r.get("nrc"),
                              "no_response": bool(r.get("no_response"))} for r in (doc.get("svc_probes") or [])],
                 extra_probes=[{"req_addr": r["req_addr"], "rsp_addr": r["rsp_addr"], "resp": r.get("resp"),
                                "nrc": r.get("nrc"), "no_response": bool(r.get("no_response"))}
                               for r in (doc.get("extra_probes") or [])],
                 read_end_value=(doc.get("read_end") or {}).get("resp"),
                 # phase-6 (security-policy matrix + DID sweep)
                 S1=_seed_hex((doc.get("S1") or {}).get("resp")), S2=_seed_hex((doc.get("S2") or {}).get("resp")),
                 S3=_seed_hex((doc.get("S3") or {}).get("resp")), S4=_seed_hex((doc.get("S4") or {}).get("resp")),
                 S5=_seed_hex((doc.get("S5") or {}).get("resp")), S6=_seed_hex((doc.get("S6") or {}).get("resp")),
                 R1=_p6_r(doc.get("R1")), R2=_p6_r(doc.get("R2")), R3=_p6_r(doc.get("R3")), R4=_p6_r(doc.get("R4")),
                 seed_stable_pre=doc.get("seed_stable_pre"), seed_after_fail=doc.get("seed_after_fail"),
                 lockout_seen=doc.get("lockout_seen"), cycle_reset=doc.get("cycle_reset"),
                 dids={k: (v.get("resp") if v.get("resp") else (None if not v.get("no_response") else "silent"))
                       for k, v in (doc.get("dids") or {}).items()},
                 dids_nrc={k: v.get("nrc") for k, v in (doc.get("dids") or {}).items()},
                 prog_session_1002=(doc.get("prog_session_1002") or {}).get("resp"),
                 prog_session_1002_positive=bool((doc.get("prog_session_1002") or {}).get("positive")),
                 dtc_19_02_a5=(doc.get("dtc_19_02_a5") or {}).get("resp"),
                 dtc_19_02_a5_nrc=(doc.get("dtc_19_02_a5") or {}).get("nrc"))
  if log_event is not None:
    log_event("esc_probe_0027", **summary)
  return summary


# ---------------------------------------------------------------------------------------------------------------------
# card hook
# ---------------------------------------------------------------------------------------------------------------------
def _ignition_from_services() -> tuple[str | None, bool, Callable[[], bool]]:
  import openpilot.cereal.messaging as messaging
  ds_sock = messaging.sub_sock("deviceState", timeout=1500)
  ps_sock = messaging.sub_sock("pandaStates", timeout=1500)
  ds = messaging.recv_one(ds_sock)
  ps = messaging.recv_one(ps_sock)
  key = None
  if ds is not None and ds.deviceState.started and ds.deviceState.startedMonoTime:
    try:
      with open("/proc/sys/kernel/random/boot_id") as f:
        boot = f.read().strip()
    except OSError:
      boot = "noboot"
    key = f"{boot}:{ds.deviceState.startedMonoTime}"
  ign = ps is not None and any(p.ignitionLine or p.ignitionCan for p in ps.pandaStates)

  def verify_mux_off() -> bool:
    messaging.drain_sock_raw(ps_sock)  # only states published AFTER the restore request count
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
      m = messaging.recv_one(ps_sock)
      if m is not None and len(m.pandaStates) and str(m.pandaStates[0].safetyModel) == "elm327" \
         and m.pandaStates[0].safetyParam == 1:
        return True
    return False
  return key, ign, verify_mux_off


def run_from_card(CP, can_callbacks, set_obd_multiplexing) -> None:
  """Called by card right after run_esc_diag_from_card, still before FirmwareQueryDone. Never raises."""
  try:
    from openpilot.common.hardware import PC
    from openpilot.common.swaglog import cloudlog
    if PC or os.environ.get("REPLAY") or CP.carFingerprint not in TARGET_FINGERPRINTS:
      return
    key, ign, verify = _ignition_from_services()
    can_recv, can_send = can_callbacks
    summary = run(can_send, can_recv, set_obd_multiplexing, fingerprint=CP.carFingerprint, car_fw=CP.carFw,
                  ignition_key=key, ignition_on=ign, verify_mux_off=verify, log_event=cloudlog.event)
    if not summary.get("ran"):
      cloudlog.event("esc_probe_0027", **summary)
  except BaseException as e:
    try:
      set_obd_multiplexing(False)
    except BaseException:
      pass
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.exception(f"esc_probe_0027 failed: {e!r}")
    except BaseException:
      pass
    if isinstance(e, KeyboardInterrupt):
      raise
