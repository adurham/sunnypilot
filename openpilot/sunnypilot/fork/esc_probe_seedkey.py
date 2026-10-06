#!/usr/bin/env python3
"""
Fork: recovered G-scan2 (GIT) ``CalKeyAlgorithm_*`` seed->key functions for the esc-probe-0027 phase-4 ``algo``
key mode. Byte-identical port of car-features/esc-software/06-git/12-seedkey.py (recovered by static disassembly of
Gs2_SpecialFunc.exe — Windows CE 6.0 ARM PE, Thumb interwork; full evidence/addresses in 12-key-algorithm.md).

INTEGRATION CONTRACT (matches the on-car probe):
    27 01 -> ESC answers with an 8-byte seed (observed: 2-byte value repeated 4x, e.g. 5A B0 x4; changes each
    ignition), extended session.  The probe sends ONE 27 02 sendKey.  Vendor frame templates: 2-byte key
    (04 27 02 XX XX), 4-byte key (06 27 02 XX XX XX XX).

I/O SIZES ([confirmed] from each function's sprintf + the XML's SecuritySupported legend 1=2-byte / 2=4-byte):
    key_for(seed)        : seed = raw bytes of the 27 01 response data — ACCEPTS 2, 4 or 8 bytes (the on-car
                           8-byte form is fine).  Every algorithm consumes only seed[0:4] (Type-2) or seed[0:2]
                           (Type-1); a 2-byte input is replicated to [v0,v1,v0,v1] to match the observed repeated
                           form.  OUTPUT: 4 BYTES (27100 vendor-exact) -> frame 06 27 02 k0 k1 k2 k3.
    cal_26300(seed[0:4]) : in 4 bytes -> out 4 bytes "00 HI 00 LO" (06 27 02)
    cal_26400(seed[0:2]) : in 2 bytes -> out 2 bytes (04 27 02)
    cal_26700(seed[0:4]) : in 4 bytes -> out 4 bytes (06 27 02)
    cal_26800(seed[0:2]) : in 2 bytes -> out 2 bytes (04 27 02)
    cal_27400(seed[0:4]) : in 4 bytes -> out 4 bytes (06 27 02)

THE 8-BYTE REPETITION IS A CLUE: no recovered algorithm consumes more than 4 seed bytes.  A [v0,v1]x4 seed makes every
4-byte window identical (v0,v1,v0,v1), so the challenge is robust to which window the tester reads.  BUT an 8-byte
seed + a single 27 02 matches NO G-scan template (their 8-byte-seed type uses 27 11/27 12, Type-3 "ASK" =
certificate-signed): the ESC's live security layer is NEWER than this G-scan release.  If every candidate draws NRC
0x35 (invalidKey), the layer is almost certainly the ASK/cert kind.

VERIFICATION: implemented byte-faithfully from disassembly; NO ground-truth seed->key pair exists anywhere in the
image (the only literal keys in the binaries are FIXED constants "07D4042702EDCB" (EPS 0x7D4) and
"07E00627029135af30" (ECS 0x7E0)).  EVERY OUTPUT IS UNVERIFIED against hardware — a wrong key should draw NRC 0x35
(invalidKey), safe to iterate candidates in the given order.
"""

# ----------------------------------------------------------------------------
# bitsum — helper fcn @ 0x698a0 [confirmed]: plain 8-bit BIT REVERSAL.
#   bitsum(x) = reverse_bits8(x); bitsum(0x01)=0x80, bitsum(0x80)=0x01,
#   bitsum(0xFF)=0xFF, bitsum(0x5A)=0x5A.
# ----------------------------------------------------------------------------
def bitsum(x: int) -> int:
  x &= 0xFF
  return int(format(x, "08b")[::-1], 2)


def _lfsr16_step(st: int) -> int:
  """Galois LFSR step, poly 0xC503, 16-bit [confirmed 0x71b84-0x71b9c].

  The ARM register keeps garbage above bit 15, but the msb test is bit 15 and the XOR mask is 16-bit, so bits 0-15
  evolve exactly as this masked recurrence (the tool itself only ever extracts bits 0-15 of the result).
  """
  if st & 0x8000:
    return ((st << 1) ^ 0xC503) & 0xFFFF
  return (st << 1) & 0xFFFF


# ----------------------------------------------------------------------------
# CalKeyAlgorithm_27100 — fcn @ 0x71a00, Securityindex 24000 [confirmed 0x71aa0-0x71cdc]
#   Seed b0..b3 (= seed[0:4]); bail (None) if ANY of b0..b3 == 0.
#   Final key = 4 wire bytes [bitsum(lo), bitsum(hi), 0x00, 0x00] (06 27 02 k0..k3).
# ----------------------------------------------------------------------------
def cal_27100(seed4: bytes) -> bytes | None:
  """4 seed bytes in -> 4 key bytes out (vendor-exact, 06 27 02 k0..k3).
  None if any seed byte is 0 (vendor bails to the default/empty path)."""
  if len(seed4) != 4:
    raise ValueError("cal_27100 expects exactly 4 seed bytes (seed[0:4])")
  b0, b1, b2, b3 = seed4
  if 0 in (b0, b1, b2, b3):
    return None
  st = 0xFFFF ^ (bitsum(b1) << 8)
  for _ in range(8):
    st = _lfsr16_step(st)
  st ^= bitsum(b0) << 8
  for _ in range(8):
    st = _lfsr16_step(st)
  st ^= bitsum(b3) << 8
  for _ in range(8):
    st = _lfsr16_step(st)
  st ^= bitsum(b2) << 8
  for _ in range(8):
    st = _lfsr16_step(st)
  lo, hi = bitsum(st & 0xFF), bitsum((st >> 8) & 0xFF)
  return bytes([lo, hi, 0x00, 0x00])


# ----------------------------------------------------------------------------
# CalKeyAlgorithm_26300 — fcn @ 0x71e40, Securityindex 26300 [confirmed 0x71ef0-0x72028]
#   Seed b0..b3; bail (None) if any byte == 0.
#   val = (b2 << 24) | (b3 << 16) | (b0 << 8) | b1;  key16 = val % 0xC503
#   Output 4 wire bytes [0x00, HI, 0x00, LO] ("00HI00LO", 06 27 02).
# ----------------------------------------------------------------------------
def cal_26300(seed4: bytes) -> bytes | None:
  """4 seed bytes in -> 4 key bytes out [00, HI, 00, LO] (06 27 02)."""
  if len(seed4) != 4:
    raise ValueError("cal_26300 expects exactly 4 seed bytes (seed[0:4])")
  b0, b1, b2, b3 = seed4
  if 0 in (b0, b1, b2, b3):
    return None
  val = (b2 << 24) | (b3 << 16) | (b0 << 8) | b1
  k16 = val % 0xC503
  hi, lo = (k16 >> 8) & 0xFF, k16 & 0xFF
  return bytes([0x00, hi, 0x00, lo])


# ----------------------------------------------------------------------------
# CalKeyAlgorithm_26400 — fcn @ 0x72200, Securityindex 26400 [confirmed 0x72288-0x72328]
#   Seed 2 bytes only; bail (None) if either == 0.
#   val = (b0 << 8) | b1;  key16 = val ^ 0x26C0 -> 2 bytes big-endian (04 27 02).
# ----------------------------------------------------------------------------
def cal_26400(seed2: bytes) -> bytes | None:
  """2 seed bytes in -> 2 key bytes out (04 27 02)."""
  if len(seed2) != 2:
    raise ValueError("cal_26400 expects exactly 2 seed bytes (seed[0:2])")
  b0, b1 = seed2
  if b0 == 0 or b1 == 0:
    return None
  k16 = ((b0 << 8) | b1) ^ 0x26C0
  return bytes([(k16 >> 8) & 0xFF, k16 & 0xFF])


# ----------------------------------------------------------------------------
# Securityindex 26700 — fcn @ 0x7261c [confirmed 0x72630-0x72730]
#   st = (b0<<24)|(b1<<16)|(b2<<8)|b3;  32-bit LFSR, poly 0x3BCC14C8, XOR-then-SHIFT, 35 iterations.
#   Output 4 bytes big-endian (06 27 02).  NO zero-bail.
# ----------------------------------------------------------------------------
def cal_26700(seed4: bytes) -> bytes:
  """4 seed bytes in -> 4 key bytes out (06 27 02)."""
  if len(seed4) != 4:
    raise ValueError("cal_26700 expects exactly 4 seed bytes (seed[0:4])")
  b0, b1, b2, b3 = seed4
  st = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3
  for _ in range(35):
    if st & 0x80000000:
      st ^= 0x3BCC14C8
    st = (st << 1) & 0xFFFFFFFF
  return bytes([(st >> 24) & 0xFF, (st >> 16) & 0xFF, (st >> 8) & 0xFF, st & 0xFF])


# ----------------------------------------------------------------------------
# CalKeyAlgorithm_27400 (name) — fcn @ 0x796d4, dispatched by Securityindex 39300 [confirmed 0x8c3cc-0x8c53c]
#   Seed b0..b3; NO zero-bail.
#   k0 = (b1*b2 + b0*b3) & 0xFF;  k1 = b3;  k2 = (b3*b2 + b0*b1) & 0xFF;  k3 = b1  -> 06 27 02.
#   NOTE: two of the four key bytes are seed bytes passed through — confirmed by register liveness.
# ----------------------------------------------------------------------------
def cal_27400(seed4: bytes) -> bytes:
  """4 seed bytes in -> 4 key bytes out (06 27 02)."""
  if len(seed4) != 4:
    raise ValueError("cal_27400 expects exactly 4 seed bytes (seed[0:4])")
  b0, b1, b2, b3 = seed4
  k0 = (b1 * b2 + b0 * b3) & 0xFF
  k1 = b3 & 0xFF
  k2 = (b3 * b2 + b0 * b1) & 0xFF
  k3 = b1 & 0xFF
  return bytes([k0, k1, k2, k3])


# ----------------------------------------------------------------------------
# Securityindex 26800 — fcn @ 0x727c4 [confirmed 0x72800-0x72940]
#   Table T[0..7] = E2BF 5218 749E ECCF 47EB 2807 14CD 46E7;  2-byte seed -> v16 = (b0<<8)|b1;
#   r = ror16(v16, 2);  acc = XOR of T[(r>>k)&7] for k in 4,7,10,13,0, plus v16;  key16 = acc ^ 0xC657.
#   Output 2 bytes big-endian (04 27 02).  NO zero-bail.
# ----------------------------------------------------------------------------
_T26800 = (0xE2BF, 0x5218, 0x749E, 0xECCF, 0x47EB, 0x2807, 0x14CD, 0x46E7)


def cal_26800(seed2: bytes) -> bytes:
  """2 seed bytes in -> 2 key bytes out (04 27 02)."""
  if len(seed2) != 2:
    raise ValueError("cal_26800 expects exactly 2 seed bytes (seed[0:2])")
  v16 = (seed2[0] << 8) | seed2[1]
  r = ((v16 >> 2) | ((v16 & 3) << 14)) & 0xFFFF
  acc = (_T26800[(r >> 4) & 7] ^ _T26800[(r >> 7) & 7] ^
         _T26800[(r >> 10) & 7] ^ _T26800[(r >> 13) & 7] ^
         _T26800[r & 7] ^ v16)
  k16 = acc ^ 0xC657
  return bytes([(k16 >> 8) & 0xFF, k16 & 0xFF])


# ----------------------------------------------------------------------------
# The phase-4 ``algo`` selector.  Names are the G-scan2 Securityindex values (the vendor CalKeyAlgorithm number).
# ----------------------------------------------------------------------------
_ALGOS = {
  "27100": cal_27100,   # LFSR16 / 0xC503, 4-byte seed -> [bitsum(lo), bitsum(hi), 00, 00]
  "26300": cal_26300,   # mod 0xC503, 4-byte seed -> [00, HI, 00, LO]
  "26400": cal_26400,   # XOR 0x26C0, 2-byte seed -> 2-byte key
  "26700": cal_26700,   # LFSR32 / 0x3BCC14C8, 4-byte seed -> 4-byte key
  "26800": cal_26800,   # table-XOR, 2-byte seed -> 2-byte key
  "27400": cal_27400,   # multiply/add, 4-byte seed -> 4-byte key
}
ALGO_NAMES = tuple(_ALGOS.keys())
DEFAULT_ALGO = "27100"


def _seed4(seed: bytes) -> bytes:
  """Normalise the 27 01 raw response bytes to the 4-byte window every algorithm consumes."""
  if len(seed) >= 4:
    return bytes(seed[:4])
  if len(seed) == 2:
    return bytes(seed) * 2
  raise ValueError("seed must be 2, 4, or 8 bytes (raw bytes of the 27 01 response data)")


def key_for(seed: bytes, algo: str | None = None) -> bytes | None:
  """Phase-4 entry point: derive the candidate key with the named G-scan algorithm.

  ``algo`` selects one of ``ALGO_NAMES`` (the vendor Securityindex); ``None`` -> ``"27100"`` (the vendor-exact
  default).  Returns the 2- or 4-byte key, or ``None`` for the algorithms whose vendor default path bails on a
  zero seed byte (27100/26300/26400).  An unknown ``algo`` raises ``ValueError`` (the probe records that as an
  abort, same as any other resolve failure).
  """
  name = DEFAULT_ALGO if algo is None else algo
  fn = _ALGOS.get(name)
  if fn is None:
    raise ValueError(f"unknown algo {algo!r} (choose from {sorted(_ALGOS)})")
  s4 = _seed4(bytes(seed))
  if name in ("26400", "26800"):
    return fn(s4[:2])
  return fn(s4)


def candidates(seed: bytes) -> dict[str, bytes | None]:
  """All recovered algorithms with their vendor wire formats, keyed the same as the G-scan Securityindex.

  Order rationale: Hyundai ESC-family security of this generation is built on the 0xC503 constant (27100 LFSR /
  26300 modulus), and CONTI ESCs on sister platforms use 26300 — so the 0xC503 family is first; then the generic
  LFSR/table ones (26700/26800/27400).
  """
  s4 = _seed4(bytes(seed))
  return {
    "27100": cal_27100(s4),      # LFSR16 / 0xC503  -> 06 27 02 [lo, hi, 00, 00]
    "26300": cal_26300(s4),      # mod 0xC503      -> 06 27 02 [00, HI, 00, LO]
    "26400": cal_26400(s4[:2]),  # XOR 0x26C0      -> 04 27 02 [HI, LO]
    "26700": cal_26700(s4),      # LFSR32          -> 06 27 02 [st BE 4B]
    "26800": cal_26800(s4[:2]),  # table-XOR       -> 04 27 02 [HI, LO]
    "27400": cal_27400(s4),      # mul/add         -> 06 27 02 [k0..k3]
  }


# ----------------------------------------------------------------------------
if __name__ == "__main__":
  failures = []

  # bitsum = 8-bit reversal [confirmed by term-by-term decode of 0x698a0]
  for x, want in {0x00: 0x00, 0x01: 0x80, 0x02: 0x40, 0x04: 0x20, 0x08: 0x10,
                  0x10: 0x08, 0x20: 0x04, 0x40: 0x02, 0x80: 0x01,
                  0xFF: 0xFF, 0x55: 0xAA, 0xAA: 0x55, 0x5A: 0x5A, 0xB0: 0x0D}.items():
    got = bitsum(x)
    ok = got == want
    print(f"[{'ok ' if ok else 'FAIL'}] bitsum({x:#04x}) = {got:#04x} (expected {want:#04x})")
    if not ok:
      failures.append(f"bitsum({x:#04x})")

  # 26300: python % vs the exact ARM magic-divide sequence [0x71fac-0x71fdc]
  def arm_magic_mod_c503(n: int) -> int:
    M = 0x4CA6779F
    prod = (n * M) & 0xFFFFFFFFFFFFFFFF
    hi = (prod >> 32) & 0xFFFFFFFF
    t = (hi + (((n - hi) & 0xFFFFFFFF) >> 1)) & 0xFFFFFFFF
    q = t >> 15
    return (n - q * 0xC503) & 0xFFFFFFFF

  import random
  random.seed(1)
  mism = sum(1 for _ in range(20000)
             if (lambda n: (n % 0xC503) != arm_magic_mod_c503(n))(random.getrandbits(32)))
  print(f"[{'ok ' if mism == 0 else 'FAIL'}] 26300 ARM magic-div vs %%0xC503: {mism} mismatches / 20000")
  if mism:
    failures.append("26300 magic-div")

  # 0xC503 constant-family check: 0xC0A3 (the reflected CRC-16 poly used by the sibling 26600) is the bit-reversal
  # of the 27100 LFSR / 26300 modulus constant 0xC503 — same Hyundai family, cross-validates the RE.
  def rev16(v: int) -> int:
    return int(format(v, "016b")[::-1], 2)

  ok = rev16(0xC0A3) == 0xC503
  print(f"[{'ok ' if ok else 'FAIL'}] 0xC503 family: bitrev(0xC0A3) == 0xC503 ({rev16(0xC0A3):#06x})")
  if not ok:
    failures.append("0xC503 family")

  # Repetition property: 8-byte repeated seed behaves as its first 4 bytes
  v = bytes([0x5A, 0xB0])
  k8, k4 = key_for(v * 4), key_for(v * 2)
  print(f"[{'ok ' if k8 == k4 else 'FAIL'}] repetition: key_for(8B 5AB0x4) == key_for(4B 5AB0x2)")
  if k8 != k4:
    failures.append("repetition property")

  # Zero-seed bails mirror the vendor default paths
  z = cal_27100(bytes([0x00, 0x11, 0x22, 0x33]))
  print(f"[{'ok ' if z is None else 'FAIL'}] 27100 zero-byte seed -> None (vendor default bail)")
  if z is not None:
    failures.append("27100 zero-bail")
  z2 = cal_26300(bytes([0x11, 0x00, 0x22, 0x33]))
  print(f"[{'ok ' if z2 is None else 'FAIL'}] 26300 zero-byte seed -> None")
  if z2 is not None:
    failures.append("26300 zero-bail")

  # Selector: default is 27100; unknown name raises
  ok = key_for(bytes.fromhex("1122334455667788")) == key_for(bytes.fromhex("1122334455667788"), "27100")
  print(f"[{'ok ' if ok else 'FAIL'}] key_for default algo == 27100")
  if not ok:
    failures.append("selector default")
  try:
    key_for(bytes.fromhex("1122334455667788"), "99999")
    print("[FAIL] unknown algo should raise ValueError")
    failures.append("unknown algo")
  except ValueError:
    print("[ok ] unknown algo raises ValueError")
  ok = set(candidates(bytes.fromhex("1122334455667788"))) == set(ALGO_NAMES)
  print(f"[{'ok ' if ok else 'FAIL'}] candidates() keys == ALGO_NAMES {ALGO_NAMES}")
  if not ok:
    failures.append("candidates keys")

  print()
  print("Candidate keys by algorithm (selector names):")
  for label, seedhex in (("observed on-car seed 5AB05AB0x4", "5AB05AB05AB05AB0"),
                         ("generic seed 1122334455667788", "1122334455667788")):
    print(f"  {label}:")
    for name, k in candidates(bytes.fromhex(seedhex)).items():
      print(f"    {name}: {k.hex() if k else 'None (zero-seed bail)'}")

  print()
  if failures:
    print("SELF-TEST FAILURES:", failures)
    raise SystemExit(1)
  print("SELF-TEST: internal consistency OK.")
  print("NOTE: outputs are UNVERIFIED against any real seed->key pair (none exists in the image).")
