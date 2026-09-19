# SPL06 Barometer Bring-Up — Debug Report

**Date:** 2026-08-24
**Board:** SpeedyBee F405 Wing (`SPEEDYBEEF405WING`)
**Firmware:** UAVXArmQ quaternion fork
**Status:** RESOLVED — baro ACTIVE end-to-end, temperature and pressure correct; driver consolidated

---

## 1. Summary

The SPL06-001 barometer on the SpeedyBee F405 Wing was invisible to UAVXQ while
working fine under iNav on identical hardware — proving the fault was firmware,
not silicon. The investigation progressed through four layers, each instrumented
via growth of telemetry tag 76:

1. **Bus silence** → fixed stale-AF handling; sensor appeared at address **0x76**
2. **Pin/peripheral config doubt** → register snapshot proved silicon state textbook-perfect
3. **Init pipeline stall** ("stuck after id") → root cause: bus-recover path
   unsticking a *healthy* bus mid-STOP, shredding peripheral state
4. **Current**: baro ACTIVE, temp ≈ 27.8 °C (plausible), pressure ≈ 15 793.6 Pa
   (should be ≈ 101 325) — compensation-input instrumentation in flight

---

## 2. Timeline & Findings

### Phase 1 — The bus was clocking but nobody answered

Bus census (per-bus ACK count / START-fail count / ACKing addresses, streamed in
tag 76) reported an alive-but-empty I2C1. Config looked right in code.

**Fix:** stale-AF handling. A NACK left over from any earlier failed probe set
the `AF` bit in `SR1`; from then on every *genuine* ACK read as failure.
`I2CPollRead` / `I2CProbeAddr` now clear AF before every transaction and always
generate a STOP on every outcome. After this: `[CENSUS] I2C1: 0x76`.

**Learning:** on STM32F4 polled I2C, one NACK poisons everything downstream
until explicitly cleared.

### Phase 2 — "Is the config real, or only right in the source?"

Added a raw register snapshot (GPIOB `MODER/OTYPER/PUPDR/AFRH`, I2C1
`CR1/CCR/TRISE`) captured immediately after the census and streamed in tag 76.

First capture attempt had a self-inflicted truncation bug: casting 32-bit
`MODER`/`PUPDR` to u16 keeps pins 0–7; PB8/PB9 live in bits 16–19. Caught
before flashing; fixed with `>> 16`.

Healthy reference vs observed (@100 kHz, APB1 = 42 MHz):

| Register | Expected | Observed | Verdict |
|----------|----------|----------|---------|
| MODER (>>16) | pins 8/9 = AF (`…A`) | `5FFA` | ✓ |
| OTYPER | bits 8/9 open-drain | `0300` | ✓ |
| PUPDR (>>16) | pull-ups (`…5`) | `5005` | ✓ |
| AFRH | nibbles = 4/4 (AF4) | `44` | ✓ |
| CR1 | PE=1 (+ACK/POS from 2-byte path) | `0601` | ✓ (POS is intentional) |
| CCR | 42 M/(2·100 k) = 210 | `210` | ✓ |
| TRISE | pclk1 MHz + 1 = 43 | `43` | ✓ |

**Learning:** `CR1=0601` looked like corruption but is by-design — the EV
handler sets POS for exactly the 2-byte reads our driver uses.

### Phase 3 — Sensor talks, init stalls anyway

Detection-pipeline diag added to tag 76 (`idMatch/coeffReady/calOK/active`
bits + raw ID read). Result decoded as: **ID matched (0x10), coeff/cal never
followed** — i.e. first transaction OK, everything after dead.

**ROOT CAUSE — the healthy-bus unstick race:**

- Every transaction leaves `SR2.BUSY` set ~90 µs while its STOP frame clocks out.
- `I2CPollRecoverBus()` treated any BUSY-at-entry as a wedged bus and called
  `UnstickI2C()`, which re-muxes PB8/PB9 from AF to GPIO-OD, bit-bangs 9 SCL
  pulses + a bogus STOP *while the peripheral believes it owns the pins*, then
  re-muxes to AF.
- SPL06 init runs back-to-back polled reads: ID read succeeds → code falls
  straight into the coeff poll with zero gap → RecoverBus sees the in-flight
  STOP → unnecessary bus-clear → peripheral state shredded → **all subsequent
  reads fail**. Exactly matched the diag signature (idMatch=1, coeff=0, cal=0).
- The census sweep never hit this: its 112 probes self-space (~ms apart) and it
  never calls RecoverBus.

**Fix:** bounded spin-wait for BUSY to clear (up to `I2C_DEFAULT_TIMEOUT`);
unstick only if BUSY persists past any legal frame time.

**Learning:** BUSY-after-transaction is normal life at 100 kHz, not a fault.
Never bus-clear without proving the bus stays busy.

### Bundled no-regret fixes (same cycle)

| Fix | Why |
|-----|-----|
| Soft reset (`0xB6` → reg 0x0C) after detection + 10 ms settle | Chip lives on the always-on 3V3 rail — survives FC reboots and accumulates stale state across flash/test cycles |
| c21 decode typo `buf[13]` → `buf[15]` | Genuine coefficient-corruption bug (c21 duplicated c20's low byte) |
| `ReadCalibration()` gated on CoeffReady | Don't burst-read coefficients before the chip says they're valid |
| Coeff-ready wait widened 30×1 ms → 50×2 ms | Thin budget before; datasheet-friendlier |
| Uninitialized `CalOK` local initialized | Read garbage when ID mismatch short-circuited the pipeline |

### Phase 4 — Pressure wrong: nine small reads vs one packed stream

Ground-truth streaming went in next: tag 76 grew to 84 bytes carrying all nine
coefficients, the last raw P/T samples and PRS/TMP/CFG echo-backs, with the GCS
recomputing the datasheet compensation (`_spl_verify`) into log.txt. Verdict:
temperature path sane, but `rawP` itself bogus (~ −3.5 M counts where
~ +4.5 M expected) — compensation exonerated, acquisition inputs guilty.

**ROOT CAUSE — misaligned coefficient decode:**

- SPL06 packs its calibration as ONE 18-byte big-endian stream across regs
  0x10..0x21, with 20-bit `c00`/`c10` straddling nibble boundaries.
- `ReadCalibration()` did nine separate 2-byte reads (regs 0x10..0x18) and
  indexed the bytes as if each register held whole fields. Everything past the
  first nibble straddle came out rotated; the tail reads at 0x18/0x19 weren't
  coefficient bytes at all (c30 lives in 0x20/0x21 and was never read).
- Temperature survived because c0/c1 occupy exactly the first three bytes,
  which the earliest reads delivered verbatim.

**Fix:** single contiguous 18-byte `I2CPollRead` from 0x10, then iNav's
shift-based unpack (`read_calibration_coefficients()` equivalent).

Verified live on hardware:

```
[SPL06-COEF] c0=205 c1=-290 c00=-8239 c10=201570 c01=25151 c11=16183
             c20=14266 c21=-17676 c30=-2851
[CENSUS] ... | SPL06 id=0x10 ACTIVE cfg=(0x03,0x83,0x00)
```

Pressure immediately correct (user-confirmed). Temperature later read ~37 °C —
die/PCB self-heating after many powered flash-test cycles; normal for
PCB-mounted baros (+5–15 °C over ambient), deliberately not chased.

**Learning:** per-register small reads silently assume fields are
register-aligned. Packed sensor tables (nibble straddles, >8-bit fields) must
be block-read and unpacked once, or not at all.

---

## 3. Final State

```
[CENSUS] I2C1: 0x76 (startFails=0) | I2C2: none (startFails=0) |
         MODER=5FFA OTYPER=0300 PUPDR=5005 AFRH=44 CR1=0601 CCR=210 TRISE=43 |
         SPL06 id=0x10 ACTIVE cfg=(0x03,0x83,0x00)
[SPL06-COEF] c0=205 c1=-290 c00=-8239 c10=201570 c01=25151 c11=16183
             c20=14266 c21=-17676 c30=-2851
```

- Baro ACTIVE end-to-end: detect @0x76 → soft reset → coeff-ready wait →
  18-byte block calibration read → configure (PRS_CFG=0x03, TMP_CFG=0x83
  with TMP_EXT=1, CFG_REG=0x00) → polled T/P cycle (~34 ms) → compensation →
  density altitude via Median3.
- **Temperature:** plausible (ambient + PCB self-heating).
- **Pressure:** correct — user-confirmed against reference.
- Driver fully polled: `I2CPollRead/I2CPollWrite` from `GetBaro()` in the
  main loop; no ISR/DMA involvement for the baro path. Every transaction is
  timed at the SIO choke point (`SIOAccumTiming`) so avg/peak exec stats
  populate from the first transaction — there was never an armed gate; the
  polled path simply bypassed where the timing lives.

## 4. Consolidation Pass

Driver tidied from debug cobble into a coherent iNav-derived unit, plus the
integrity work that fell out of it:

| Change | Why |
|--------|-----|
| Dead `CoeffU` union removed | Leftover of an abandoned union-decode experiment |
| `PrintCensus()` moved from spl0601.c into `I2CCensusBus()` (i2c.c) | Census data and its reporting belong together; the baro driver is now pure sensor logic |
| `DiagStage` comment corrected | Scheme settled: 0 ok / 10 coeff-wait stuck / 11 bus fail / 12 readback mismatch |
| Dual-read coefficient verification | No chip-side checksum exists; two independent 18-byte block reads must agree byte-for-byte before the decode runs, else `DiagStage=12` |
| MS56xx dead `BaroCheckCRC()` wired into init | The PROM CRC4 check existed but had no caller; it now gates `F.BaroActive` on MS5611/5607 |
| SPL06 traffic routed through `SIOAccumTiming` | Polled reads/writes bypassed the SIO choke point entirely — baro avg/peak exec times never accumulated |

Union/bitfield coefficient decoding was considered and rejected: the stream is
MSB-first big-endian with nibble straddles while Cortex-M4 numbers overlay bits
LSB-first, making the straddled fields non-contiguous in target bit order — no
legal bitfield declaration can capture them, and big-endian members would
byte-swap anyway. The explicit shifts mirror iNav line-for-line and stay.

No checksum exists over the SPL06 coefficients chip-side (`COEFF_RDY` only
attests the OTP row loaded into registers, nothing about our readback), which
is why Phase 4's misaligned decode produced plausible-looking garbage
undetected; the dual-read verification above closes that gap. MS5611/5607 by
contrast ship a factory CRC4 over their PROM words (AN520) — it just was never
being checked.

## 5. Tag 76 Growth Record

| Rev | Len | Added |
|-----|-----|-------|
| base | 21 | per-device busNo, error counts, avg/peak µs |
| +census | 33 | per-bus ACK/startFail counts + up to 4 ACKing addrs ×2 buses |
| +regs | 47 | GPIOB/I2C1 register snapshot (bus 1) |
| +diag | 49 | SPL06 pipeline bits + raw ID read |
| +stage | 51 | last MEAS_CFG, stage code (10=coeff stuck, 11=bus fail, 12=readback mismatch) |
| +truth | 84 | all 9 coefficients, rawP/rawT, PRS/TMP/CFG echo-backs — made Phase 4 decidable in one flash |

All extensions are length-keyed optional trailers — old parsers ignore them.

## 6. Files Touched

- `UAVXArmQ/src/i2c.c` — AF clearing, census, reg capture, RecoverBus race fix
- `UAVXArmQ/src/i2c.h` — diag externs + expected-value documentation
- `UAVXArmQ/src/sensors/spl0601.c` — soft reset, gating, diag capture, single 18-byte block calibration read
- `UAVXArmQ/src/altitude.h` — 0x76 primary addr, diag externs
- `UAVXArmQ/src/telem.c` — tag 76 payload growth
- `uavx-python/src/packet_parser.py` — tag 76 parsing growth + `_spl_verify()` offline compensation check
- `uavx-python/src/ui/main_window.py` — `[CENSUS]` printing/dedupe refresh + `[SPL06-COEF]` decode

Both boards build clean (`SPEEDYBEEF405WING`, `FLYINGRCF4WINGMINI`).

## 7. Transferable Lessons

1. **Clear AF (and any sticky status) before trusting an ACK check** — one NACK
   must not define the rest of the session.
2. **Snapshot the registers, not just the code path** when hardware misbehaves
   against apparently-correct configuration.
3. **BUSY ≠ wedged.** Wait it out first; bit-bang recovery only on persistence.
4. **Chips on always-on rails outlive reboots** — reset peripherals you're about
   to trust, every boot.
5. **Stream the pipeline's internal state** (stage counters, raw reads) rather
   than bisecting by reflashing guesses; each tag-76 extension paid for itself
   within one flash cycle.
6. **Per-register reads assume field alignment.** Packed coefficient tables
   with nibble straddles must be block-read then unpacked once — nine
   "obvious" 2-byte reads produced plausible-looking garbage that cost a full
   debug cycle.
7. **Endianness kills union overlays.** Big-endian nibble-packed wire data on
   a little-endian core makes bitfield overlays illegal; explicit shifts are
   the minimal correct expression.
8. **No checksum means no alarm.** SPL06 exposes no CRC over its calibration
   block; only ground-truth streaming exposed the corruption.

---

*PDF conversion is run by the user on host Merlin via `scripts/md2pdf.sh`; this
file is intentionally left as `.md`.*
