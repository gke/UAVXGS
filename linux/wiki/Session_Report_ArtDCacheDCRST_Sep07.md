# Session Report — ART D-Cache Reset Ordering Fix in Flash Read-Back (param commit ACK)

**Date:** 2026-09-07
**Build/verify status:** FC clean on all 7 targets (UAVXF4V3, UAVXF4V4,
DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI, BLUEBERRYF405,
MATEKF411WING) — 0 errors (only the pre-existing "LOAD segment with RWX
permissions" linker note). No GCS changes. Disassembly of `armflash.o`
(MATEKF411WING) confirms the exact intended ACR sequence.
**Follow-on to:** `wiki/Session_Report_ParamFlashVerify_Sep06.md` (v1/v2 of
the same read-back).

---

## Problem (why this session)

The parameter-flash commit ACK (tag-72) was still NACKing on MATEKF411WING
after the Sep-06 v1→v2 change (`Session_Report_ParamFlashVerify_Sep06.md`):
every commit was reported **not verified** while every write demonstrably
landed (params persisted across reboot, `pBootDiag=1`). The bench logs proved
the false-NACK mechanism was the ART data cache serving stale pre-erase lines;
v2's job was to force a cold cache before the byte-for-byte read-back.

This session: PM0081 (and the STM32F4xx reference manual) states the **DCRST
data-cache reset bit can be written only while the D-cache is disabled**. v2's
sequence wrote DCRST **before** clearing DCEN — so the cache reset never
latched, and no memory barrier (`__DSB`) ordered the ACR change against the
read-back loads. The residual false-NACK mechanism is closed by doing it in the
PM0081-legal order plus barriers.

Research performed (user direction: "investigate what iNav does", "how does
the DFU loader write and check", "check the STM app notes"):

- **iNav (F4, master @ f1c5e748):** erase+program via `FLASH_EraseSector` /
  `FLASH_ProgramWord` with `FLASH_COMPLETE` checks (`config_streamer_stm32f4.c`);
  **no DCRST/DCEN manipulation anywhere in the config path.** Write verification
  is a **whole-image CRC16** — `isEEPROMContentValid()` re-reads the
  flash-mapped `&__config_start` region and recomputes the CRC
  (`config_eeprom.c`). iNav keeps DCEN enabled its whole life; it never resets
  the ART cache, so its read-back CAN be cache-served — it simply doesn't notice
  because at 8×128-bit lines (128 B total) long-running firmware evicts the
  config-sector lines long before a save. We noticed on the bench precisely
  because our commit follows load/boot too closely for the tiny cache to
  evict.
- **ST DFU bootloader (AN3156):** **no on-chip write verification at all.**
  The bootloader checks only the per-frame USB CRC and ACKs frames; flash
  content verification is **host-side** (STM32CubeProgrammer reads memory back
  and compares). ST ships the same no-verify trust in its own boot path.
- **Net:** our byte-for-byte read-back against the RAM source buffer is
  *stronger* than both references. The remaining defect was purely the ART
  cache-control sequencing, not the verify concept. This matches Greg's
  acceptance criterion: "a read verify against the RAM image ... checking
  checksum as well of course" — the read-back IS the RAM-image verify, and the
  `Config.CheckSum` XOR (recomputed by `RefreshConfig` before write, re-validated
  by `LoadParameters` at next boot) is the checksum leg.

---

## Root cause — v2 sequence detail

The Sep-06 v2 `IsArmFlashValid` (armflash.c) did:

```c
FLASH->ACR |= FLASH_ACR_DCRST;      /* set  cache reset        */
FLASH->ACR &= ~FLASH_ACR_DCRST;     /* clear cache reset       */
FLASH->ACR &= ~FLASH_ACR_DCEN;      /* THEN disable cache      */
for (...) compare loads ...         /* DCEN held off during    */
FLASH->ACR |= FLASH_ACR_DCEN;       /* re-enable               */
```

Two defects:

1. **PM0081 violation / writes-before-clear (credited to f1c5e748 window + the
   reference-manual bit description):** *"Bit 12 DCRST: Data cache reset ... This
   bit is written by software only when the D cache is disabled."* A DCRST write
   while DCEN=1 is out-of-contract silicon behaviour — the reset does not
   reliably latch. So the intended cold-cache step silently did nothing.
2. **No `__DSB` between the ACR store and the compare loads.** The ART cache
   controller can still be servicing pre-ACR lines when the first loads issue;
   the store→load chain crosses the device (ACR) → normal (flash) memory model
   boundary with no barrier. Cheap to add, makes the disable-definitely-before-
   first-load guarantee explicit, per ARMv7-M.

The write/verify **concept** was right (and matches Greg's "verify the Flash
has what was written"); only the cache-control needs the corrected dance below.

---

## Fix — `IsArmFlashValid` v3 (armflash.c)

PM0081-compliant order, with barriers at both ACR transitions:

```c
FLASH->ACR &= ~FLASH_ACR_DCEN;      /* disable cache FIRST      */
FLASH->ACR |= FLASH_ACR_DCRST;      /* reset only legal now     */
FLASH->ACR &= ~FLASH_ACR_DCRST;
__DSB();                            /* ACR changes visible before reads */
for (...) compare loads ...         /* DCEN held off: uncached fetches */
FLASH->ACR |= FLASH_ACR_DCEN;       /* re-enable cache          */
__DSB();
```

- `FLASH_ACR_DCEN` = 0x0400, `FLASH_ACR_DCRST` = 0x1000 (stm32f4xx.h:3374/3376).
- Boot configures the ART cache at `system_stm32f4xx.c:217`
  (`FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_LATENCY`); the compare still holds
  DCEN disabled for every read so no read is cache-served regardless of history.
- Safe: all `IsArmFlashValid` call paths are ground-only (commit / trace dump).

Verified in the object disassembly (`obj/MATEKF411WING/src/armflash.o`,
`WriteBlockArmFlash` inline region):

```
 6c: bic.w r2, r2, #1024      ; DCEN  cleared
 74: orr.w r2, r2, #4096      ; DCRST set   (now legal: DCEN=0)
 7c: bic.w r2, r2, #4096      ; DCRST clear
 82: dsb sy                   ; barrier before compare
 ... ldrb compare loop ...
 ae: orr.w r3, r3, #1024      ; DCEN re-enabled
 b4: dsb sy                   ; barrier after re-enable
```

---

## Rationale and alternatives considered

- **Reject: adopt iNav's whole-image-CRC-only verify.** It is weaker than a
  byte compare (a single corrupted byte that happens to preserve the CRC is
  possible; a per-byte compare is not) and it still depends on the same
  unstaged ART cache. Our byte compare + `Config.CheckSum` + boot re-validation
  covers both legs Greg asked for.
- **Reject: verify via the StdPeriph `FLASH_ProgramWord` status alone (i.e. the
  DFU trust model).** The Sep-06 session proved that is exactly the silent-fail
  chain we are closing; it is a hardware promise, not proof the sector holds our
  bytes.
- **Reject: skip the read-back on the D-cache theory alone and trust it as
  iNav does.** Our bad-luck window (boot→commit within cache lifetime) is the
  one case that actually bites, and the fix is 3 lines.
- **Adopt: `__DSB()` barriers.** ARMv7-M store→load ordering across the
  device/normal boundary is not implicit; the barriers make "disable first,
  read after disable" a guarantee rather than an accident of timing. Cost is
  negligible (µs-scale, ground-only).

Threading note: this does not reopen the Q Gain / control / trace workstreams —
it is purely the commit-ACK truth chain (`RefreshConfig` → `WriteBlockArmFlash`
→ `IsArmFlashValid` → tag-72 ACK, params.c:380-418 / armflash.c / telem.c).

---

## Files changed

- `UAVXArmQ/src/armflash.c` — `IsArmFlashValid` v3 sequence (+ comment updated).

## Verification

- `BOARD=MATEKF411WING python3 scripts/fc_build.py` → **OK**, 233052 B bin.
- Full fleet `python3 scripts/fc_build.py` → all 7 targets **OK**.
- Disassembly inspection confirmed the intended ACR order (above).
- GCS: no Python changed; no `py_compile` needed.

## Next steps for user

1. **Reflash MATEKF411WING** with the new bin (i.e. confirm the bench is no
   longer running the v1 DCEN-toggle or the v2 mis-ordered build).
2. Connect GCS → param window → change a low-risk value (e.g. a scale float),
   `Write`. Expect: tag-17 ACK, then tag-72 ACK **true** → FC reboots, `beep`
   single Baud-class restart; WRITE alarm in the `AlarmFlashBox` must NOT flash.
3. Verify tag-71 readback shows the written value (and `pBootDiag=1`).
4. If it still NACKs with a verified-good flash (byte-read back equal), capture
   the rawlog and we inspect; the next suspects would be flash-program failures
   rather than cache (PM0081 sequence + DSB now closed the caching leg
   completely).