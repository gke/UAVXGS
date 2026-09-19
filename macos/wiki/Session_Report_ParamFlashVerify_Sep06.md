# Session Report — Verified Parameter Flash Writes + GCS Flash-Alarm Box

**Date:** 2026-09-06
**Build/verify status:** FC clean on all 7 targets (UAVXF4V3, UAVXF4V4,
DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI, BLUEBERRYF405,
MATEKF411WING) — 0 errors (only the pre-existing "LOAD segment with RWX
permissions" linker note). GCS `py_compile` clean on all changed files.

---

## Problem (why this session)

On MATEKF411WING, parameter writes were not persisting across reboot. The
write path had several silent-fail chains that (a) made real flash failures
invisible and (b) made the GCS *believe* writes had succeeded:

- `WriteBlockArmFlash` ignored the return of `FLASH_EraseSector` /
  `FLASH_ProgramWord` — a failed erase left a partially-programmed sector and
  the routine reported success.
- `RefreshConfig()` unconditionally cleared `ConfigChanged` and returned void —
  a failed write was silently dropped and never retried.
- The tag-72 commit handler always ACKed `true` and always rebooted, so the
  GCS proceeded to a post-reboot verification of flash content that the FC
  never actually wrote.
- The GCS dropped the tag-72 ACK frame entirely.
- The GCS status message font was too small to be usable.

The trigger was "confirmation = confirm the Flash has what was written": a
program-complete flag is a hardware-level promise, not proof that the sector
now contains our bytes.

---

## FC changes (UAVXArmQ/src/)

### `armflash.c` — single verified choke point

- New static `IsArmFlashValid(uint32 a, uint32 l, uint8 *v)` (armflash.c:40):
  reads the programmed block straight back from physical flash and
  byte-compares it against the source buffer.
  - **D-cache hazard addressed (two iterations):** the ART data cache
    (`FLASH_ACR_DCEN`, stm32f4xx.h:3374; set at boot by
    `system_stm32f4xx.c:217`) can serve a read-back from a line cached *before*
    this sector was erased/programmed (e.g. `LoadParameters` read the config at
    boot) — a false match/mismatch instead of real flash.
    - **v1 (0920):** toggled `FLASH->ACR &= ~FLASH_ACR_DCEN; FLASH->ACR |=
      FLASH_ACR_DCEN;` before comparing. **PROVEN WRONG on hardware (see
      Verification status):** DCEN on/off is NOT a documented invalidation, so
      the compare was still served from stale pre-erase cache lines → every
      commit NACK'd even though every write landed (bench logs: param 118
      5→4 NACK'd @212013 yet read back **4.0000** after reboot @213110,
      bootDiag=1; param 15→22 NACK'd @185818 persisted across all later
      reboots — repeated on 4 sessions).
    - **v2 (final):** the real ART data-cache reset is `FLASH_ACR_DCRST`
      (0x1000) — set then cleared — **and `FLASH_ACR_DCEN` is held disabled
      for the whole compare** so every byte is an uncached fetch, then DCEN is
      re-enabled. Safe here: all write paths are ground-only (`RefreshConfig`
      gates `State != eInFlight`), so the cache flash is harmless.
  - Uses `volatile int8 *` reads and an early `break` so a single bad byte
    fails fast.
- `EraseArmFlash` now returns `boolean` — result of
  `FLASH_EraseSector(sector, VoltageRange_3) == FLASH_COMPLETE` (armflash.c:57).
- `WriteBlockArmFlash` now returns `boolean` (armflash.c:72):
  - erase result captured; a failed erase **skips programming** (leaves the
    prior sector content intact).
  - each `FLASH_ProgramWord` result checked; aborts on the first failure.
  - final proof is `IsArmFlashValid` — only a byte-for-byte read-back match
    returns true. This is the single choke point for every config /
    airframe-name / trace / blackbox write.
- Prototypes updated in `armflash.h:51-52`.

### `params.c` — `RefreshConfig()` becomes the truth-teller

- `RefreshConfig` now returns `boolean` (params.c:380), prototype in
  `params.h:542` and `nvmem.h:66`.
- Both `WriteBlockArmFlash` calls are checked. The AirframeName region
  (erased by the whole-sector erase of the CONFIG sector) is only rewritten
  **after** the Config block itself verifies — a failed erase leaves the
  sector in an unknown state, so writing the name alone would add a second
  partial write on top of a broken sector.
- `ConfigChanged = false` is only cleared when **both** blocks are confirmed;
  on failure it stays set so the next `RefreshConfig` retries the whole
  sector rather than silently discarding the pending values.
- In-flight guard unchanged: `ConfigChanged && (State != eInFlight)` → returns
  false without writing, so an in-flight commit now NACKs instead of silently
  persisting nothing.

### `telem.c` — commit ACK reflects the flash, not the intent

- Tag-72 handler (telem.c:1117-1133):
  ```c
  ConditionParameters();
  ok = RefreshConfig();
  SendAckPacket(s, UAVXParamCommitTag, ok);
  if (ok) { Delay1mS(500); systemReset(false); }
  else    { DoBeeps(2); }
  ```
  Rationale: reboot only when the config (+ name) are **confirmed** in flash.
  A failed write must not be ACKed — the GCS alarms on the NACK and the FC
  keeps running so the user can retry instead of rebooting into the old,
  unpersisted values (the original symptom). `DoBeeps(2)` is the local
  audible arm of the warning (auxiliary to the GCS alarm box).
- Tag-75 (`UAVXAFNameSetTag`) handler now ACKs the true `RefreshConfig()`
  result (telem.c:1149), plus restores the name region in the same
  erase/write cycle.

### `trace.c` / `nvmem.c` — same verified discipline

- `TraceCommit` (trace.c:282-288): records written with erase, then the
  24 B critic trailer programmed on the SAME erase cycle; `TraceCommitted`
  is only set when both blocks verify. A failed commit leaves the previous
  sector content intact (erase fails atomically) and the dump falls back to
  the older committed copy instead of a partial.
- `nvmem.c:205`: `F.HaveNVMem = EraseArmFlash(BLACKBOX_FLASH_SECTOR)` — the
  blackbox "format" now reports whether the erase actually completed.

---

## GCS changes (uavx-python/src/)

### New `widgets/alarm_flash.py` — `AlarmFlashBox`

- "In your face" alert box: dark/blank when idle, flashing red with bold white
  text when any alarm source is active (500 ms blink).
- **Multi-source** model: `set_alarm(source, message)` / `clear_alarm(source)`
  are independent per source key; the box shows the union and only goes dark
  when every source is cleared. This matters because FC telemetry periodically
  reports a benign alarm state (0/1) — an independent `clear_alarm('FC_ALARM')`
  must not wipe a persisted `'WRITE'` failure, and vice versa.
- Sources today: `FC_ALARM` (FC flight `AlarmState` from tag-14, real alarms
  ≥ 2) and `WRITE` (tag-72 commit NACK, post-reboot verify mismatch, no-params-
  stored-to-verify).
- Self-contained repaint in `paintEvent` (no matplotlib/pyqtgraph — GCS venv
  constraint); 188×188 minimum.

### `ui/main_window.py`

- Left panel now: `alarm_box` (stretch 1) above the artificial horizon;
  `attitude` minimum size halved 375 → 188 px (stretch 2). Tooltip explains
  the box. This matches Greg's directive: "halve the AH and place the error
  flash box the same size above it".
- Tag-72 ACK (case 51, was previously dropped): `ok=True` → log "Flash commit
  CONFIRMED in FC" + `clear_alarm('WRITE')`; `ok=False` → log failure +
  `set_alarm('WRITE', "FLASH COMMIT FAILED\nparams were NOT saved!")`.
  Also routes to `param_window.on_param_commit_ack(ok)` (main_window.py:2482).
- Tag-14 alarm-state path feeds `set_alarm('FC_ALARM', ...)` for `alarm >= 2`,
  else `clear_alarm('FC_ALARM')` (main_window.py:3008-3013).
- `status_msg` font 10px → **14px** (both the Ready baseline and the
  `log_debug` stylesheet); removed the dead `clear_debug()` NameError
  (`index` was undefined, method had no callers).

### `ui/parameter_window.py`

- New `on_param_commit_ack(ok)` (parameter_window.py:1430): `ok=True` logs and
  returns (deferred post-reboot tag-71 verification still runs); `ok=False`
  hides the progress, `reset_write_button()` and disarms
  `_pending_param_verification`.
- "No parameters stored for verification" path and the post-reboot
  read-back-mismatch path raise the `WRITE` alarm (parameter_window.py:1133,
  1317).
- A fresh write attempt clears the `WRITE` alarm at the start (an explicit new
  try, so no alarm is carried over into it).

---

## Decision log / alternatives considered

- **Confirm-by-readback (Greg's directive) vs program-complete-only:** adopted.
  `FLASH_ProgramWord == FLASH_COMPLETE` is a controller-level completion
  promise, not evidence that flash physically holds the bytes; the read-back
  closes the last silent-fail gap. The ART D-cache toggle makes the read-back
  trustworthy (see hazard note above).
- **NACK + keep-running vs ACK + always reboot:** adopted NACK + keep running +
  `DoBeeps(2)`. Rebooting after a failed write boots into the old values and
  the user has to discover the failure — the exact bug class we are killing.
  Keeping the FC alive lets the retry path work immediately.
- **In-flight commit** returns false (guard `State != eInFlight`) rather than
  writing mid-air; the GCS surfaces the NACK instead of silently dropping the
  params.
- **Name-region write ordering**: name restored only after the Config block
  verifies — avoids stacking a partial write onto a failed sector.
- **Retry semantics**: `ConfigChanged` stays set on failure → the normal
  ground `RefreshConfig` cadence retries until it succeeds, so a transient
  erase/program glitch self-heals without user intervention while still being
  reported loudly on the immediate commit.

---

## Verification status

- FC: `python3 scripts/fc_build.py` — all 7 targets OK (v1 and v2), 0 errors.
- GCS: `python3 -m py_compile` clean on `widgets/alarm_flash.py`,
  `ui/main_window.py`, `ui/parameter_window.py`.
- **Hardware (2026-09-06 evening, Greg, MATEKF411WING):** with the v1
  (DCEN-toggle) firmware every tag-72 commit **NACK'd** yet every changed
  param **persisted across reboot** — proven from the always-on rawlogs:
  - Session `20260906_212013`: param 118 written 5→**4**; commit @195897ms
    `ok=0`; post-reboot session `20260906_213110` (`bootDiag`=1 "restored
    from flash") reads **4.0000** — the NACK was FALSE.
  - Sessions `185818`/`211539`/`211738`: param 15 → 22, same NACK-then-
    persisted fingerprint, 4 sessions total.
  - Explanation (not write failure): `LoadParameters` at boot reads the
    config → ART data cache holds pre-erase lines; DCEN toggle ≠ invalidation;
    verify read stale bytes → false mismatch. **Write path was healthy all
    along** — silent-fail chain removed, but the read-back was the new liar.
- **v2 (DCRST + DCEN-off compare) firmware built clean — pending reflash +
  retest (Greg):** commit should now return a truthful CONFIRMED (green) with
  a reboot, and the box should stay empty/colorless until a real kaboom.
- GCS robustness fixes in the same pass: `update_motors_display` now guards
  NaN PWM/throttle floats (`math.isfinite`) instead of crashing the whole GCS
  with `ValueError: cannot convert float NaN to integer`; `AlarmFlashBox`
  resized to one-third height (188→63 px) and fully empty/colorless (no fill,
  faint 1 px placeholder border) until a source raises; status font 14 px;
  dead `clear_debug()` NameError removed.

## Cross-references

- AGENTS.md "Parameter System" / "Key Protocol Details" (tag-72 semantics).
- Prior: `Session_Report_AFFleetAudit_Sep06.md` (same day, different topic).