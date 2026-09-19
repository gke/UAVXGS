# Session Report — MATEKF411WING motors not driving (FW drive-floor bug)

**Date:** 2026-09-10
**Build/verify status:** FC clean — `MATEKF411WING` target rebuilt via
`python3 scripts/fc_build.py`, 0 errors (`_r0.bin` 233388 bytes; only the
pre-existing "LOAD segment with RWX permissions" linker note). SpeedyBee default
target rebuild deferred to the next full 7-board pass — the change is
`outputs.c`-local and target-independent.

---

## Problem (why this session)

Greg bench-tested a **MATEKF411WING** board running our firmware: the board
arms, the GCS M1/M2 bargraphs rise with throttle, **but the motors never
spin** on the physical pads. Same wiring works under iNav. Initial hypothesis
(per Greg): a pin/port definition error vs iNav.

## Investigation (what was checked)

### Pin/timer definitions — NOT the cause (verified against iNav)

| output | pad | UAVX `matekf411wing.inc` | iNav `MATEKF411` target | match |
|---|---|---|---|---|
| S1 = M1 | PB4 | TIM3_CH1 | TIM3_CH1 (S1) | ✓ |
| S2 = M2 | PB5 | TIM3_CH2 | TIM3_CH2 (S2) | ✓ |
| S3 | PB6 | TIM4_CH1 | TIM4_CH1 (S3) | ✓ |
| S4 | PB7 | TIM4_CH2 | TIM4_CH2 (S4) | ✓ |
| S5 | PB3 | TIM2_CH2 | TIM2_CH2 (S5) | ✓ |
| S6 | PB10 | TIM2_CH3 | TIM2_CH3 (S6) | ✓ |
| S7 | PA15 | TIM2_CH1 | TIM2_CH1 (S7) | ✓ |
| softserial TX | PA0 | TIM5_CH1 | PPM/softserial_tx1 TIM5_CH1 | ✓ |

(The iNav-board doc "Matek F411 Wing.md" confirms this board flies under the
iNav `MATEKF411` **target** — the newer inav `target.c|h` in `OtherCode/iNav` is
**FLYINGRCF4WINGMINI**, a red herring.)

Channel→pin routing (`DM[]` in outputs.c): drive channel 0 → pin 0 (PB4/M1
pad), channel 1 → pin 1 (PB5/M2 pad), channels 2/3 → PB6/PB7, channel 4 →
PA15. For a twin-motor elevon wing this routes the FW throttle channels
(0,1) to exactly the M1/M2 pads. **Definitions are byte-for-byte iNav-
compatible; the problem is not the port map.**

### JTAG-pin theory — closed

PB4 (M1) is JTAG NJTRST, PB3 (S5) is JTDO, PA15 (S7) is JTDI. Neither UAVX
(`harness.c` leaves PA13/PA14 alone, no SWJ remap) nor iNav
(`system_stm32f4xx.c` identical, no `GPIO_Remap_SWJ_JTAGDisable` anywhere in
the tree) disables SWJ — iNav drives the same pins fine, so JTAG is not the
differentiator.

### ROOT CAUSE — the "centralized drive floor" in `UpdateDrives` (`outputs.c`)

The drive-write loop (drives 0..`NoOfDrives`) was:

```
PWp[m] = LPF1(PWp[m], PW[m], LPF1DriveK);
if ((eCatMr) && (eInFlight || eLanding))  PWp[m] = Limit(..., pIdleThrottle, ...);
else                                      PWp[m] = 0.0f;          // ← BUG
driveWritePtr(m, PWp[m]);
```

For **any non-MR airframe** (eCatFw, eCatVtol, eCatLand) the else-branch
forced `PWp[m] = 0.0` **every tick**. `DoMotors()` had just computed the real
FW throttle into `PW[0]`/`PW[1]` (`NetThrottle` from
`DesiredThrottle`/`AltHoldThrComp`), but the loop overwrote it with 0 before
`servoWrite` → pads held at PWM_MIN (1000 µs) forever. The GCS bargraph reads
`RawPW[]`/telemetry so it showed the *computed* throttle rising — exactly the
reported "bars rise, motors dead".

The loop's own comment claimed "FW/VTOL throttle and surfaces go via
DoServos/servoWrite" — **false** for throttle: `DoServos()` for eElevonAF/
eDeltaAF/eAileronAF etc. only writes the surface channels
(RightElevonC/LeftElevonC/SpoilerC/…), never `RightThrottleC`/`LeftThrottleC`.
The throttle channels 0/1 are drive channels and are written **only** here.

Legacy reference (`UAVXArm32F4/src/outputs.c`) used the correct gate:

```
PWp[m] = (Armed() && !F.Emulation) ? LPF1(PWp[m], PW[m], LPF1DriveK) : 0.0f;
```

## Fix (adopted)

Keep the MR safety floor exactly as designed (idle floor in eInFlight/eLanding,
forced 0 on the ground — an attitude-mixed MR must never run a motor on the
bench), but restore the legacy armed gate for the non-MR drives whose channels
carry **only the throttle** (no attitude mixing, so no ground-noise hazard):

```
if (eCatMr) {
    if (eInFlight || eLanding)  PWp[m] = Limit(PWp[m], pIdleThrottle, cOutMaximum);
    else                        PWp[m] = 0.0f;
} else
    PWp[m] = (Armed() && !F.Emulation) ? PWp[m] : 0.0f;
```

- **Fixed-wing**: armed → throttle passes through to servoWrite
  (`(v+1)*1000` µs → 1000–2000 µs), disarmed → 0 → PWM_MIN, motors off.
  This is exactly the legacy 32F4 semantics.
- **VTOL pusher** (future): same armed gate — a VTOL also needs armed bench
  run-up; the old code had broken VTOL too.
- **MR**: unchanged.
- **Emulation**: `F.Emulation` still yields 0 (the `!F.Emulation` term) and
  `driveWritePtr` is still skipped; `emu.c` does not consume `PWp[]`, so the
  emulator behaviour is bit-identical.
- **Comment updated** in place; stale "go via DoServos/servoWrite" throttle
  claim removed.

## Alternatives considered / rejected

- **Manual re-wiring / pin shuffle**: rejected — definitions are provably
  iNav-identical; the bug was software.
- **Writing throttle from DoServos instead**: rejected — larger refactor, and
  the drive loop is the single charter path for drive channels; the armed gate
  in the loop is a 2-line fix that exactly restores the proven legacy model.
- **Reinstating the blanket `Armed()` gate for ALL airframes**: rejected —
  would silently re-enable bench MR prop spin that the MR floor was
  deliberately introduced to prevent.

## Verification

- `MATEKF411WING` build: clean (this report's headline build).
- Bench: reflash `obj/MATEKF411WING/MATEKF411WINGQ_r0.bin`, arm, raise
  throttle → M1/M2 pads must now sweep ~1000→2000 µs (motors spool).
- Follow-ups: rebuild remaining 6 boards on next full pass; commit via
  `update_all.sh` on host Merlin later.