# Session Report — Armed-Pin Kill Switch (UAVX boards only) + GCS Speech De-noise + D8 on UART4

Date: 2026-09-01 (Prof Greg)

## Summary

Three pieces of work landed this session: (A) reinstated a **kill-style
use of the legacy Armed pin** as an active-low arming gate scoped to **UAVX-prefixed
targets only**, (B) reduced GCS speech chatter and added a direction/distance
callout, and (C) **moved the kill switch to PA1 and put FrSky D8 telemetry on a
hardware UART (UART4 TX/PC10)** — replacing the bit-banged soft-serial carrier and
freeing the Armed pin to let D8 live on a real UART. They share the PA1/PC10 pins but
each is independently described below.

---

## Part A — Armed pin as an arming kill switch (FC, UAVX boards only)

### The change (two files)

**1. `UAVXArmQ/src/main.h:25-31`** — `ArmingSwitch` redefined to be active-LOW and
target-scoped:

```c
#define ArmingSwitch (!GPIOPins[eArmedSel].Used || DigitalRead(&GPIOPins[eArmedSel].P))
```

- The Armed pin (`eArmedSel`) is **active-low**: it must read **high** for arming to
  be allowed at all.
- It carries the **internal pull-up** (`InpPinConfig` = `...GPIO_PuPd_UP`, harness.h:173),
  so a config with **no switch reads high → arming allowed**; a switch wiring the pin
  **to ground kills arming**.
- The `!GPIOPins[eArmedSel].Used ||` guard means the pin is only dereferenced where a
  target actually maps it (**UAVX v3/v4 = PC10**); all other targets treat the pin as
  "allowed" and never read a null GPIO.

**2. `UAVXArmQ/src/alarms.c` `Armed()`** — the pin is only consulted in **SwitchArming**
mode; **TxArming ignores it entirely**:

```c
if (State != eInFlight) {
    if (pArmingMode == eSwitchArming)
        F.IsArmed = ArmingSwitch && !F.RCMapFail;
    else
        F.IsArmed = TxSwitchArmed && !F.RCMapFail;
}
```

Old form was `((TxSwitchArmed || pArmingMode==eSwitchArming) && ArmingSwitch) && !F.RCMapFail`,
which applied the kill gate in BOTH modes. The user (2026-09-01) asked to add a layer so
**the pin is ignored unless SwitchArming is selected** — so a fitted (or absent) kill switch
has no effect under Tx arming.

### Which pin — the nominal name

- GPIO selector: **`eArmedSel`** (harness.h enum, index 1).
- Physical pin: **PA1** on both `uavxf4v3.inc` and `uavxf4v4.inc`
  (`{ true, PA1, InpPinConfig, } // Armed (was PC10; freed for UART4 TX)`).
  **See Part C** — the kill was moved from PC10 to PA1 to free PC10 for D8 telemetry.
- **Nominal name: "Armed".** DevEBoxF4 uses **PB0** (kept but flagged `??????`).

### Pin-group caveat — UART4 is NOT free on the old Armed pin (CORRECTION to earlier analysis)

The board header comments (`// Arming, LED3, LED2`) and the `{false, SPI3,{PC10,PC11,PC12}}`
/ `{false, UART4, PC10, PC11, ...}` candidate footprints suggested PC10 might be part of a
spare UART4/SPI3 group. **It is not.** On UAVX F4 V3/V4 the whole PC9–PC12 block is already
committed (verified 2026-09-01):

| Pin | V3/V4 | Role |
|---|---|---|
| PC9 | `{true, InpPinConfig}` | **eLanding** (active input) |
| PC10 | `{true, InpPinConfig → UART4 TX}` | **Armed → D8 telemetry TX** (see Part C) |
| PC11 | `{true, LEDPinConfig}` | **LED Blue** (active output) |
| PC12 | `{true, LEDPinConfig}` | **LED Green** (active output) |

UART4's pins are **PC10 (TX) and PC11 (RX)**. **Two options existed**: keep PC11 as
UART4 RX (killing the Blue LED) or run **TX-only** (D8 is one-way, FC→RX). We chose
**TX-only** so PC11 stays the Blue LED (see Part C). The old conclusion — "you cannot use
UART4 here without giving up the Armed pin and the Blue LED" — is now **moot for PC10**:
the Armed pin moved to PA1, and D8 needs only TX (PC10).

### Rationale / discourse

- Reused the existing `ArmingSwitch` infrastructure (present since the port and identical
  to legacy `UAVXArm32F4`) instead of adding a new config bit or pin — the pull-up
  hardware was already correct.
- **Tx-mode pull-up concern resolved by scoping:** because `ArmingSwitch` is not read in
  Tx mode, the ~40 kΩ internal pull-up quality is irrelevant there. It only matters in
  Switch mode, where it fails **toward-safe** (a noise glitch reads low → no arming).
  An external 4.7–10 kΩ pull-up is only warranted for a long/EMI-prone switch run; not
  required for a clean short install.
- **SwitchArming at large was NOT reinstated** (user dropped it — a friend will fly
  Tx-arm). Only the happy-side percentage of the old switch pin is used: as a kill gate.
- No GCS change: the Ecks remains `eTxArming` and is unaffected.

### Verification

- All 6 targets build clean: `UAVXF4V3`, `UAVXF4V4`, `DEVEBOXF4`, `SPEEDYBEEF405WING`,
  `FLYINGRCF4WINGMINI`, `BLUEBERRYF405` (`python3 scripts/fc_build.py`).

---

## Part B — GCS speech de-noise + direction callout

### Reduced redundant qualifiers (`core/speech.py`)

Units/context words dropped where the value's meaning is obvious by context (user
directive 2026-09-01: "we know the units so there is no need to state them"):

| Event | Before | After |
|---|---|---|
| Battery (status) | "Battery {v} volts" | **"Battery {v}"** |
| Low battery (alert) | "Warning, battery low: {v} volts" | **"Low battery {v}"** |
| GPS acquired | "GPS fix acquired, {n} satellites" | **"GPS acquired, {n}"** |
| Alarm | "Alarm: {alarm}" | **"{alarm}"** |
| Waypoint reached | "Reached waypoint {n}" | **"Reached {n}"** |
| Altitude | "Altitude {n} meters" | **"Altitude {n}"** |

Kept: **nav mode** (flight-state change) and **altitude**.

- **Altitude was dead code** (no call site). Now wired into `_check_speech_events`
  (main_window.py) to speak on each **5 m** step, tracked via `_last_spoken_alt`
  (inverse-multiply `alt * 0.2` for the 5 m bucket, not division).
- **Voice** changed from `en-gb+f3` to **`en-gb+f1`** + rate 135 (warmer/calmer) in both
  the pyttsx3 primaries and the espeak-ng/espeak fallback `_VOICE_ARG`. Piper/MBROLA were
  considered and **parked** — the pilot relies on the TX, laptop speech is background,
  and a neural engine was disproportionate scope (user agreed 2026-09-01).

### Direction + distance callout (main_window.py)

- Source is the **FC's own math**: `self.guidance_data` from tag-59
  `SendGuidancePacket` (in-flight + OriginValid gated on the FC). The FC already
  computes aircraft-relative-to-home distance (m) and bearing (deg) — the GCS does **not**
  re-derive a haversine.
- Speaks only when **> 500 m** (`_DIRECTION_FAR_M`), rounded to **10 m** (`*0.1` inverse),
  bearing snapped to **8 compass points** (`N/NE/E/SE/S/SW/W/NW`, 45° sectors via
  `* (1/45)`, `% 360`), e.g. **"640, South-West"**.
- **Periodic**: re-speaks every `_DIRECTION_SPEAK_PERIOD_S` = 10 s while beyond 500 m
  (and sooner when the 10 m bucket changes), via `_last_direction_speak`.
- New helper `speak_direction(distance, point)` at **STATUS** level (silenced by
  Alerts-only/Off).
- New `import`/state: `_last_direction_speak`, `_last_spoken_alt`, module constants
  `_DIRECTION_FAR_M`, `_DIRECTION_SPEAK_PERIOD_S`, `_DIRECTION_POINTS`. Removed the stray
  `import math` that became unused once the guidance-data (not haversine) route was adopted.

### Verification

- `python3 -m py_compile src/core/speech.py src/ui/main_window.py` — clean.
- Compass/rounding/periodic logic validated headless (due N/E/S/W/SW/NE/SE/NW, 22.5°/67.5°
  boundaries, 10 m rounding, 10 s period).
- `sync_kits.sh` (master_update) will push these to linux/macos/windows kits.

---

## Part C — FrSky D8 on a hardware UART (UART4 TX) + kill switch moves to PA1

### Motivation / engineering discussion

The user asked whether the legacy **Armed pin (PC10)** could be repurposed for FrSky D8
telemetry instead of the bit-banged **PA1 soft-serial** (which browns out ~25% CPU budget
on the 9600-baud softserial TX). The block is a genuine win: **D8 is a one-way, TX-only
protocol** (FC → FrSky RX → radio), so a **hardware UART** is the natural carrier — no
bit-banging, no TIM5, no ~25% softserial headroom.

Investigation confirmed the serial layer **already supports UART4 hardware**:
- `UART4_IRQHandler → SerialISR(4)` (isr.c:283) — the generic TX/RX ISR.
- `TxChar(s, …)` `default` branch pushes to `TxQ[s]` + enables `USART_IT_TXE` (serial.c:260).
- `InitSerialPort` clocks UART4 (harness.c:569) and init's TX/RX GPIO in AF mode.
- `TxQ[eMaxSerialPorts][]` already allocates a per-port TX ring — **no new ring/ISR needed**.

The **only** softserial tie was the dispatcher gate `s == eSoftSerial`
(frsky.c:571). Two boards (MATEKF405TE, FLYINGRCF4WINGMINI) already set
`FrSkySerial = eUart4` but `SendFrSkyTelemetry` did **nothing** for them — a silent no-op.
The fix is exactly the user's suggestion:

```c
void SendFrSkyTelemetry(uint8 s) {
	if (s == FrSkySerial)
		SendFrSkyDTelemetry(s);
}
```

Now the same D8 frame builders (which all emit via generic `TxChar(s, …)`) run over
whatever port `FrSkySerial` names — softserial **or** a hardware UART.

### The changes

1. **`telemetry/frsky.c` `SendFrSkyTelemetry`** — gate changed from `s == eSoftSerial`
   to `s == FrSkySerial`. Fixes the latent no-op on the two UART4-FrSky boards and
   enables the UAVX UART4 swap.

2. **`uavxf4v3.inc` / `uavxf4v4.inc`**:
   - `SerialPorts[eUart4]` (index 4) enabled: `{ true, UART4, PC10, { 0,0,0 }, 9600, ...,
     UART4Config }` — **TX-only** at D8's 9600 baud.
   - `GPIOPins[eArmedSel]` (index 1): **PC10 → PA1** — the kill moves to the (now-dormant)
     old soft-serial pin. `{ true, PA1, InpPinConfig } // Armed (was PC10; freed for UART4 TX)`.
   - `SoftSerialTxPin` → **dormant** (`{ false, PA1, OutPinConfig }`). The soft-serial carrier
     is no longer used on these boards; PA1 becomes a plain pull-up input for the kill hook.
   - `InitTarget`: `FrSkySerial = eUart4; InitSerialPort(FrSkySerial, true);`

3. **`boards/harness.c` `InitSerialPort`** — guarded RX config for **TX-only** ports:
   - Skip `GPIO_InitStructure.GPIO_Pin = u->Rx.Pin; GPIO_Init(u->Rx.Port, …)` +
     `GPIO_PinAFConfig(Rx…)` when `u->Rx.Pin == 0`.
   - Skip `USART_ITConfig(…, USART_IT_RXNE, ENABLE)` when `u->Rx.Pin == 0`.
   - **Backward-compatible**: every existing port has a non-zero RX pin, so nothing changes
     for other targets; only a port that deliberately declares RX=0 becomes TX-only.

### Why TX-only — the Blue LED owns PC11

UART4's RX pin is **PC11 = LED Blue** (`LEDPins[2]`). A full UART4 (TX+RX) init would
reconfigure PC11 as AF, killing the Blue LED. Since D8 is one-way, we declare RX=0 (a null
`ConnectDef { 0,0,0 }`) so the harness leaves PC11 under GPIO control — **Blue LED keeps
working**. (The initial attempt passed a scalar `0` for RX, which the C brace-initializer
took as only `Rx.Port`, shifting `9600` into `Rx.Pin` and misaligning the trailing
`UART4Config` — the explicit `{ 0, 0, 0 }` fixes the positional parse.)

### Pin assignment after the swap (UAVX F4 V3/V4)

| Pin | Before | After |
|---|---|---|
| **PA1** | soft-serial D8 TX (TIM5) | **Kill switch input** (`eArmedSel`) |
| **PC10** | Armed input | **FrSky D8 telemetry TX** (UART4, 9600, TX-only) |
| **PC11** | LED Blue | **LED Blue** (untouched) |
| **PC9** | eLanding | eLanding (untouched) |
| **PC12** | LED Green | LED Green (untouched) |

### Scope / blast damage

Changes are confined to **UAVX-prefixed targets** (uavxf4v3/v4 `.inc`), the **shared
`SendFrSkyTelemetry` dispatcher gate** (which now *fixes* the two UART4-FrSky boards instead
of no-op'ing), and the **backward-compatible** TX-only guard in `InitSerialPort`. All other
targets (SpeedyBee, Blueberry, Matek, FlyingRC, DevEBox, omnibus, discovery) are unchanged
in pin/port layout and compile clean.

### Verification

- All 6 targets build clean: `UAVXF4V3`, `UAVXF4V4`, `DEVEBOXF4`, `SPEEDYBEEF405WING`,
  `FLYINGRCF4WINGMINI`, `BLUEBERRYF405` (`python3 scripts/fc_build.py`).
- The UAVXF4V3 (Ecks) target — the airframe that flies — is the primary consumer and
  compiles clean.

### Notes / follow-ups

- Soft-serial is now **dormant** on UAVX v3/v4 (per design intent), not removed — the
  `eSoftSerial` port stays defined but unused so the codebase still references it.
- FrSky D8 polarity is **non-inverted** on the D4R telemetry pin (SBUS is the inverted one),
  so a plain non-inverting UART4 TX is correct.
- FrSky D8 still only works over a **FrSky RF link** (D4R/RX); CRSF/ELRS telemetry is a
  separate protocol needing a CRSF RX + a CRSF packer (future item, not this swap).

---

## Open items

- Cap = switch-arming remains NOT offered to non-UAVX boards; the Armed pin is only real
  on UAVX v3/v4 (**PA1** after the Part C move).
- External pull-up on the kill wire only if it runs long/near power; internal 40 kΩ is
  adequate for clean installs (fails safe).
- The old "UART4 needs PC10+PC11" caveat is superseded: with kill on PA1 and D8 TX-only
  on PC10, the Blue LED (PC11) is untouched.
- Software / field test pending: verify D8 arrives on the radio over UART4, and that the
  PA1 kill gate reads high (armed) with the switch absent.
