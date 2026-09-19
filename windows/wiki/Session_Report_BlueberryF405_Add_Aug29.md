# Session Report — BLUEBERRYF405 Board Port (Aug 29)

## Objective / context
Add a new FC board target to the quaternion fork: the **Blueberry F405**
(STMicro STM32F405RGT6, RCC 8 MHz HSE), a long-range fixed-wing FC with an
onboard VBAT divider (1K:20K), current shunt, and a 5V BEC. The board family
(Blueberry F4xx) carries the ICM-42605 dual-pad option (ICM-42688-P compatible),
an SPL06-001 barometer, and a newly-standardised motor/SERVO convention where
S7–S10 are *servo* group (TIM2/TIM12/TIM13) rather than more motor outputs —
the key difference driving the S10-on-TIM13 mapping below.

Pinout source: the iNav `BLUEBERRYF405` target (INAV 9.0, PR #11280).
Attribution: file header carries
`// Based originally on work by the iNavFlight project (src/main/target/BLUEBERRYF405).`
`// Adapted 2026-08-29 by UAVX Arm project.`

## Changes (what / where)

| File | Change |
|---|---|
| `UAVXArmQ/src/boards/targets/blueberryf405.inc` | **new** — full PinDef tree for the board |
| `UAVXArmQ/src/boards/targets/targets.inc` | added `#elif defined(BLUEBERRYF405)` include branch |
| `UAVXArmQ/src/UAVX.h` | `INCLUDE_ICM426XX` + `INCLUDE_SPL0601` now also selected for `BLUEBERRYF405` |
| `UAVXArmQ/src/boards/harness.h` | added `PWMOut_13_1` macro (TIM13, CH1, CCR1) |
| `UAVXArmQ/src/boards/harness.h` | **fixed latent bug** in `PWMOut_2_3`: `TIM_Channel_2` → `TIM_Channel_3` |
| `UAVXArmQ/Makefile` | `BOARDS` += `BLUEBERRYF405` |
| `UAVXArmQ/scripts/fc_build.py` | `BOARDS_ALL` += `"BLUEBERRYF405"` |

## Logical discourse (rationale / options / rejected alternatives)

### 1. Motor vs servo allocation — where the S7–S10 outputs live
The iNav target's motor list is S1..S6 (plus a mirrored S7/S8 already occupied),
and S9/S10 are servos. iNav doubles CW motors on S1/S2/S3 and CCW on S4/S5/S6 —
those S1/S2 (TIM8_CH4/CH3) are **not** in the default FixWing mixer, so here they
are left mapped but unassigned by default (matches iNav). S7/S8 = TIM2_CH2/CH1
(TIM2 has no DMA on this board), S9 = TIM12_CH1 and S10 = **TIM13_CH1** — all
servo-group. `CurrMaxPWMOutputs = 10` (driver cap `MAX_PWM_OUTPUTS`, harness.h:188).

**Rationale for the new `PWMOut_13_1` macro:** the board's `LED2/PW` pad doubles
as motor-4 output in iNav but the silent-crash FW convention uses **only** the 6
DMA-supported motor channels for ESCs; using the pad as S10 servo output is what
the iNav default configuration does with its `S10 = SERVO` (PA6/TIM13_CH1).
TIM13 is free in this firmware: no scheduler claim (SysTick-based), no other
target uses it, `InitTIM_RCC_APB` (harness.c:129-130) already clocks APB1 TIM13,
and `InitPWMPin` already halves the prescaler for non-APB2 timers
(`(pwmprescaler >> 1) - 1`, harness.c:500-501) so TIM13 CH1 runs the same
1 MHz tick → same PWM_PERIOD_DIGITAL/ANALOG frame rate as the TIM2/TIM12 servo
group. TIM13 has no MOE/BDTR, so `TIM_CtrlPWMOutputs` is a no-op there — same
situation as the long-proven TIM12 servo path, so no change needed.

**Rejected:** mapping S10 to a TIM1/TIM8 channel — an 8th/9th motor as in iNav
needs an extra driver chunk; the frame's FW airfames want 2-4 servos, not more
DMA motors. Keeping S10 on TIM13 avoids touching the DMA scheduler.

### 2. `PWMOut_2_3` latent bug (fixed in passing)
The macro read `{true,TIM2,TIM_Channel_2,0,&(TIM2->CCR3),GPIO_AF_TIM2}` — Channel
and CCR mismatch. `InitPWMPin` switches on `u->Timer.Channel`: Channel_2 would
call `TIM_OC2Init` (enables CC2E) while all duty writes go to `CCR3`. On PB10
(TIM2_CH3, S6 of FLYINGRCF4WINGMINI and now BLUEBERRYF405) the compare output on
CH3 was therefore never enabled → the pad would not switch. Channel aligned to
CCR3. Compile-compatible, so all six targets rebuild unaffected.

### 3. IMU on SPI1 — dual ICM-42688-P / ICM-42605 support
`busDev[eImuSel] = icm42688IMU` (CS = PC14, SPI1, "lib CLI"). iNav carries both
parts on the same footprint; the driver's whoami check accepts the 42688-P
(0x47) and, via the shared ICM426XX driver path the fork already has, the
42605 (0x42). No BDM. MISO note: the totality of SPI1's MISO (PB4) is a
standard PinDef on this board even though iNav "CLI on SPI" — the firmware uses
SPI1 only for the IMU, so the CLI pin is not brought up.

### 4. Mount / sensor quadrant — the mapping reconstruction
iNav's alignment for the 42605 on this board is `IMU_ICM42605_ALIGN =
CW270_DEG_FLIP` (270° rotation, then flip about X) giving gyro
`(B[Y], B[X], −B[Z])`. The fork maps IMU alignments as `SensorQuadrant` ×
`SensorFlip` through the (roll[Y],pitch[X],yaw[Z]) frame, so the equivalent pair
here is **quadrant 3 + flip**: gyro reads `(+gy, +gx, −gz)`, accel `(+ay, +ax)`
with Acc Z left in the SDK convention (signed inverted from iNav's z-axis
definition). `Imu1G = 2048.0f` matches the driver's ±16 g range; gyro scale
0.001064225154 rad/s/bit = ±2000 dps.

**Open bench item (flagged in-file):** level board must read `Acc[Z] = −1 g`.
If it reads +1 g use `SensorFlip = false` (quadrant 3 unchanged) and re-check
attitude. The symmetry of the iNav rotation means quadrant 3 reproduces the
iNav angular rates exactly; the residual doubt is only in the Z sign convention.

### 5. Baro SPL0601, mag external
SPL0601 on **I2C1** (PB8/PB7) — the driver's native bus, matching iNav (which
also carries DPS310/BMP280/MS5611; we keep the fork's SPL06-only build). Mag:
none onboard; `eMag2Sel = hmc5xxxMag` probe on the same I2C1 pads lets an
external QMC5883/HMC5883-compatible compass anchor yaw (deliberately a probe,
not a hard requirement — mags presence is runtime-detected).

### 6. Serial layout (board-owned connectors)
- USB VCP first (telemetry/MSP).
- USART2 = RC input (CRSF/SBUS) — Q3 unbinding from a softserial, PB6/PB7
  conflict avoided by keeping the CRSF decode on a real UART.
- USART3 = GPS (matches BLUEBERRYF435WING's GPS UART habit).
- UART4 = FrSky S.Port push (57600 8E2 parity ─ S.Port spec).
- USART1 + UART5 = spares. **eSoftSerial disabled**: its default TX is PA2,
  which is USART2 TX (RC) — a hard clash, so `SoftSerialTxPin = { false }`.
- UART6 (PC6/PC7) exists on silicon but the framework's serial slots end at
  `eUart5` + `eSoftSerial`, so it is deliberately not mapped (comment in-file).

### 7. Analog (ADC1 DMA group, all 6 channels in one scan)
VBAT on PC4 (CH14, 1K:20K divider), Current on PC5 (CH15, onboard shunt),
RSSI on PB0 (CH8, framing-only), Airspeed on PC0 (CH10, analog pitot).
`CurrBattVoltsAnalogSel = eExternalVoltsAnalogSel` — the volts come off the
external pad, not a rail.

### 8. LEDs / GPIO
Blue = PA14, Green = PA13 (shared SWD pads — iNav's default), both high-enabled
(`ledsLowOn = false`). Beeper PB9 inverted (`beeperLowOn = true`); PINIO1/PA4
(VTX power), PINIO2/PB5 out. The former LED2/PW pad (PA6) becomes S10's PWM
output, so the yellow LED stays off.

## Verification status
- `BOARD=BLUEBERRYF405 python3 scripts/fc_build.py` — **build OK**, 228500 B
  flash artifact `obj/BLUEBERRYF405/BLUEBERRYF405Q_r0.bin` (rev 0, svn-less).
- Full board sweep (`python3 scripts/fc_build.py`, all 6 boards): UAVXF4V3,
  UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI, BLUEBERRYF405 —
  **all OK** after the harness.h macro additions/fix.
- GCS: no board-name list to update (board names are tag-63 revision strings),
  so no `uavx-python` change; nothing to py_compile.
- No new params added — no ParamTable/GCS `PARAM_LIMITS` sync needed.

## Open items / bench plan
1. **IMU mount confirmation** — level board must read Acc Z = −1 g; else toggle
   `SensorFlip` per the file comment and re-verify; then a 90° yaw-bench check of
   the attitude heading.
2. First boot preflight: `UseMag = off` (gyro-only yaw, per Mag Gating practice),
   confirm S10 servo frame rate visually while configuring an airframe.
3. If the production board ships the ICM-42688-P, confirm whoami — the dual
   footprint is already covered by the same `icm42688IMU` driver select.
4. Wire both airframe-type sliders (`AFType` fixed-wing) — FW-specific airframe
   params come from the `.af`, not this port.