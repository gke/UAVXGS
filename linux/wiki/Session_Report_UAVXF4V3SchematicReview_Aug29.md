# UAVX F4V3 Flight Controller — Schematic Review & Session Handoff

Date: 2026-08-29
Author: Assistant session (opencode / big-pickle)
Status: **REVIEW IN PROGRESS — pass to another session for completion**

> **Purpose of this file:** This is a **handoff document**. The reviewing session hit a
> topic/side-track (see §0) and stopped before finishing. Another session (or the user) can
> pick this up and continue exactly where work stopped, without re-deriving history.
> All hard facts extracted from the schematic are recorded here so they don't need re-parsing.

---

## 0. Session path — what happened, so you don't repeat it

1. User asked for a schematic review of a board, initially identified as **"UAVXArm 20120727"**
   (`Hardware/UAVXArm Board/Boards/UAVXArm 20120727/uavxarm.sch`). Assistant did a full parse.
2. User said "it's our UAVXF4V3, full review please." Assistant noted the schematic said **STM32F103**.
3. User pointed at **`uavxarm-nano production 20140608`** instead. That folder contains **two boards**:
   - `uavxarm.sch` / `.brd` = the **flight controller** (fully parsed).
   - `Dc-motor/uavx-nano.sch` / `.brd` = a **PWM→DC motor converter** (no MCU) — NOT yet reviewed in detail.
4. Assistant scanned the whole `Boards/` tree for F4/F405 and found **no schematic carries an STM32F4 part
   anywhere** — all are labeled `STM32F103RxT6`.
5. User clarified: **the 103 label is stale; the board is really an STM32F405RG** (the schematic symbol/library
   was never updated after the silicon decision). Firmware `UAVXArmQ/` is Cortex-M4 and matches the F405RG.
6. Assistant wrote a corrected review treating the part as **F405RG**, then was told the session had
   "gone sideways on the topic" → **write a full-context handoff report.**

**Net effect:** the *flight controller* part of the review is substantially done (facts below). The
*Dc-motor converter board* review is **NOT done**. Remaining open questions are in §7.

---

## 1. File identities (do not re-derive)

Root of all hardware: `/home/gke/Documents/Flight/Hardware/UAVXArm Board/Boards/`

| Board | File(s) | MCU in symbol | Real silicon (per user) | Parsed? |
|---|---|---|---|---|
| UAVXArm "20120727" | `UAVXArm 20120727/uavxarm.{sch,brd}` | STM32F103RxT6 | unknown (older) | ✅ (earlier, superseded) |
| **Nano production FC** | `uavxarm-nano production 20140608/uavxarm.{sch,brd}` | STM32F103RxT6 | **STM32F405RG** | ✅ **fully** |
| PWM→DC converter | `uavxarm-nano production 20140608/Dc-motor/uavx-nano.{sch,brd}` (+ `v2/`) | — (no MCU) | — | ❌ **not reviewed** |

Other dirs in `Boards/`: `uavxarm-nano 20140324`, `uavxarm_nano 20140325`, `uavxarm_nano_spi_20150104/21`
("v4"), `uavxarm-lite 20{12,14}*`, `uavxem 2011*`, `uavx-stm32f2x 2011`, `Ken Pics` (many copies).
**The `uavx_nano_spi_2015` "v4" boards are also STM32F103-labeled** (checked MCU deviceset = STM32F10XRXT6).

All schematics are **EAGLE ASCII/XML** (`<?xml ... <eagle version=...>`). Older = 6.4, newer = 9.4.2.
**All parseable as text** — a reviewer can read the netlist directly (see §6 for the parse recipe).
Image files (`top.png`, `*.jpg`) are **not** viewable by this model (no image input) — rely on the XML, which is authoritative anyway.

---

## 2. Hard facts — Nano Production FC (`uavxarm.sch`, EAGLE 6.4, 9423 lines, 1 sheet)

### 2a. System-on-board inventory (BOM)
- **MCU:** STM32F405RG (LQFP64) — symbol mislabeled STM32F103RxT6.
- **Oscillator:** `Y1` 8 MHz (CTX853CT-ND) + `R1` 1M across + `C1`/`C2` 18pF load caps.
- **IMU:** `MPU6050` (accel+gyro) on I2C; `INT→PC14`; C15 REGOUT 0.1µF, C16 CPOUT 2.2nF, C17 VDD.
- **Baro:** `U$2` = `MS5611` on I2C.
- **Mag:** `HMC5883L` on **main I2C** (this is the improvement vs the 20120727 board, which had an SJ bus-select).
  `DRDY→PC15`; SETC/SETP via C13 0.22µF; C14 4.7µF.
- **EEPROM:** `IC2` = 24\* I2C serial EEPROM (8-pin, A0/A1/A2/WP→GND).
- **LDO:** `IC1` = `LT1763CS8` low-noise 500mA 3.3V; BYP cap C18 0.01µF; input C19/C22 10µF; output C23 10µF.
- **Level shift:** `Q1` = `BSS138` N-MOS, USART_TX → 5V **inverting** line ("FRY-OP"), R15 1k gate, R16 10k pull-up.
- **Beeper driver:** `U$1` N-MOS; PA12→R11 2.2k→gate, R17 10k gate pull-down; drain=BEEPER→X1.p2; supply via R14 240Ω.
- **LEDs:** LED1–LED4 on PB3/PB4/PC11/PC12.
- **100V cap:** `C7` 0.1µF/100V on a rail (verify node, §7).
- **Connectors:** X1–X7 (Molex 53047 SPI header), JP1 (moto), JP2(5V), JP3(GND), SPEKTRUM-RX (B3B-ZR).
- **Solder jumper:** SJ1 = LDO input source select (VCC_IN vs 5V). Default **open** (`SOLDERJUMPER NO`).

### 2b. Power topology
```
Battery → VCC_IN (LSP1) ── C19/C22 10µF ──┐
                                          ├─[SJ1]─► IC1 LT1763CS8 ──► 3.3V (C23 10µF, C18 BYP)
External 5V (JP2 ×6, X6.p9) ── 5V rail ───┘
```
- 3.3V rail feeds: MCU VDD1-4, VDDA, VBAT; MPU VDD+VIO; MS5611; HMC (VDD+VDDIO); EEPROM; LEDs; headers X1/X5/X7.
- **Battery sensing:** `BATT_VOLT` = R12(10k)–R13(2.2k) divider + C10 0.1µF → **ADC PC2**.
- **No on-board current sense** (no shunt/amp) — current lives on the external converter/PD board.
- **VBAT tied to 3.3V**, no 32.768kHz RTC crystal → RTC disabled. Acceptable for FC.

### 2c. Connector pin maps (gate = pin number on Molex multi-gate symbols)
- **X1** (4p): p1=N$1(beeper supply via R14), p2=BEEPER, p3=BOOT0, p4=3.3V  → boot/programming.
- **X3** (4p): p1=AUX_1, p2=AUX_2, p3=AUX_3, p4=GND  (PC6/7/8).
- **X4** (4p): p1=LANDING, p2=GND, p3=GND, p4=ARMING  (PC9/10).
- **X5** (6p): p1=GND, p2=3.3V, p3=I2C_SCL, p4=I2C_SDA, p5=5V, p6=RANGE_FD(PC0) → I2C + rangefinder expansion.
- **X6** (10p): p1=PB1, p2=PB0, p3=PA7, p4=PA6, **p5=PA3(=N$13, RC3 + SPEKTRUM-RX.3)**, p6=PA2, p7=PA1,
  p8=PA0(RC0), p9=5V, p10=GND  → RC / analog inputs.
- **X7** (5p): p1=USART_TX(PA9), p2=USART_RX(PA10), p3=GND, p4=3.3V, p5=FRY-OP(inverted level TX) → telemetry.
- **JP1** (6p): p1=M6, p2=M5, p3=M4, p4=M3, p5=M2, p6=M1 (PWM signals to external ESCs).
- **JP2** (6p): all 5V. **JP3** (6p): all GND.
- **SPEKTRUM-RX** (B3B-ZR): pin3 data → **PA3 (N$13)** — same pin as RC ch3.

### 2d. MCU pin map (as F405RG)
```
RC/analog: PA0..PA7, PB0, PB1        → X6  (PA3 also = Spektrum-RX data ⚠)
RC0        PA0 (SJ4/boot test point) → X6.p8
Motor PWM: PA8(TIM1_CH1)=M6, PA11(TIM1_CH4/USB_DM)=M5 ⚠,
           PB6(TIM4_CH1)=M4, PB7(TIM4_CH2)=M3, PB8(TIM4_CH3)=M2, PB9(TIM4_CH4)=M1
Telemetry: PA9=USART_TX, PA10=USART_RX → X7
Beeper:    PA12(TIM1_CH2/USB_DP) → R11 → U$1.G ⚠ (USB_DP)
I2C:       PB10=SCL, PB11=SDA  (MPU/MS5611/HMC/EEPROM + X5)
3-w SPI:   PB12=LIS_CS, PB13=LIS_CLK, PB14=LIS_SDA  (GPIO bit-bang "LIS")
LEDs:      PB3,PB4,PC11,PC12
Sense:     PC2=BATT_VOLT (ADC)
Aux:       PC6,PC7,PC8 → X3 ; PC9=LANDING,PC10=ARMING → X4 ; PC0=RANGE_FD → X5
Ints:      PC14=MPU.INT, PC15=HMC.DRDY
Crystal:   PD0/PD1 (OSC)
Boot:      BOOT0 via R7 + X1.p3
```

---

## 3. Findings on the FC (F405RG-aware)

**Correctness / risk (FIX or CONFIRM):**
1. **Schematic symbol labeled STM32F103, real silicon F405RG** → update EAGLE symbol/library/part name. Document truth gap.
2. **No NRST capacitor** — only R8(4.7k) pull-up to 3.3V. Add ~100nF NRST→GND for glitch immunity.
3. **PA3 dual use = RC ch3 AND Spektrum-sat data** (both on net `N$13`). Real IO conflict → confirm only one is used (firmware must not enable both).
4. **PA11/PA12 = USB_DM/USB_DP on F405**; used for M5 and beeper-gate. **Ensure USB/DFU disabled in firmware**, else pin glitches at boot/USB-enum.
5. **No ESD/TVS/ferrite/polyfuse on ANY external connector** — top robustness gap for an FC on a vibrating airframe (ESCs/servos + RF link).
6. **BSS138 path is an INVERTING 3.3→5V level shift** ("FRY-OP"). Confirm polarity matches the FrSky D8 telemetry link used in `UAVXArmQ/`.

**Design notes (not defects):**
7. 1k I2C pull-ups with 4 loads + exposed X5 header → if 400kHz rise time marginal, use ~4.7k.
8. VBAT=3.3V, no RTC crystal → RTC disabled, acceptable.
9. PB13/PB14 also SPI2 pins; used as GPIO bit-bang for "LIS" — note only.
10. Quiet-ground split on VSS via 0Ω straps + dedicated caps (C30/C31 on VSS_1/VSS_2) — good practice.
11. C7 0.1µF/100V placement — verify node (a 100V ceramic suggests battery-side; if on 5V/3.3V it's odd).

**Improvements vs the older `UAVXArm 20120727` board:**
- HMC5883L directly on main I2C (no solder-jumper bus-select) — matches firmware better.
- Proper 10µF bulk decoupling on 3.3V output + inputs.
- Added Spektrum sat input and a level-shifted FrSky TX.

---

## 4. Extra facts from the older 20120727 board (for reference, superseded)

Same sensor suite (MPU6050/MS5611/HMC5883L + 24C EEPROM + LT1763 + F103 label), but:
- HMC5883L was behind an **SJ1/SJ2 solder-jumper bus selector** (main I2C vs MPU-AUX pass-through), default = AUX side.
- No on-board current sense (BATT_CURRENT/BATT_VOLT exit raw on SL9).
- No reset cap (same issue).
- Motor/RC pins were PA0–PA7/PB0/PB1 via SL1; PA11/PA12 to SL4.
Not the target board — informational only.

---

## 5. Not-yet-reviewed: PWM→DC motor converter board

- Files: `uavxarm-nano production 20140608/Dc-motor/uavx-nano.{sch,brd}` (+ `v2/`), EAGLE 6.4.
- Parts seen (from a quick scan): **6× N-MOSFET**, **6× BAT54C**, **1× TPS6300 buck-boost + inductor**,
  MOSFET 1k/0.1µF/2.2µF parts, M06/M08/LSP10 connectors, 5V rail. **No MCU** → PWM-in / motor-out power stage.
- **Review NOT done.** Next session should parse this netlist (recipe in §6) and check: PWM input conditioning,
  gate drive, MOSFET SOA with the DC motor load, flyback/BAT54C diode adequacy, TPS6300 buck-boost regulation/
  inductor sizing, current sense, thermal/ESD, and the polarity/inversion relationship between this board's
  PWM inputs and the FC's M1–M6 signal outputs.

---

## 6. How to parse these schematics (reusable recipe)

```bash
cd "/home/gke/Documents/Flight/Hardware/UAVXArm Board/Boards/<board>/"
python3 - <<'EOF'
import re
data=open('uavxarm.sch',encoding='utf-8').read()
# Parts:
for m in re.finditer(r'<part name="([^"]+)" library="([^"]+)"(?:[^>]*?)(?: value="([^"]*)")?(?:[^>]*?)deviceset="([^"]+)"(?:[^>]*?)(?: device="([^"]*)")?', data):
    pass
# Nets:
for name,body in re.findall(r'<net name="([^"]+)"[^>]*>(.*?)</net>', data, re.S):
    pins=re.findall(r'<pinref part="([^"]+)" gate="(\-?\d+|[A-Za-z0-9]+)" pin="([^"]+)"/>', body)
    print(name, pins)
EOF
```
**Gotchas:**
- Molex 53?-N connectors use **multi-gate symbols** (`gate="-1".."-N"`); the **gate = physical pin number**.
  A naive parse printing `X6.S` (pin name only) collapses pins — always capture `gate` → that is the pad/pin.
- `$` in net names must be `re.escape`'d.
- Values are inline `value="..."` attributes on `<part>` for some libs; other libs encode size in the deviceset
  (e.g. `22UF-6.3V-20%(0805)`).
- Image `/PNG/JPG` are not viewable; the XML netlist is authoritative and sufficient for electrical review.

---

## 7. OPEN ITEMS for the next session (handoff checklist)

1. **Review the Dc-motor converter board** netlist (§5) — the incomplete half of this task.
2. **Confirm PA3 Spektrum-vs-RC3 decision** with the user/firmware (netlist says both share N$13).
3. **Confirm USB (PA11/PA12) stays disabled in firmware** — else M5/beeper pins glitch.
4. **Decide/implement NRST cap addition** (100nF) if the board is still being spun.
5. **Confirm the BSS138 TX polarity** matches the FrSky D8 link polarity used in `UAVXArmQ/` (inverting opto/frsky convention).
6. **Verify C7 100V cap node** on the schematic (odd value for a 5V/3.3V rail).
7. **Update the EAGLE symbol from STM32F103RxT6 → STM32F405RG** (project-truth cleanup) if a board revision is planned.

---

## Files
- This report: `wiki/Session_Report_UAVXF4V3SchematicReview_Aug29.md`
- FC source parsed: `Hardware/UAVXArm Board/Boards/uavxarm-nano production 20140608/uavxarm.sch`
- Converter (not yet reviewed): `Hardware/UAVXArm Board/Boards/uavxarm-nano production 20140608/Dc-motor/uavx-nano.sch`
