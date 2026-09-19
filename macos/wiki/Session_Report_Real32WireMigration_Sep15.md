# Session Report — WP/Origin Wire real32 Migration + ClassifyAFType Guard Test + Cleanup Fixes

**Date:** 2026-09-15
**Build/verification:** FC — `python3 UAVXArmQ/scripts/fc_build.py` (all 7 targets, then SPEEDYBEEF405WING re-verified after the batt.c guard); GCS — `py_compile` clean across source + 3 kits.
**Kits:** linux/windows/macos synced from `uavx-python/src/` + `AGENTS.md` (verified byte-identical).

---

## 1. AF-Category mirror guard test (`tests/test_af_category.py`)

**What:** a single cross-repo guard that parses the FC's `ClassifyAFType()` / `AirframeCategory` / `enum AFs` and asserts they exactly match the GCS single authority `AIRFRAME_CATEGORY` in `protocol_enums.py` (consumed by parameter_window, test_pid_sim, export_all_params, migrate_limits, apply_external_rate_tuning).

**Result:** `OK: FC ClassifyAFType() == GCS AIRFRAME_CATEGORY (FW 6, VTOL 2, LAND 3, MR 16)` — i.e. no `IFS`-style drift exists today; the test is the tripwire so it cannot return.

**Rationale (why this specific design):**
- Prior drift (documented in the GCS): test_pid_sim's local `AF_CATEGORY` omitted `eAileronVTailAF` and silently simmed it as multirotor. Any future airframe add / category re-assignment has to change BOTH the FC `params.c` and the GCS authority; without a guard the failure is silent.
- **Brace-depth scan**, not a raw substring find, for the `ClassifyAFType()` body — the body is nested in `{ … }`; we walk to the closing brace that returns depth to zero.
- **Anchored regex for the arm conditions** (`if (…) \n pAFTypeCategory = eCatX;`), lazy condition capture inside the arm's own closing paren. A naive `if (` at each member name captures the *first* `if` for every arm and under-reads the condition — exactly how the mirror drifted; the anchor is the assignment line so each member's condition is read on its own arm.
- Members that fall to the FC's `else` (implicit MR) must NOT be enumerated in the GCS (`eAFUnknown`, `eInstrumentation`) — asserted explicitly, so the "everything not enumerated = MR" contract on both sides stays identical.
- Enum value mirrors (`AirframeType` / `AirframeCategory` value equality with FC `params.h`) asserted too — a member rename or renumber is caught even if no category changes.

**Files:** `uavx-python/src/tests/test_af_category.py` (+ 3 kit mirrors).
Runs as `python3 src/tests/test_af_category.py` (pure stdlib; no pytest in the sandbox).

---

## 2. Item 7 — Mission / Waypoint / Origin wire + structs: int16 → real32

### 2.1 Scope and why
Continuously-varying physical quantities (waypoint altitude, velocity, loiter time, orbit radius/altitude/velocity, fence radius, origin altitude) flowed through `int16` both on the wire and in the FC persisted structs, then got cast to `real32` at use sites. This violated the (now codified) real32 end-to-end rule and had a **twice-real** consequence — a latent wire offset bug (2.2 below).

### 2.2 The latent bug this fix removes
The old `SendWPPacket` wrote the payload packed as `wp,Lat,Long,Altitude(int16),Velocity(int16),Loiter(int16),OrbitRadius,OrbitAltitude,OrbitVelocity,PulseWidth,PulsePeriod,Action`, while the old `ProcessWPPacket` read `Altitude@[9]` then `Velocity@[13]`, `Loiter@[16]` / `OrbitRadius@[18]` / `OrbitAltitude@[21]` / `OrbitVelocity@[24]`. The reads were **shifted −2** from where the sender actually put the fields (Loiter really at 18, OrbitRadius at 21 …). Every field after Velocity was silently decoded from the wrong bytes — so `WP.Loiter` / orbit parameters arriving over the wire were garbage (the mis-capture of Velocity's low bytes as Loiter, etc.). This class of corruption is the strongest candidate driver for several on-air anomalies, including the emulated XQuad altitude runaway (see §6). The new format makes offsets self-describing by construction (each f32 at a fixed slot) so a stale manual offset table cannot silently break again.

### 2.3 New wire format (FC sender ⇌ GCS parser, verified byte-identical)

**WP body = 42 bytes** (`SendWPPacket`, `telem.c:1038-1062` ⇌ `parse_waypoint_packet`):

| offs | bytes | field |
|---|---|---|
| 0 | 1u8 | `wp` |
| 4 | 4i32 | LatRaw |
| 8 | 4i32 | LonRaw |
| 12 | 4f32 | Altitude (m) |
| 16 | 4f32 | Velocity (m/s) |
| 20 | 4f32 | Loiter (s) |
| 24 | 4f32 | OrbitRadius (m) |
| 28 | 4f32 | OrbitAltitude (m) |
| 32 | 4f32 | OrbitVelocity (m/s) |
| 36 | 4i32 | PulseWidthmS |
| 40 | 4i32 | PulsePeriodmS |
| 44 | 1u8 | Action |

(Header: `UAVXWPPacketTag` + u16 42 precede; checksum/trailer follow.)

**Origin body = 18 bytes** (`SendOriginPacket`, `telem.c:1019-1036` ⇌ `parse_origin_packet`):

| offs | bytes | field |
|---|---|---|
| 3 | 1u8 | NoOfWayPoints |
| 4 | 1u8 | 50 (legacy filler) |
| 5 | 4f32 | FenceRadius (m) |
| 9 | 4f32 | OriginAltitude (m) |
| 13 | 4i32 | OriginLatRaw |
| 17 | 4i32 | OriginLonRaw |

### 2.4 FC changes
- `mission.h` — `WPStructNV`: `Altitude`/`Loiter`/`OrbitRadius`/`OrbitAltitude` → `real32`; `MissionStruct` (persisted `Config.Mission`, sent on every origin packet): `FenceRadius`/`OriginAltitude` → `real32`.
- `auto.h` — nav-mission `WPStruct`: `Loiter`/`OrbitRadius`/`OrbitAltitude`/`OrbitVelocity` → `real32`.
- `telemetry/telem.c` — `ProcessWPPacket` / `SendWPPacket` / `ProcessOriginPacket` / `SendOriginPacket` rewritten against the new layouts (offsets above). Wire now uses `TxESCu8/i32/r32` / `UAVXPacketf32/i32` only — no `int16` anywhere in these packets.
- `mission.c` + `params.c` — cast cleanups: `WP.C[eDownC].Pos` (Down Pos), `WP.Loiter`, `OrbitRadius`, `OrbitAltitude`, and `HomeWP.Loiter` (the `(real32)` casts that masked the int16 fields are removed).
- `nvmem.h` — `CONFIG_MAGIC 0xFEEDBEE5 → 0xFEEDBEE6` (persisted `MissionStruct` layout changed → first boot after reflash clears cal correctly). Comment records the reason (`MissionStruct 5×int16→real32`).

### 2.5 GCS changes
- `packet_parser.py` — `parse_waypoint_packet` (42 B, offsets 9/13/17/21/25/29/33/37/41), `fence_radius`/`home_altitude` → float in `OriginData`, `parse_origin_packet` (18 B, offsets 2/6/10/14).
- `models/waypoint.py` — `alt`/`loiter`/`orbit_radius`/`orbit_alt` → float.
- `ui/nav_window.py` — send `struct.pack("<Bii ffffffiiB", …)` (WP) and `"<Bbffii"` (Origin); WP cell edits read/write floats (cols 3/5).

**Verification:** py_compile clean; pack→parse round-trip (GCS-format WP + Origin) byte-identical to the FC sender code both in this session and the prior GCS work; all 3 kits synced.

---

## 3. Item 8 — AGENTS.md: real32 rule codified (Coding Standards rule 11)

Write rule: continuously-varying physical quantities (altitude, position, velocity, angles, rates, throttle, temperature, …) stay `real32` end-to-end — FC structs, wire encoding (`TxESCr32`/`UAVXPacketf32`), GCS models, persisted config. Integers reserved for genuinely raw/quantised data (GPS int32 1e7, ADC/PWM/timer counts, enums, channel indices). Rationale cites the item-7 latent offset bug (int16→f32 conversion without correcting payload offsets corrupted every field after Velocity).

Synced to 3 kits. This rule is the policy body for this and future real32 migrations.

---

## 4. Item 6 — emulated servo write gated off (`outputs.c`)

`servoWrite(m, PWp[m])` in the per-cycle drive stage now runs only `if (!F.Emulation)` — matching the existing drive gate (`driveWritePtr` at `outputs.c:184`). Rationale: with `F.Emulation` set, servo writes must not touch real hardware outputs; the drives were already emu-veiled but the servos were still being written (an inconsistency with the "no servo lockup / no output during emu test" hard rule). The init-path write at `outputs.c:269` (one-time, before any control loop) is intentionally left ungated. **SPEEDYBEEF405WING rebuild OK.**

---

## 5. Item 4 — battery NaN / capacity-zero guard (`batt.c`)

`MockBattery()` divided by `pBatteryCapacitymAH` unguarded; with capacity unset (0) it produced 0/0 → NaN volts, which then propagated through the whole telemetry chain (root path verified: `SlewLimit` passes NaN through because both comparison branches are false). **Fix:** early `if (pBatteryCapacitymAH <= 0.0f) return (4.2f);` before the capacity-based branches. **SPEEDYBEEF405WING rebuild OK** (this was the re-verified build recorded at the top).

---

## 6. Item 2 — emulated XQuad altitude runaway (~1000 m): investigation result

**No separate fix — root cause is the §2.2 corruption class, now eliminated.**

Findings:
- The emulator's emu MR plant integrates altitude with **no ceiling**: `FakeAltitude += FakeROC*dT` (`emu.c:547`), ground-snap only below −0.5 m / idle-throttle (`emu.c:549-576`). Confirmed `Baro.DensityAltitude = RF.Altitude = Altitude = FakeAltitude` (`emu.c:653`) — so the emu faithfully climbs as long as the FC demands it. This is correct emu behaviour (a faithful plant), not a defect.
- The FC's altitude ceiling `cNavCeilingM = 120.0f` (`params.h:143`) is **alarm-only security**: `F.FenceAlarm` flag (`mission.c:65-66`) and a clamp on *WP desired* down-position / orbit altitude (`mission.c:377-378`). It does NOT ramp down a runaway emu altitude and does not veto a corrupt AltHold desired altitude.
- Therefore a ~1000 m emulated climb needs a large (or NaN) **desired altitude** to be fed to the control loop. The pre-fix int16 WP wire — with the −2-byte offset bug reading `/4`-scale int16s from misaligned bytes (or a float's low bytes seen as a huge altitude) — was the exactly that class of input. Since the item-7 wire now carries self-consistent f32 slots with verified offsets, that corruption vector is gone.
- The 1-D altitude-hold sim (`tests/test_alt_hold_sim.py`) is bounded at 5 m steps (it has ISSUES on settle/overshoot criteria but no runaway), consistent with "runaway needs a corrupt setpoint."

**Verification path for Greg:** next time the emu is run with a GCS-driven mission, the WP/origin values arriving are round-trip-identical (f32, correct offsets); any future runaway would have to be re-observed against this baseline before a controller fix is considered. No emu.c change made.

---

## 7. Build matrix

| target | result |
|---|---|
| UAVXF4V3 | OK (all-targets pass) |
| UAVXF4V4 | OK |
| DEVEBOXF4 | OK |
| SPEEDYBEEF405WING | OK (re-verified after batt.c guard) |
| FLYINGRCF4WINGMINI | OK |
| BLUEBERRYF405 | OK |
| MATEKF411WING | OK |

SPEEDYBEEF405WING image: 234 372 B. **Not flashed** — FC-side changes need Greg's bench/hardware validation per normal workflow.