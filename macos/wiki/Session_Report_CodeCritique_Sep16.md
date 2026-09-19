# Code Critique Sweep — telemetry/parameter/battery/Nav hardening (Sep 16)

**Date:** 2026-09-16
**Status:** FIXES APPLIED + BUILDS CLEAN
**Scope:** a volunteer critique pass over `UAVXArmQ/src/` (FC) and
`uavx-python/src/` (GCS), verifying agent-raised concerns and applying fixes
only where behaviour was provably wrong. Items that are *flight-visible* or
*design-decisions* are **reported, not changed solo** (see §4).

---

## 1. Fixes applied (FC — `UAVXArmQ/src/`)

### 1.1 telem.c — `ProcessWPPacket` WP index bounds (telem.c:984-1008)
**Concern:** packet-controlled `wp = UAVXPacket[4]` indexed `M->WP[wp]` without a
bounds check. `NAV_MAX_WAYPOINTS = 64`; a crafted/errored packet with `wp ≥ 64`
is an OOB read/write into the struct following `WP[]`.

**Fix:** the write path is now wrapped in `if (wp < NAV_MAX_WAYPOINTS) { ... }`
(pass path in the positive `if`, **single logical exit — no early return**,
per the coding standard). Out-of-range writes are dropped silently (WP writes
have never had an ACK, so dropping is consistent with the existing transport).

### 1.2 telem.c — `ProcessOriginPacket` NoOfWayPoints clamp (telem.c:1015-1018)
**Concern:** `NewNavMission.NoOfWayPoints = UAVXPacket[3]` — an unclamped
packet-controlled `uint8` (0..255) into an `int8`. Values 65..127 survived the
sign wrap as positive `int8` and then drove the `RefreshFence()` and mission
loops to **OOB-read** `Config.Mission.WP[wp]` past the 64-entry array.

**Fix:** clamped to `NAV_MAX_WAYPOINTS` before the store. (Values ≥128 were
already safe — they wrapped negative and loop guards failed `wp < n`.)

### 1.3 telem.c — `PacketsReceived[]` tag-overrun guard (telem.c:1162)
**Concern:** `PacketsReceived[RxPacketTag]++` where `PacketsReceived[]` is
`[128]` but `RxPacketTag` is `uint8` (0..255). Any tag ≥128 (`TxTag` 65-bit
range of 0x80..0xFF) is an **OOB write**. The dispatch switch (`switch(UAVXPacket[1])`)
only handles the known tags (<~76), but the counting line ran **before** the
switch and indexed with the raw tag.

**Fix:** `if (RxPacketTag < 128) { PacketsReceived[RxPacketTag]++; }` — the count
is diagnostic-only, so dropping out-of-array tags loses nothing of value.

### 1.4 batt.c — three parameter-clamp / dead-branch defects
- **`pBatteryAlarmPct` is stored as a raw FRACTION** (`ParamTable` default
  `0.15f`; GCS `PARAM_DISPLAY_MULT[119]=100`, `PARAM_LIMITS[119]=(0.0,0.5)`).
  The LowBatt test was `BatteryChargeUsedmAH > pBatteryCapacitymAH *
  pBatteryAlarmPct * 0.01f` — a **double-divide** of the 15% default → battery
  alarm fired at 0.15% capacity used. **Fixed:** minus the bogus `*0.01f`
  (batt.c:158). The GCS shows 0.15 (display mult 100) yet the FC semantics is a
  true 0.15 fraction — **this is a latent GCS/FC display-semantics trap**: the
  GCS `%` spinner writes `0.15f` *raw* through the normal write path, which the
  FC now consumes correctly. (The misleading `real32 pBatteryAlarmPct = 15;`
  init at params.c:92 is overwritten by `UseDefaultParametersEx` before use.)
- **`MockBattery` 95% taper was unreachable** (batt.c:65-71): a literal `95.0f`
  (instead of `0.95f`) in `BatteryChargeUsedmAH < (pBatteryCapacitymAH * 95.0f)`
  → the tail taper branch never fired while true mock-battery civil behaviour
  should. **Fixed:** `* 0.95f` (+ the two consistent `- 3.0f * (... - 0.95f)`
  terms). This is emulation-only code (no physical effect on real flights).
- **`VOLTS_SCALE` trailing `= 18.3` removed** (batt.c:24): the macro was
  `((3.3f*(10.0f+2.2f))/2.2f) = 18.3` — a trailing literal after the parenthesised
  expression, i.e. a stray default value from a half-refactor. It was compiled
  only since `adjust for the onboard regulator` branch is active, but the
  `onboard` branch (`= 18.3`) is **dead** (`VOLT_MEASUREMENT_ONBOARD` never
  defined in-tree) — would have broken a future compile. Now branch is
  `(3.3f)` (batt.c:26), dead branch plain `#define VOLTS_SCALE (0.0f)`-style
  — actually the dead branch was removed leaving the macro unconditional.

### 1.5 auto.c — `InitiateLiftEscape` N/E excursion swap (auto.c:279-282)
**Concern:** escape position offset used
`North += Excursion * sinf(Bearing); East += Excursion * cosf(Bearing);` — the
**reverse** of the codebase convention (`nav.c:322` `WPBearing = atan2(East, North)`
⇒ North→cos, East→sin; cf. nav.c:344-345). A 90°-off escape offset on a
lift-escape (ExcessLift recovery) climb-out.

**Fix:** `North += cosf(Bearing); East += sinf(Bearing);` — now matches the
established `atan2(E,N)` convention used everywhere else.

### 1.6 emu.c — `InertiaR` axis-ordering (emu.c:521-526)
**Concern:** `InertiaR[] = { base, base/1.5f, base/2.5f }` indexed by Attitudes
enum `{ePitch=0, eRoll=1, eYaw=2}` — but the block comment says `J_pitch =
J_roll×1.5` (pitch heavier) and the yaw row is lighter (`/2.5`). The *matching*
array should be `{ pitchR, rollR, yawR }` = `{ base/1.5, base, base/2.5 }`, so a
heavier physical pitch gets a **smaller** inertial angular acceleration factor.

**Fix:** MS plant now `{ rollPitchInertiaR / 1.5f, rollPitchInertiaR,
rollPitchInertiaR / 2.5f }` — pitch uses the dominant (smaller) term, roll the
base term, yaw the lightest. Emulator-only; physical aircraft unaffected.

---

## 2. GCS — `uavx-python/src/packet_parser.py`

### 2.1 `gps_type` offset bug (packet_parser.py:603)
**Concern:** parser read `extract_byte(data, 69)` while the FC payload
(`SendNavPacket`) puts `ubxMajorVersion` at **offset 68** and a **`TxESCu8(s,0)`
pad at 69**. The docstring even documented `[68] ubxMajor [69] pad`. So
`gps_type` always read the zero pad byte → always 0.

**Fix:** `extract_byte(data, 68)`.

### 2.2 `parse_calibration_packet` docstring (packet_parser.py:1017)
The FC `SendCalibrationPacket` sends **2 orientation bytes** (`SensorQuadrant`,
`SensorFlip` — telem.c:625-626) and the length word is
`FLAG_BYTES+4+2+3*22+10+16+4+2+3` = 119; the GCS docstring claimed
`orientation(3)` (would be 120 and misaligned). Code itself is correct
(reads at 16,17); **docstring corrected** to `orientation(2)`.

---

## 3. Verified — NOT bugs

- **`UpdateRCMap` (rc.c:698-736):** `Map[]` values are ParamTable-bounded
  0..15 (`params.c:170-252`), all < `RC_MAX_CHANNELS`=20 → `Count[NewMap[c]]++`
  is in-bounds. Params are the only writers of `Map[]`.
- **`NavComNames[]` (mission.c:28) 4 entries vs 9-value `NavActions` enum:**
  **dead code** — no consumer in tree (verified with grep). Not a runtime bug
  today, but a latent OOB for any future index-by-enum; flagged for cleanup.
- **`eAcquiringAltitude` timeout (auto.c:624-625):** `mSTimeout` is assigned but
  never re-checked inside the `eAcquiringAltitude` case body (auto.c:512-558).
  **Gap, not a bug:** a stuck alt-hold acquisition orbits forever because the
  FSM never times out. **Reported, not changed** (flight-visible FSM; needs
  Greg's decision on the escape path — `InitiateLiftEscape` already exists for
  the ExcessLift variant).
- **FW turn-sign (`nav.c:335-337`):** `Turn = MakePi(Heading - Nav.DesiredHeading)`
  with `MinimumTurn()` producing negative roll for East targets looked
  wrong-headed — but the **legacy commented-out call
  `MinimumTurn(Make2Pi(Heading - Nav.WPBearing))` ALSO produces negative roll**,
  so the two agree and only the MR branch is an end-to-end-validated emu anchor.
  **Do NOT change solo** — recommend an SVN diff of nav.c:335-337 with Greg
  before touching.
- **`DoOrbit` (nav.c:148-158):** local frame is orbit-centre-aligned
  (`TangentialVelocity` = radial, `OrbitVelocity` = tangential) but rotates by
  `-Heading` (which during orbit is `bearing+90°`), argued to be `-WPBearing`.
  **Flight-visible orbit geometry — reported, not changed solo.**
- **Division by `Nav.MaxVelocity` (nav.c:96 `WPDistanceTimeout`,
  auto.c:288):** param `MaxVelocity` min is 0 (params.c:194). A `/0` wait —
  but max velocity 0 is a never-sanctioned config (idle ground setups only).
  **Reported**; recommend a ParamTable min bump or a guard. No division-by-zero
  in shipped configs.
- **Trace/identify CAPTURE_FLASH_SECTOR collision:** `identify.c` reuses the
  mothballed CAPTURE sector at `0x80C0000` — deliberate (identify docs, §flash
  snapshot). No trace captures and identify snapshots can coexist (both 128 B
  headers at the same base + exclusive use of the sector). Verified agreement
  in AGENTS + Session_Report_InflightPlantID_Sep14 §11.
- **tag-57 / WP-42B / Origin-18B packet offsets + ESC state-machine:** verified
  against the FC senders — all correct (the tag-57 v1 parse + the legacy 38 B
  fallback both decode cleanly).

---

## 4. Report-forward items (do-not-change-solo)

1. **FW turn-sign** (`nav.c:335-337`) — see §3. Needs SVN history + Greg.
2. **DoOrbit** `-Heading` vs `-WPBearing` (`nav.c:148-158`) — flight-visible.
3. **`eAcquiringAltitude`** timeouts never checked (`auto.c:512-558`) — escape
   path decision needed.
4. **Division by `Nav.MaxVelocity` min 0** (`nav.c:96`, `auto.c:288`) — clamp or
   ParamTable min bump.
5. **GCS live-link ESC decode duplication:** `main_window.py:270-346` still has an
   inline ESC-stuff/unstuff decoder in the live telemetry handler, while
   `core/frame_decoder.py::FrameDecoder` (the "one decoder" per AGENTS) is used
   only by replay. Behaviourally byte-identical; the invariant is aspirational,
   not enforced. Deferred refactor so live telemetry isn't disturbed.
6. **`NavComNames[]` dead 4-entry array** — delete or widen to the enum.

---

## 5. Build / verification status

- **FC:** `BOARD=SPEEDYBEEF405WING python3 scripts/fc_build.py` → **ALL OK**,
  `SPEEDYBEEF405WINGQ_r0.bin` **235,268 bytes** (was 235,268 — net-zero image
  size for the telemetry/battery/clamp edits, `.data` unchanged, purely intra-
  function logic).
- **GCS:** `python3 -m py_compile packet_parser.py` → **OK**.
- Untouched flow: no protocol tags, wire formats, param indices, or struct
  layouts changed. All fixes are intra-function — no reflash-required layout
  changes, and existing `.af` files loadable unchanged.