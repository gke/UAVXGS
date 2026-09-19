# Telemetry Scheduler Redesign — Context-Drive Everything

## Goal
Replace the flat "send the same 6 packets every 100 ms whatever the state"
(`SendUAVXTelemetry`, telem.c:1332) with a context-driven rate table. Keep
wire tags stable (no GCS parser breakage); the refactor is purely *what runs
when*.

## Untouched(**) — separate physical link, not part of this refactor
- **FrSky D8 / softserial** (`SendFrSkyDTelemetry`, frsky.c:509): already a
  model multi-rate schedule — attitude+vario @125 ms, baro/GPS-course/compass
  @500 ms, battery/fuel/GPS/where @1 s. Runs on `SoftSerial`/`FrSkySerial`,
  not `TelemetrySerial`. **Do not reschedule this.**
- **LUA script** on the Tx consumes the D8 IDs (see frsky.c ID_* enum); the
  WHERE_DIST/BEAR/ELEV/HINT IDs (0x07/0x2a/0x2b/0x2c) exist specifically for
  a custom home-guidance script. Keep the IDs and their cadence.
- Alarms for the LUA script ride the packed flight-mode word
  (`TxFrSkyDGetFlightMode`, frsky.c:434) and the GPS-state word
  (`TxFrSkyDGetGPSState`, frsky.c:473). If we want richer alarms than the 15
  flag bits allow, plan a NEW D8 ID (free space above 0x39 / below 0x3F) and a
  Tx-side script change — separate task from the GCS scheduler.

## Scheduler mechanics (Fed to the existing `CheckTelemetry`)
Keep a base cadence of 10 Hz (100 ms, `UAVX_TEL_INTERVAL_MS`) as the
*scheduler tick*. Each context provides a **period-multiplier table**:

```c
typedef struct {
    enum { RATE_0, RATE_EVENT, RATE_10, RATE_5, RATE_2, RATE_1 } rate; // 0Hz, event, 10..1Hz
} TelRate; // per tag

typedef struct {
    uint8 contextId;
    TelRate rate[NUM_TEL_TAGS]; // period multiple of the 10 Hz tick
} TelContext;
```

An event-driven packet is emitted by its own trigger (we already do this:
tag-76 I2C errors, tag-74 anomaly). `RATE_0` = send nothing in this context.

## Contexts
Selected in `SendScheduledTelemetry` → `GetTelContext()` from `State` / `F.*`
flags / `Armed()`:

| # | Context | Selector |
|---|---------|----------|
| — | `DIAG_TEST` | `F.TestActive` (link owned by tests; telemetry suppressed — handled in `CheckTelemetry`) |
| 0 | `TEL_CTX_BENCH` | not Armed: `Ready/Landed/Preflight/MonitorInstruments` |
| 1 | `TEL_CTX_GROUND_IDLE` | `Armed()` and not InFlight |
| 2 | `TEL_CTX_HOVER_MANUAL` | `InFlight`, no WP-nav/RTH/POI/Navigate |
| 3 | `TEL_CTX_AUTO_NAV` | `InFlight` + `UsingWPNavigation`/`ReturnHome`/`Navigate`/`OrbitingWP`/`UsingPOI` |
| 4 | `TEL_CTX_CRISIS` | `RCFailsafe`/`RCSignalLost`/`LowBatt`/`RapidDescentHazard`/WDT trip |

`CRISIS` evaluates live from the flags each scheduler tick — those latch
themselves (failsafe/RTH/WDT) and decay as conditions clear.

## Rate table as implemented (`TelPeriodTable`, telem.c)

Period in 100 ms scheduler ticks: **1 = 10 Hz, 2 = 5 Hz, 5 = 2 Hz, 10 = 1 Hz, 0 = off**.

| Stream (tag)             | BENCH | GROUND | HOVER | AUTO_NAV | CRISIS | Why |
|--------------------------|:-----:|:------:|:-----:|:--------:|:------:|-----|
| Flight (14)              |  1    |   1    |  1    |   1      |   1    | Attitude/horizon/batt/alt — pilot-critical, never stalls |
| Nav (15)                 |  0    |   5    |  5    |   2      |   1    | GPS pos/speed — busy when navigating |
| Guidance (59)            |  0    |   0    |  5    |   2      |   2    | Only meaningful with origin+nav |
| Wind (64)                |  0    |   0    |  5    |   5      |   5    | Slow-moving estimate |
| RCChannels (23)          |  2    |   1    |  1    |   2      |   2    | Sticks matter in manual; idle in auto |
| RCLinkStats (69)         |  5    |   2    |  2    |   2      |   1    | Link health is crisis-critical |
| ExecTime/IMU/WDT (67)    | 10    |  10    | 10    |  10      |  10    | Diagnostics — 1 Hz is plenty |
| SerialPort (66)          | 10    |  10    | 10    |  10      |  10    | Diagnostics |
| Tuning (57)              |  E    |   E    |  E    |   E      |   E    | GCS-requested on connect (tag 17 style request); not in table |
| I2C errors (76)         |   E   |   E    |   E   |   E       |   E    | Already event + 1 Hz cadence |

\* = optional fast-core: send Flight (or its slim variant) at 20 Hz by also
emitting on the 50 ms half-tick.

Note: `ExecTime`/`RCLinkStats`/`SerialPort` today ride the `SendStatus`
alternation; that logic folds into the table and disappears.

## Expected budget (in-flight ≈ 1.5 KB/s vs ~3.0 today; bench ≪ 1 KB/s)
- Fast core: Flight 125 B @10 Hz + RC 23 B @10 Hz (hover/ground) = ~1.48 KB/s
- AUTO_NAV drops RC to 5 Hz; Nav/Guidance @5 Hz ≈ +360 B/s ≈ 1.7 KB/s peak
- Health/Exec/Link @1 Hz ≈ 45 B/s
- Event packets ≈ 0 steady-state; crisis briefly pushes Link+Nav to 10 Hz
Total ≈ **1.5–1.7 KB/s (≈15% of 115200)**; bench (Flight 10Hz + RC 5Hz + Link
2Hz + health 1Hz) ≈ 1.8 KB/s of mostly-attitude.

## Optional phase 2 (wire change — do only with GCS in lockstep)
- **Slim Flight core:** attitude/quat + battery + throttle + heading + alt +
  flags (~65–70 B) in a new tag at 10–20 Hz; move KF variances / temps /
  AccConfidence / pressure to a single 1 Hz `HealthPacket`. Needs a new tag +
  GCS parse; the numbers above already assume the tagging stays as-is, so this is
  purely extra headroom for a future 50 Hz horizon.
- **Alarm word** for the LUA/FrSky side: extend `TxFrSkyDGetFlightMode` bits or
  add an ExLift/WDT/IMU-fault alarm into the D8 stream via a single extra ID.

## Implementation status (2026-08-10)
1. `telem.h`: added `TelContextTypes`, `TelStreamIdx`, `SendScheduledTelemetry`
   prototype. ✅
2. `telem.c`: added `TelPeriodTable` + `GetTelContext()` + replaced the
   `SendStatus` alternation with `SendScheduledTelemetry(tick%period)`. ✅
3. `SendTuningPacket` removed from the 10 Hz loop; the FC now answers the GCS
   `TUNING` request (tag 57) inside `ProcessRxPacket` (was falling through to
   `SendAckPacket` fail before). ✅
4. `CRISIS` is selected live from `RCFailsafe || RCSignalLost || F.LowBatt ||
   F.RapidDescentHazard || (WdtTripMark != WDT_TRIP_NONE)` — these flags latch
   themselves, so no extra decay timer was needed. ✅
5. Rebuilt binary + `py_compile` both GCS files (no wire-format change, so the
   GCS parser needed no edit). ✅

Pending bench spot-checks (next session): horizon smoothness in HOVER, no
"Discarding bytes" on bench, FrSky D8 cadence untouched, and `imu_peak_us` /
`avg_us` (tag-76) sanity during flight.

## Dross removal (2026-08-10)
Removed dead FC senders (zero callers), wire-format untouched:
- `SendSoaringPacket`, `SendAltitudeControlPacket` (tag 60),
  `SendAttitudeControlPacket` + `SendQuaternionPacket` (tag 68) and their
  `telem.h` prototypes; also the commented-out call in `state.c`.
  The horizon/attitude is fed by Flight (tag 14) quaternion, not these.

Unused tag names in `telem.h` PacketTags renamed with an `Unused` prefix
(NavLev 1–12 + 53,55,56,58,60,61,65,68). Names only — enum ordering and
implicit values 0,1,2… are untouched so wire numbers and the GCS parser stay
in lockstep. Mirrored in GCS `protocol_enums.py` (`UNUSED_*`) and the
tag-name label dicts (`main_window`, `misc_window`, `packet_logger`), which
are display-only and keyed by number.

GCS dead tag-60/68 consumers removed: `AltitudeControlData`,
`AttitudeControlData`, `parse_altitude/attitude_control_packet`, dispatch
cases 60/68, `altitude_control_data` attr + handlers.