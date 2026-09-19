# Feasibility — TX-SD logging of in-progress Trace ("crash") data over CRSF

Date: 2026-08-30. Question (user, Aug30): **"Can I send in-progress crash data
to my TX for it to log, when I am currently using iNav LUA in CRSF Rx mode?"**

Scope: UAVXArmQ (this repo's active fork) as the CRSF device, EdgeTX radio in
CRSF/ELRS receiver mode running the iNav Telemetry Lua. The "crash data" is
the in-progress **Trace** capture ring (`trace.c`: CCM `BBQ[65536]`,
`0x54524143`, 16 B × 3-axis rows — see AGENTS.md Trace section).

This is the third candidate **capture carrier** for the Stage-1 roadmap
(after USB-tag-54 and the D8 softserial burst). It is the only one that
survives a battery-eject crash: the TX SD is in the pilot's hand.

---

## 1. What the FC already does over CRSF (facts from source)

`src/rc.c` + `src/telemetry/txtelem.c` (adapted from iNavFlight, attribution
present):

- **RX path** (`pRxType == eCRSFRx`, half-duplex `MODE_RXTX`): parses
  `RC_CHANNELS_PACKED` (0x16), `DEVICE_PING` (0x28), `LINK_STATISTICS` (0x14).
- **TX telemetry**: 8-slot schedule (Attitude, GPS, Vario, Battery,
  FlightMode, Temp, BaroAlt) at one frame per `CRSF_TELEMETRY_SLOT_US = 12500`
  (= 12.5 ms ≈ 80 frames/s capability), written by `CRSFTTelemetryWrite` →
  `TxChar(RCSerial, …)`. DeviceInfo (0x29) reply to a ping.
- **Not implemented**: parameter frames (`PARAMETER_*` 0x2B/0x2C/0x2D), MSP
  (0x7A–0x7C), DisplayPort (0x7D). `crsfDecode()`'s default arm just drops
  them (silently).

**Consequence 1 — iNav LUA cannot couple to UAVX.** The iNav Telemetry Lua
discovers the device via DeviceInfo, then reads/writes parameters using the
CRSF parameter API. UAVX answers neither. Today the Lua would find a "UAVX"
device string and then stall on empty param pages. If you run iNav LUA in CRSF
mode it is wired to an **iNav** craft elsewhere in your fleet; it is not (and
cannot currently be) the drone-side link for UAVX data.

## 2. Can the ring ride the CRSF downlink live? Bandwidth verdict

| item | rate/size |
|---|---|
| Trace capture (rate type) | 16 B × 3 axes @ 500 Hz = **24 kB/s** |
| 64 K CCM ring (fresh window) | ~2.7 s @ 500 Hz |
| 50 Hz types | 2.4 kB/s → ~27 s |
| CRSF telemetry air budget available to a third-party sender | **~0.2–0.5 kB/s realistic** (shared with the RC uplink + the existing 8-slot sensor feed; ELRS further reserves uplink priority) |
| RPM-sensor side-channel capacity | ≤ 8 B/payload usable cleanly ⇒ ≈ 4 payload frames/s monitored ⇒ **≈ 32 B/s**, or a transient burst of ~64 B/frame×5 ⇒ ~0.3 kB/s |

**Verdict:** streaming the whole ring live is off by ~two orders of magnitude.
A circular fire-and-forget dump would fall ~60× behind the capture line, so
the crash moment would never be in the flushed content anyway. **Do not design
around live full-rate transfer — it is the "crazy idea" to squash now.**

## 3. What IS feasible: post-crash / on-demand flush, or a bounded summary

The TX-SD channel is valuable exactly because the TX survives the impact while
the FC may not (ring is volatile CCM; flash commit happens on disarm — if the
battery ejects before `TraceCommit()` the ring is lost). Three honest modes:

1. **Post-crash flush (recommended target)**: craft is down but the FC is still
   powered (typical for at least tens of seconds; RX + telemetry draw ~mA).
   Stream the committed snapshot (32 B header + `recordCount×16`) up as RPM
   frames on the existing CRSF serial; EdgeTX logs the RPM sensors to SD with
   seconds of residual battery. 64 kB at ~0.3 kB/s ≈ **3–4 min** — acceptable
   while walking over; 50 Hz captures (~2.4 kB/s content, ~2–27 s of ring =
   ≤~64 kB) land in <1 min.
2. **Purpose-built crash summary**: last ~2 s of 50 Hz attitude/alt-hold rows +
   a few flag octets ≈ 1–5 kB → ~15 s flush even at marginal link quality.
   Reuse the *same* `critic` extractors to decide what is worth shipping.
3. **On-demand pre-landing flush** (trigger = ch8 or a nav-mode edge while
   still powered): gives the "in-progress capture" without the crash.

In every mode the FC keeps `TraceCommit()`/tag-54 as the primary path; CRSF-SD
is an additional sink, not a replacement.

## 4. The mechanism that makes TX logging actually work

EdgeTX (file-based evidence: `radio/src/telemetry/crossfire.cpp`) decodes CRSF
remote-telemetry into a fixed `crossfireSensors[]` table; each known frame type
feeds fixed sensors, and **every sensor carries a per-sensor "Logs" flag**;
SD Logs (Special Function) then writes `LOGS/<model>-<date>.csv`. Critically:

- `CF_RPM_ID` (frame 0x0C) yields **RPM0…RPM7** sensors, index = `sensorID`,
  three 16-bit value fields per frame.
- The **per-sensor "Logs" flag exists**; the fallback unknown entry is
  `STR_UNKNOWN UNIT_RAW` — i.e. unknown types are *not* surfaced, so a raw
  0x7F "user-defined" frame (reserved per crsf-wg issue #4) would silently die
  on stock EdgeTX. **Do not bet on 0x7F.**

Consequently the clean approach is: **pack trace bytes into RPM frames**
(`id` byte = chunk index, three value fields carrying 8 little-endian chunk
bytes, chunks 0–N with the header first: magic, period, type, axisMask,
recordCount). EdgeTX treats the stream as RPM chatter, logs it natively —
**no custom Lua, no script slot, no conflict with iNav LUA** (the Lua only
demands the parameter API the FC cannot answer anyway). The GCS side later
re-assembles `RPM0..RPM7` columns of the CSV back into the TRAC blob via the
existing `parse_trace()`.

## 5. Conflict check against "iNav LUA in CRSF Rx mode"

- **Lua slot**: EdgeTX has a single telemetry-script slot; iNav LUA occupies
  it. The RPM-carrier needs *no* script, so there is nothing to evict.
- **Link**: the FC already shares one half-duplex serial with the CRSF RC
  uplink; adding frames only reuses the slot budget it already honours. ELRS
  module link-share backoff must be measured, not assumed (see §7).
- **What changes**: a new `trace→CRSF` flush state added to `txtelem` (or a
  small `trace_flush_crsf.c`), gated so it never starves the normal 8-slot
  schedule (insert N RPM frames as up to ~30 % of slots while flushing).

## 6. Verdict

| option | feasible now? |
|---|---|
| iNav LUA displaying UAVX params over CRSF | **No** — FC has no param 0x2C/0x2D/MSP support (addable, but a sizeable protocol feature; independently useful only if you want iNav-style tuning on the radio) |
| Live full-rate ring over CRSF | **No** — ~50× bandwidth shortfall, and the flush lag misses the crash moment |
| Post-crash / on-demand summary or snapshot flush over CRSF → TX SD | **Yes** — RPM-carrier, native EdgeTX CSV logging, no script slot, survives battery eject |
| D8 burst (existing alt) | **Yes** — softer bandwidth but no TX SD (logs live in μSD of the RX-free radio only via own sensors) — remains the cheap bench/backup path |
| USB tag-54 | **Yes** — primary bench path, zero airtime cost (already built) |

**Bottom line:** the idea is sound and worth building — but as the **crash
carrier with a bounded (post-crash/on-demand) payload**, not as a live stream,
and it does not depend on iNav LUA in any way.

## 7. Decisions + verify-on-your-radio notes (before any code)

1. Commit to the **RPM-frame carrier** vs a Lua-based 0x7F scoop (RPM is
   native, robust, cheap; 0x7F needs a custom Telemetry Lua → conflicts with
   iNav LUA's slot — so RPM wins unless you want a dedicated second radio
   layout without iNav LUA).
2. Measure real downlink headroom with your ELRS/TBS module (fly a test,
   watch `LQ`/`RF Mode` while the FC floods RPM frames) before fixing slot
   share; guard: abort flush if `F.LQCrit`.
3. Decide the trigger: post-crash = disarm edge (`TraceCommit` exists),
   pre-landing = ch8/Aux2 hold or nav-mode edge; the FC should never flood the
   link mid-manoeuvre (in-air flush only when disarmed — consistent with the
   "no writes in the air" stance; this is RF, not flash, but the link must
   never be monopolised while the pilot needs it).
4. EdgeTX per-model: enable "Logs" on the RPM sensors it discovers + an SD
   Logs special function (TELE trigger). Verify the FC's frames parse — note
   the FC currently emits CRSF frames without the `dest=0xEA / origin=0xEC`
   two-byte prefix some EdgeTX builds expect; the current telemetry clearly
   works for you, but confirm RPM-index sensors appear on "Discover".
5. GCS reassembly: CSV→TRAC converter in `uavx-python` reusing
   `parse_trace()`; the schemas (16 B base row, header offsets) are already
   pinned in AGENTS.md, so the converter is mechanical.

No code written for the carrier this session (report only). This is the
documented Stage-1b/Phase‑2 candidate alongside the D8 burst; the USB-tag-54
path remains the committed baseline.