# Session Report — D8 Telemetry, Mag Gating, Yaw I Removal (28 Aug 2026)

## Summary

Three FC change-sets landed this session: (1) FrSky D8 telemetry bug fixes and
feature enables, verified against the EdgeTX D-protocol decoder; (2) re-scoped
magnetometer gating to a config-bit master switch with PassThru-only auto
exclusion; (3) removed the dead yaw rate-loop I-term scaffold, yaw rate control
is now pure PD. Plus (4) a GCS-side fix: the speech level combo now defaults to
All and no longer gets silently persisted to Off on an engine hiccup.

## Changes

### 1. FrSky D8 Telemetry (`UAVXArmQ/src/telemetry/frsky.c`)

| Item | Before | After | Rationale |
|------|--------|-------|-----------|
| Vario | `ROC * 10.0f` | `ROC * 100.0f` | EdgeTX decodes ID_VERT_SPEED (0x30) as m/s, prec 2 → expects cm/s. Was 10× low on air. |
| Time (day/month) | `GPS.month << 8 + GPS.day` | `((GPS.month << 8) \| GPS.day)` | Precedence bug (`+` binds tighter than `<<`). EdgeTX: `month = data>>8`, `day = data & 0xFF`. |
| Time (hour/min) | `GPS.minute << 8 + GPS.hour` | `((GPS.minute << 8) \| GPS.hour)` | Same precedence bug. EdgeTX: `hour = data & 0xFF`, `min = data>>8`. |
| Time (year) | N/A | `GPS.year` raw 4-digit | EdgeTX UNIT_DATETIME_YEAR stores raw value; ublox year is 1999..2099 — already correct, verified. |
| Heading | `MakeFrac(v, 0)` | `MakeFrac(v, 10)` | Latent divide-by-zero on frac 0. |
| Cells | commented out | enabled; burst all indices in one 1 s slot; `BatteryCellCount > 0` guard | Cells are synthesized single-pack averages (standard — Betaflight/iNav do the same). Burst (not 1 cell/s rotation) so EdgeTX "Cels" total populates fast. |
| mAh | scheduled | **dropped** | ID_MAH (0x36) is a UAVX-custom ID → raw hex sensor on stock EdgeTX; radio computes consumption itself. Function + both ID enum entries + schedule call removed. GCS mAh telemetry untouched. |
| Time send | scheduled 5 s slot | kept, now functional | EdgeTX cannot RTC-sync without a full frame set. |
| Schedule | — | attitude+vario 125 ms; baro/COG/heading 500 ms; cells/VFAS/current/fuel/GPS/sats 1 s; time 5 s | Bandwidth: ~243-255 B/s ≈ 25-27% of the 9600-baud softserial budget; peak 5 s slot absorbed by the 1024-byte TX FIFO. |

Framing decision: verified EVERY byte layout against the EdgeTX decoder source
(`frsky_d.cpp`, `telemetry_sensors.cpp`) rather than the widely-copied but
sometimes-wrong community docs.

### 2. Mag Gating — re-scoped (`UAVXArmQ/src/inertial.c` `MadgwickUpdate`)

```
if (F.NewMagValues && !F.MagnetometerFailure
        && !F.PassThru && (State == eInFlight)) {
```

- `UseMag` config bit (`UseMagMask`) is the **master switch**: off = gyro-only
  yaw. Already upstream-zeroes mag inputs and clears `F.NewMagValues`
  (`inertial.c` `DoSensorUpdate`), so the gate is belt-and-braces.
- Auto-exclusion ONLY in PassThru/ePIC (bare-metal), never in normal manual or
  autonomous flight.

### 3. Yaw Rate I-Term — removed (`UAVXArmQ/src/control.c` `ControlRateYaw`)

Stripped the hold-only integral scaffold (`R->IntE`, `R->IntLim`,
`R->ITerm` from output). Yaw rate output is now `PTerm + DTerm` only.

## Rationale & Discourse

### D8 — why these items
- Original vario was 10× low (dm/s vs cm/s) — users saw "10.5" for 1.05 m/s and
  thought it broken. Confirmed against EdgeTX decoder units, not folklore.
- Time byte-order fixes: the FC previously sent `(month<<8) | date-with-a-bug`;
  the radio showed garbage dates. GPS clock data is only useful if it matches
  the decoder's packing.
- mAh: explicitly a **borderline** call. Rejected keeping it after checking that
  EdgeTX treats 0x36 as an unknown "raw" sensor (no decoding, no consumption
  integration). Dropping also frees a 1 s slot in the burst.

### Mag — the discourse (this is the interesting one)
- **First proposal**: gate mag fusion to autonomous flight only (RTH/WPNav/
  AltHoldNav), excluding manual flight, motivated by bench interference.
- **User counterpoint**: the bench interference was an artifact of the test
  rig (adjacent fields); airborne interference sources don't justify crippling
  manual-flight yaw anchoring. The existing `UseMag` config bit already lets
  the developer turn the mag off while bench-debugging — a "kill switch" is
  better than a mode heuristic.
- **Adopted**: `UseMag` = master switch (off while debugging), `PassThru`
  exclusion only (in PassThru the mag has no role and its readings would drag
  the estimate in bare-metal mode), fusion active in ALL other flight states
  (manual + autonomous) so a live calibrated mag anchors yaw everywhere.
- Rejected: re-gating on `NavigationEnabled || AltControlEnabled` — that had
  been implemented earlier but the user judged the mode heuristic wrong for
  flight, and the config bit makes it unnecessary.
- Note: `eInFlight` and `MagnetometerFailure` guards retained — they are state/
  health gates, not mode gates, and match the prior behavior.

### Yaw I — why it is NOT a missing feature
- A partial I-scaffold had crept in (reset-on-stick / zeroed in rate mode) but
  `A[eYaw].R.Ki`/`.IntLim` were never driven by any param or runtime write
  (both 0.0 at boot) — dead code, either way was always 0.
- **Discourse**: could we justify finishing it? Rate-level I only pays off when
  a constant rate is sustained long enough for steady-state error to converge.
  Here the yaw rate setpoint comes from the outer quaternion heading loop, so it
  is transient during every heading change and near-zero otherwise. On MC (fast
  yaw dynamics) and FW (inertia + weathercocking self-trim + tiny rudder
  authority) the integral would mostly accumulate transient lag → windup/
  overshoot on settle, FW snaking (fighting aero self-trim). Sustained heading
  error is already held by the outer `YawAngleQKi`.
- **Adopted**: strip the dead scaffold, keep yaw rate pure PD. User agreed after
  the "sustained constant rate" framing. Revisit only if a dedicated rate-hold
  (constant demanded rate sustained) application appears.

### 4. GCS Voice Feedback — speech level defaults to All (`uavx-python/src/core/speech.py`, `ui/main_window.py`)

Symptom: the GCS main-page "Speech:" level combo came up on **Off** and the GCS
was completely silent, and it was "lost track" of when it broke. Two independent
defects caused it:

| Item | Before | After | Rationale |
|------|--------|-------|-----------|
| Level vs capability | `SpeechController` forced `_level = OFF` whenever `pyttsx3.init()`/voice selection failed (`speech.py` init + `level` setter), then main_window **persisted** that OFF to QSettings (`speech_level`) | `level` is a pure preference, never auto-downgraded; engine capability lives in `available` + `init_error` | A one-off engine hiccup wrote OFF into persistent storage → every later boot loaded the mute forever. Preference and capability must not be conflated. |
| Stale stored OFF | `load_settings` honored whatever was stored | missing/invalid/non-int stored value **and** a stored `Off` → reset to `All` (logged via `log_debug`) | Operator asked for "default to all"; a stored Off is stale by construction (only the old forced-OFF ever wrote it unintentionally) |
| Concurrent speech | one daemon thread spawned **per call**; several `speak()`s overlapping on one pyttsx3 engine | single consumer worker thread (`queue.Queue`), `runAndWait` serialized | pyttsx3/espeak is not thread-safe; overlapping runs silently drop audio = the "completely silent, was working" symptom. |
| Linux voice verification | `want in engine.getProperty('voice')` with `want = "en-gb+f3"` — pyttsx3 echoes back the base id `"en-gb"`, so the variant was **rejected every time** and the fallback loop ran | compare against the **base** name (`want.split('+')[0]`) so `en-gb+f3`/mbrola variants are accepted and stick | Previously the preferred "gentle British female" voice never applied. |

Additional hardening in speech.py: engine init is wrapped so any driver/TTS
exception is swallowed into `init_error` (never crashes the GCS, and the
reason is now surfaced to the status log); `speak(..., async_mode=False)`
waits on the worker queue; `Queue`/`threading` imports replace the unused
`Optional`.

### 5. Telemetry "not updating" — `TelPeriodTable` depopulated (regression, same session)

Symptom: GCS main page frozen; console flooded `[RX] tag=0 (Unknown(0)) FAIL
len=70B` while request/response frames (71/63/57/51/76) still decoded fine and
**no tag-13 (Flight) / tag-14 (Nav) frames ever appeared**.

Diagnosis path:
- Confirmed the GCS link itself was healthy: `[SERIAL]` framing + parser decoded
  tag 70/71/63/57/51/76 correctly; the FAIL frames were downstream noise, not
  the cause.
- Confirmed the D8 (softserial) work was NOT mixing with the GCS link:
  `FrSkySerial = eSoftSerial` (PA2), `RCSerial = eUsart1` (CRSF), and
  `TelemetrySerial = eUSBSerial` (USB VCP) on SPEEDYBEEF405WING —
  three separate serials. CRSF signalling+telemetry share USART1 by design and
  never touch the USB UART (checked `speedybeef405wing.inc`,
  frsky.c:570-573 dispatch gated `s == eSoftSerial`).
- Root cause: **`TelPeriodTable` initializer was accidentally commented out**
  during the earlier comment-cleanup pass — the table stood as `= { };` (all
  zeros), so `SendScheduledTelemetry` skipped every stream on every context.
  On the bench (disarmed) the craft sits in `TEL_CTX_BENCH`; with no values the
  FC deliberately emitted nothing but event/request-driven frames (tag 76 I2C
  counters, tag 71/57/63 responses) — exactly "the packet sequencer was
  depopulated — all zeros" that the user recognised from a previous episode.

Fix: restored the real per-context periods as ACTIVE code in
`UAVXArmQ/src/telemetry/telem.c` `TelPeriodTable` (Flight 10 Hz in every
context; Nav 2 Hz + others per the original rows), with an explicit warning
comment that an all-zero table mutes the whole link. Kept the documented
column order matching `enum TelStreamIdx`.

Build: all 5 FC targets rebuilt clean
(`obj/<BOARD>/<BOARD>Q_r0.bin`). Flash
`SPEEDYBEEF405WING/SPEEDYBEEF405WINGQ_r0.bin`, relaunch the GCS (kill any stale
instance — run.sh's PID guard ignores `!!` against an already-running app, which
is why the earlier reruns showed no `raw=` FAIL dump), and tag-13 flight frames
should stream again. The new `raw=` FAIL hex dump (main_window.py:2123) stays in
as a standing diagnostic if any FAIL frames remain.

WATCH-ITEM for future cleanup passes: keep the `TelPeriodTable` initializer as
real code; never let a "comment cleanup" swallow a data table again.

### 4b. Boot greeting + no-backend fallback (follow-up, same session)

The `_speak_boot_greeting` test (Help ▸ Test Spoken Feedback / Ctrl+T, and
1.5 s after launch) was added so the fix can be heard on boot. First real run
reported "nothing heard" — diagnosis:

- This dev box had **no TTS backend at all** (no pyttsx3, and no
  espeak/espeak-ng/spd-say/flite). With the original code the controller was
  forced to OFF, the combo showed Off, and nothing ever spoke.
- The "Speech engine unavailable" diagnostic I'd logged was at **Warnings**
  level, which the default "Info" combo filter drops — so the failure was
  invisible. Now logged at Info (always visible), including which backend is
  in use.

Additions:
- `backend` property (pyttsx3 / espeak-ng / spd-say / flite / none) so the
  status log says exactly how a phrase is being rendered.
- `_SubprocessTts` fallback: when pyttsx3 is unusable, shell out on the worker
  thread to the first available of espeak-ng → espeak → spd-say → flite.
  Rationale: the GCS promises pyttsx3 via pyproject, but a broken/missing
  pyttsx3+espeak stack is exactly the silent-history failure mode — a
  subprocess TTS keeps voice alive with zero extra python deps.
- Voice selection failure no longer nulls a **working** engine (init success
  and voice pick are now independent; a bad voice keeps the default voice).
- Boot greeting method logs "spoken via <backend>" or the exact
  `init_error` ("pyttsx3 init failed: …; falling back to espeak-ng", or
  "…no TTS backend found — install espeak-ng or spd-say/flite").
- Help ▸ Test Spoken Feedback menu item (Ctrl+T) re-runs the greeting so the
  operator can re-test without restarting.

Note (host facts from probing `/usr/bin/python3` + no GUI deps present): the
actual GCS launch happens on the host that CAN render it; this change makes the
voice path self-diagnosing there. On "Merlin", first boot should print
`🔊 Boot greeting spoken via pyttsx3` (or `…via espeak-ng`) in the debug log
and speak; if it prints the `🔇` skip line, the reason is right there.

## Build Verification

- `python3 UAVXArmQ/scripts/fc_build.py` — all 5 targets clean:
  UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI.
- GCS: `python3 -m py_compile core/speech.py`, `python3 -m py_compile ui/main_window.py` — both clean (full GUI runtime not runnable in the sandbox: no PyQt5/pyttsx3).
- Deferred note: this Flatpak runtime lacks pyttsx3 and any TTS binary, so live
  voice is tested on the host ("Merlin") where `uavx-python/venv` provides
  PyQt5/pyttsx3. First host launch there showed **All**, spoke the boot
  greeting, and reported the backend via the status log — **confirmed working**;
  any stale `~/.config/UAVX/Groundstation.conf` `speech_level=0` is now reset by
  the load path.

## Open Items (added this session)

- On-air test of the D8 telemetry: vario scaling (cm/s), GPS time/cells decode,
  softserial headroom ~25%, all cells in the 1 s burst.
- Voice feedback investigation (item 1) — `core/speech.py` uses pyttsx3; not
  yet diagnosed.
- On-host voice verification after the speech default-to-All fix: first boot
  spoke and showed **All** in the combo — **confirmed working 28 Aug** (boot
  greeting heard, `backend` reported via status log).