# Session Report — Trace V1 Probe + GCS Viewer (Aug30)

Date: 2026-08-30. Companion docs: AGENTS.md **Trace Capture (implemented Aug30)**
section (ring/header/dump/commit contract) and the **Trace V2** TODO bullet.

## 1. What changed

### FC (`UAVXArmQ`)
- `src/trace.h`: added `TraceStates.eTraceArming`; new probe constants
  `TRACE_TEST_AXIS eRoll`, `TRACE_SETTLE_RATE_RAD_S 0.10f`,
  `TRACE_SETTLE_TICKS 50`, `TRACE_PREROLL_MS 100`, `TRACE_STIM_FRAC 0.25f`;
  prototype `TraceStimulusSetpoint(idx a, real32 d)`. Header doc comment now
  describes the go-ahead semantics.
- `src/trace.c`:
  - `TraceCapture()` reworked from the Aug30 "start on ch8 edge" 3-state machine
    into the 4-state **go-ahead** machine: `eTraceIdle/eTraceHolding` — ch8
    OFF→ON edge while in flight → `eTraceArming`; `eTraceArming` — all three
    `|Rate| < 0.10 rad/s` for 50 ticks (~100 ms) while ch8 held → open capture
    (counters zeroed, 32 B header written); `eTraceCapturing` — 3 records
    (Roll/Pitch/Yaw) per sampling tick until ch8 off / out-of-flight / ring
    full, with the pre-roll baseline running `TRACE_PREROLL_MS` then
    `TraceStimOn = true, TraceStimValue = A[eRoll].R.Max * TRACE_STIM_FRAC`;
    on exit finalize header → `eTraceHolding`.
  - New `TraceRatesQuiet()` settle helper (small fixed loop, no bounds check).
  - New `TraceStimulusSetpoint(a, d)` — the probe override: while the step is
    live ONLY the test axis is replaced with `TraceStimValue`, everything else
    passes through (single-exit form).
  - `InitBlackBox()` resets `TraceSettle`, `TraceStimOn`, `TraceStimValue`.
- `src/control.c`: `ControlRate()` and `ControlRateYaw()` each consume
  `TraceStimulusSetpoint()` once at the top, so the injected step lands in the
  rate-loop error of the current cycle → the trace records show the step in
  `Desired` and the plant response in `Rate` with `Out` tracking.

### GCS (`UAVXGS`)
- NEW `src/ui/trace_viewer.py`: Qt-free `parse_trace(data)` (validates magic
  0x43415254, recordSize, truncation; splits the 3-records-per-tick stream into
  per-axis `t/rate/desired/out`) behind a thin `TraceViewer(QWidget)` shell.
  One `TraceStrip` pane per axis, plain `QPainter`: Desired dashed blue, Rate
  solid green (rad→°/s left axis), Out dotted red on its own ±1 right axis;
  wheel zooms the time window, double-click resets. v1 renders the base 16 B
  row overlay for every type (type selects title + period from the header).
- `src/ui/main_window.py`: `File ▸ Open Trace Dump…` opens any TRAC `.bin` off
  disk; `_finalize_bb_dump()` now opens the viewer directly when the assembled
  blob starts with `TRAC` (no save-as), and keeps the legacy `.bin` save path
  for non-TRAC dumps. `self._trace_viewer` reference prevents GC.
- NEW `src/tests/generate_trace_samples.py`: builds the exact FC byte format
  (32 B header + 16 B base rows) for all 5 selector types with seeded-pure-Python
  signals; writes `tests/trace_samples/trace_{rate,attitude,althold,actuator,imu}.bin`
  and self-verifies header/count/length.

## 2. Rationale and discourse

### ch8 = go-ahead, not trigger (the day's key design point)
Earlier in the session the user asked: "So Ch8 is the pilot saying they are
ready?" → Yes. "The FC knows when the stimulus will be applied so the switch
becomes a go-ahead but it is the test that turns on the capture yes?" → Yes.

Options considered:
- *Pilot-timed capture* (ch8 edge starts and stops recording, pilot also cues
  the stimulus) — rejected: the pilot would have to time the stimulus against
  an arbitrary 2.7 s (500 Hz) / 27 s (50 Hz) window, and the FC would never
  know which sample is the probe onset. Useless for identification.
- *FC auto-run on arm* — rejected: no pilot control over when a test happens;
  a test during stick inputs is garbage and there is no way to decline it.
- *ch8-as-go-ahead + FC-driven sequence* (ADOPTED): the pilot authorises by
  arming ch8 (one test per OFF→ON edge); the FC settles the aircraft, opens
  the capture, runs a fixed pre-roll baseline and then injects the stimulus it
  schedules. The sample timestamps in the records carry the timeline, and the
  viewer can locate the step exactly. This is the correct division of labour:
  the pilot controls *whether* and *when roughly*, the FC controls *what*.

### Settle gate before capture
The probe response is only interpretable from a settled baseline. If the
aircraft is mid-manoeuvre when ch8 is armed, the pre-roll baseline is
contaminated and the step response is buried. So `eTraceArming` waits for all
three gyro rates to sit inside ±0.10 rad/s for ~100 ms and re-checks every 500 Hz
cycle; if ch8 is released or the aircraft leaves flight during the settle the
armed request cancels (no partial capture). The user's motion, "ch8 is the
pilot saying they are ready", maps directly onto this: release-and-rearm if the
first try isn't clean, exactly like arming a switch.

### FC-injected stimulus via ControlRate override
The natural injection point is the rate setpoint, not a separate disturbance,
because the record stores `Desired` and the v1 viewer overlays `Desired` vs
`Rate`. Injecting at the top of `ControlRate()`/`ControlRateYaw()` guarantees
the step lands in the wanted-error of the same control cycle the capture sees,
and the recorded `Desired` holds the step value the control loop actually used.
Options rejected:
- Setting `A[a].R.Desired` from `TraceCapture()` (called at the top of
  `DoControl()`) — the attitude loops recompute Desired immediately afterwards,
  so the override would never reach `ControlRate`. (The very bug that the first
  attempt hit: wiring 218 in an earlier working tree.)
- A separate stimulus-add term accumulated elsewhere — no: the probe IS a
  setpoint step; the trace must show the setpoint it steps.

### Probe amplitude
`TRACE_STIM_FRAC = 0.25` × the test axis rate limit keeps the step inside the
rate limit (Limit1 still clamps) and large enough that the response exceeds gyro
noise. 25% of a typical ~8 rad/s roll limit = ~2 rad/s, a firm but safe step.
V1 deliberately fixes `TRACE_TEST_AXIS = eRoll` — one axis, one step per capture,
per the user's "V1 but note v2 in the todos" decision. A chained Roll→Pitch→Yaw
sequence in one hold, plus per-segment markers so the viewer can draw three step
segments on one timeline, is recorded as **Trace V2** in the AGENTS TODO (with
the extended per-type record rows and the toolbar Trace combo wiring).

### Linear fill, no wraparound
Retained from the earlier Aug30 capture design: the snapshot starts at slot 32
and fills toward the ring end, then holds. Wraparound would make the header's
`snapStart` bookkeeping and the flash commit truncation more complex for no
benefit — the probe window (2.7 s / 27 s) fits comfortably, and re-arming on the
next ch8 edge overwrites from slot 32. Changing to by-record wrap (the AGENTS
ring paragraph still describes wrap for the dump source path) stays consistent
because a finished snapshot is contiguous.

### Viewer shape
Options considered for plotting: matplotlib/pyqtgraph — rejected, deliberately
not in the GCS runtime (per AGENTS). Big hand-rolled QtGraphicsView scene —
rejected: a plain `QPainter` polyline strip is robust, cheap, and sufficient for
3 axes × 3 traces. Two y-scales (left = °/s for Desired/Rate, right = ±1 for
Out) matching the FC record semantics, since Out is a mixer fraction and cannot
share the rad/s axis. All v1 types share the 16 B base row, so one generic
overlay with a type title is the honest rendering today; when a type ships an
extended row (`recordSize > 16`, `version` bump) the parse+layout switch will
happen on `recordSize`, not on viewer special-casing.

### Synthetic dumps
Needed because there is no aircraft on the bench and no PyQt5 in the Flatpak
runtime to eyeball the viewer live. The generator writes the exact wire format
(no Python-side reinterpretation) so the viewer exercises the real layout, and
`parse_trace` was factored out of the widget purely to allow a headless
round-trip check. Signals are seeded `random` + `math` (no numpy available) and
*content*-suggestive per type — Attitude/AltHold/Actuator/IMU will get true
field content when their extended rows land; today they only prove the pipeline.

## 3. Build / verification
- FC: `scripts/fc_build.py` — **all 6 targets OK**
  (UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI,
  BLUEBERRYF405). `control.o`/`trace.o` relinked; no linker errors.
- GCS: `python3 -m py_compile ui/trace_viewer.py tests/generate_trace_samples.py
  ui/main_window.py` — OK.
- Generator run: wrote 5 `trace_samples/*.bin`; internal asserts passed
  (magic/version/recordSize/length).
- Headless parse round-trip (PyQt5 stubbed): all 5 dumps parse; ticks monotonic;
  Out clamped to ±1; per-axis alignment 3×; rate probe shows the step
  (peak Desired 2.17 rad/s ≈ 2.0 step + overshoot, peak Rate 2.14 rad/s) while
  the other types show their intended signatures (attitude sweep 0.55 rad/s,
  althold duty ~0.33, actuator sweeps to 0.75, IMU gyro burst 0.27 rad/s).
- Not run (impossible in the rootless runtime): a live PyQt5 render of the
  viewer. First visual check is on the host/Merlin — open `tests/trace_samples/
  trace_rate.bin` via **File ▸ Open Trace Dump…**.

## 4. Files touched
- `UAVXArmQ/src/trace.h`, `UAVXArmQ/src/trace.c`, `UAVXArmQ/src/control.c`
- `UAVXGS/uavx-python/src/ui/trace_viewer.py` (NEW),
  `UAVXGS/uavx-python/src/ui/main_window.py`
- `UAVXGS/uavx-python/src/tests/generate_trace_samples.py` (NEW),
  `UAVXGS/uavx-python/src/tests/trace_samples/*.bin` (generated)
- `UAVXGS/AGENTS.md` (state table, viewer, Trace V2 TODO)

## 5. Follow-ups
- First live look: render `trace_rate.bin` and one slow type on the host.
- Trace V2: chained per-axis sequence + per-segment markers; extended
  record rows for the 4 locked types (recordSize > 16, version bump, layout
  builder, type-specific viewer layouts) — see AGENTS TODO.
- Inflight auto-tune Stage 1a (criteria externalisation) still gated on user
  approval; the v1 capture gives the data pipeline for it.

## 6. Live-viewer crash + fix (same session, post-report)
First host run of the real PyQt5 (live FC dump via the Dump button):

    AttributeError: 'QPolygonF' object has no attribute 'reserve'
    ... in series_poly -> poly.reserve(len(vals)) -> core dump

- **Root cause**: PyQt5's `QPolygonF` binding does not expose C++ `reserve()`.
  The headless verification had stubbed the Qt classes, so the API never ran
  against the real binding. The parse path (the logic that mattered for the
  byte format) was correct; the paint path was not.
- **Fix** in `ui/trace_viewer.py`:
  - `series_poly` now builds the polygon from a list comprehension directly in
    the `QPolygonF([...])` constructor — no `reserve`, no `append` dependency.
  - `paintEvent` wrapped: the drawing body moved to `_draw()` and the
    event handler catches any exception and paints a red `TraceStrip render
    error: <msg>` readout instead of letting Qt abort the whole GCS. A bad
    pane must never take the app down with a core dump.
- **Why not test-drive it here**: PyQt5 is not installable in the rootless
  Flatpak runtime (AGENTS: compile-checks only). The lesson is recorded —
  PyQt API-surface calls (container methods, ctor forms) are exactly what the
  stub import cannot vouch for; list-constructor/render paths should be kept
  to the most commonly-supported forms.
- **Status**: py_compile OK; headless import + parse of all 5 trace samples OK.
  Re-run Trace Dump on the host to confirm the strips render.

## 7. Toolbar Trace combo wired (same session, after live link confirmed)
Host feedback: "the trace combobox is not populated in GCS yet but the traces
seem OK" (live Dump → viewer worked; the toolbar combo was still the old log
filter). This change makes the combo the live trace-type selector.

- `ui/main_window.py`:
  - The `debug_level_combo` (items Info/Warnings/Errors/All) is replaced by
    `trace_type_combo`: items `None/Rate probe/Attitude/AltHold/Actuator/IMU`
    with `itemData` 0..5 matching `TraceTypes`, default Rate (1), width 96.
  - `currentIndexChanged` → `_on_trace_type_changed()` → `_write_param_direct(
    ParamIndex.TRACE_TYPE, tt)`: queues a single tag-17 write through the
    existing `_param_write_list`/`_send_next_param` machinery (5 ms cadence).
    Drops the write (with a Warnings log) if a bulk param queue is mid-flight.
  - `_handle_param_packet` (shared tag-17/tag-71 handler) now syncs the combo
    from the FC's stored value via `_sync_trace_combo()` (blockSignals-guarded
    with `_trace_combo_loading`), so connecting/reconnecting re-seeds the
    selector to what the FC actually has. Writes only apply live; persisted on
    the next param commit (tag 72), matching the AGENTS contract.
  - `log_debug`'s level filter no longer reads a combo — fixed `_log_level =
    "All"` (the old filter never actually filtered at its default, and the
    combo belongs to Trace now). Level lock removed: keeps the function's
    behaviour identical to the previous default.
- `protocol_enums.py`: `UNUSED_125` → `ParamIndex.TRACE_TYPE = 125` (comment
  documents the TraceTypes mapping).
- `parameters.py`: index 125 renamed `Unused126` → `TraceType` with a
  descriptive message; `PARAM_DEFAULTS[125]` 0.0 → 1.0 (FC U8 default
  `eTraceRate`); `PARAM_EXPLICIT_LIMITS[125]` (0.0,255.0) → (0.0,5.0) —
  mirrors the FC ParamTable `U8(&pTraceType, eClassExplicit, eTraceNone,
  eTraceIMU, eTraceRate, "enum")` bounds, which is the authoritative
  no-clamp agreement per AGENTS. Display mult 1.0 and type `U8` unchanged.
- **Discourse**: considered writing the combo change through
  `send_params_typed` — rejected, that reads param-window widgets (a bulk
  128-param path). A one-item queue through `_send_next_param` is the minimal
  reuse and inherits the existing serial-open/error handling. Considered
  always-overwriting the FC value on connect (write then readback) — not
  needed: tag-71 readback syncs the combo TO the FC, so user + FC agree after
  connect without GCS writing anything.
- **Status**: py_compile OK (main_window, trace_viewer, parameters,
  protocol_enums); table asserts pass (limit (0.0,5.0), type U8, default 1.0,
  enum 125). Headless-only — the real PyQt5 toolkit interaction (combo signal,
  itemData, blockSignals sync) needs a host run to confirm visually.

## 8. Combo sync-state colour + "does a selector change need a param update?" (cont.)
User decision: the combo must be **orange until it has been actually uploaded or
downloaded from the FC, then green**; re-choosing a type re-arms orange.

- `main_window.py`:
  - `_TRACE_STATE_STYLE` maps `synced` (#27ae60) / `pending` (#e67e22);
    `_set_trace_combo_state()` swaps the combo's text colour + tooltip suffix
    (green = confirmed, orange = awaiting confirmation, empty = unknown).
  - `_on_trace_type_changed` now records `_trace_write_pending = tt`, paints
    orange, then queues the tag-17 write.
  - `_sync_trace_combo(fc_val)` (driven by `_handle_param_packet` for tag-71
    downloads AND the tag-17 echo ACK) resolves the colour:
    * fc_val == combo selection → confirm: clear pending → green;
    * a write is pending for the current selection → still orange (an older
      ACK / unrelated param packet must not falsely confirm);
    * no pending write (a genuine download) → adopt the FC value → green.
    Re-picking mid-flight simply overwrites `_trace_write_pending`, so the next
    matching ACK is the one that turns green.
  - `disconnect()` resets `_trace_fc_value`/`_trace_write_pending` → neutral.
- **Answered question (from FC source, not assumption)**: does a selector change
  need a parameter update/commit to take effect? No.
  * Effect is immediate: `ProcessParamsWrite` (telem.c:1249-1255) writes the
    tag-17 payload straight into the live target variable (`*(uint8*)e->target`
    for U8), i.e. `pTraceType` directly — the next trace cycle uses it.
  * Persistence is **opportunistic**: the write also sets `ConfigChanged` and
    RefreshConfig() (params.c:376) saves the whole config block whenever the FC
    is not in flight — bench flashes nearly immediately, airborne waits for
    landing/disarm. Tag 72 (commit) is only the apply + system-reset flow.
    So the earlier AGENTS phrase "persisted only on the next param commit" was
    wrong and is corrected in AGENTS.md.
- **Status**: py_compile OK. State-machine colour/confirm branches exercised
  headlessly; the actual colour swap and the connect/download sequence need a
  host run to eyeball.

## 9. Second live-run crash: missing `_param_write_progress` (cont.)
First live run with the wired combo:

    AttributeError: 'MainWindow' object has no attribute '_param_write_progress'
    _on_trace_type_changed -> _write_param_direct -> _send_next_param

- **Root cause**: `__init__` never declared `_param_write_progress` (nor
  `_param_write_total`), and `_write_param_direct()` — the new single-write
  path — set only list/port/index/total. `_send_next_param()` reads
  `_param_write_progress` on every send, so any combo change blew up. The bulk
  path (`send_params_typed`) set all fields before use, which is why nothing
  had bitten earlier. Headless verification never ran `_send_next_param`
  through a real write, so the missing attribute was invisible — the same
  lesson as the QPolygonF crash: the stub proves parsing, not runtime paths.
- **Fix**: `__init__` now initialises all six `_param_write_*` invariants
  (list, port, on_complete, progress, total, index); `_write_param_direct`
  sets all six before queuing. Static audit of every `_send_next_param` read
  now shows each field is always set before use.
- **Status**: py_compile OK; attribute audit green. Re-run the combo on the
  host to confirm the write/ACK/green cycle.