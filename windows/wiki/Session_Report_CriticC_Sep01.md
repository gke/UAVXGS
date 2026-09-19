# Session Report — Critic Metrics C Port (Stage 1a tail) (Sep01)

Date: 2026-09-01. Stage 1a tail of the inflight auto-tuning roadmap (AGENTS.md
TODO bullet): port the shared critic measurement to C so the FC (`trace.c`
probe captures) and the sim measure a step IDENTICALLY. The Python
`src/critic/metrics.py` is the specification; `src/tests/ab_critic_metrics.py`
is the Python oracle; this report is the C transcription + its verification.

## 1. What changed

### NEW `UAVXArmQ/src/critic.h` + `src/critic.c`

A faithful C transcription of `critic.metrics.step_metrics_window`, evaluated
on the trace ring's `eTraceRate` capture for the single v1 test axis
(`TRACE_TEST_AXIS = eRoll`). The interleaved ring (Roll/Pitch/Yaw triplet
records at `3·record_size` stride per sample) means the metric reads only the
Roll record of each triplet: timestamp (u32 ms) at slot `+0`, measured `Rate`
(f32 rad/s) at `+4`.

- `CriticEvaluateRateStep(setpoint, struct CriticResult *r)` — computes
  **rise** (first 10 %→90 % cross of `|setpoint|`), **settling** (first
  sample at/after the 90 % cross whose next `min(200, n)` samples all stay
  inside `±2 %·setpoint`), **overshoot** (`max(peak above setpoint)` /
  `setpoint × 100`). Mirrors `step_metrics_window` exactly, including:
  - the `not_reached_s` **sentinel is a parameter** — the FC passes its own
    capture duration `(TraceEndTick − TraceStartTick) · 0.001`, so "did not
    reach" means "not reached within THIS capture" (per the Python docstring).
  - sample count `n` from `TraceCount` via **multiply-by-inverse**
    (`TraceCount · (1/3)`), never a `/3` (division rule).
  - the only `/` is `peakAbove / setpoint` (runtime divisor; the overshoot %
    exception the rule carves out — see discourse).
- `struct CriticResult { RiseTimeS, OvershootPct, SettlingTimeS, NotReachedS }`
  (real32 each) + `CriticLast` / `CriticLastValid` globals to hold the last
  measurement for a future GCS tag / Stage-1b consumer.
- Reads the ring via `fu32_t`-union `memcpy` helpers (`CriticReadU32` /
  `CriticReadF32`) with an explicit bounds guard, mirroring the existing
  `TracePut*` write helpers.

### `UAVXArmQ/src/critic.h` externs

`critic.h` declares the trace-ring state it consumes (`BBQ`, `TraceCount`,
`TraceSnapStart`, `TraceStartTick`, `TraceEndTick`) as `extern` — they are
non-static globals in `trace.c`.

### `UAVXArmQ/src/UAVX.h`

Added `#include "critic.h"` (after `trace.h`; `misctypes.h` already included,
so `real32`/`boolean` are defined for the header).

### `UAVXArmQ/src/trace.c` — wired into `TraceCommit()`

`TraceCommit()` (disarm, both landed + shutdown paths) now calls
`CriticEvaluateRateStep(TraceStimValue, &CriticLast)` after the flash commit
and sets `CriticLastValid = true`. `TraceStimValue` is still valid at disarm
(the rate step injected during the capture), so it is the metric setpoint.
The RAM ring is intact at this point (reused only on the next arm), so the
measurement reads the just-captured step before it is overwritten. Runs on
disarm with motors stopped — safe for a few-thousand-iteration loop.

**Time base (corrected):** `ts = (tick − TraceStartTick) · 0.001` — the metric
measures time **relative to capture open**, matching the sim/Python time base
(`times` near 0 s at the step). The initial port used absolute wall-ms
(`tick · 0.001`), which would have offset settle-time by the wall clock at
capture open. Fixed 2026-09-01; parity re-verified (below).

Scope per user decision (2026-09-01): **metric code only** — no telemetry /
protocol change yet (GCS retrieval is Stage-1b); **rise/overshoot/settle only**
— the zero-crossing oscillation count is deferred because `eTraceRate`
records Rate, not the angle series `count_zero_crossings` needs (v2 extended
record rows add the angle series).

### GCS trace viewer — reference metrics on rate dumps

`ui/trace_viewer.py` gains `rate_step_metrics(parsed)` + a header readout for
`eTraceRate` dumps (rise/overshoot/settle on the Roll-test-axis rate series,
setpoint = peak Roll Desired, `not_reached_s` = capture duration), computed via
the shared `critic.metrics.step_metrics_window`. This is a **ground-side
reference** only, for cross-checking a dump against what the FC reports once a
transport exists — it is NOT a competing on-board path (see discourse). The
`src/tests/generate_trace_samples.py` `rate_probe` was made faithful to the
real FC probe: **Desired is a constant step** (`ds` after pre-roll), the gyro
Rate is the plant response — previously Desired followed the response curve,
which broke the `max(desired)`-as-setpoint assumption (settle could never
converge to a mis-derived target).

## 2. Rationale and discourse

### Why a dedicated `critic.c` rather than the dormant `tune.c`
`tune.c` is the *tuning-application* scaffold (apply gains, TuningScale ramp)
— measurement is a different concern. A standalone `critic.c` mirrors the
Python `src/critic/` package 1:1, keeping the two-source-of-truth
relationship explicit and the module build-automatically included (the build
glob's every `src/**/*.c`, so no link-list edit — confirmed via the link line
`src/critic.o`).

### Faithfulness vs. "sharpen simulation fidelity"
The user's broader note — the Ecks tuning/flight experience shows we still
have a way to go on simulation fidelity — does not change this port's job: the
metric must be *identical* between FC and sim so any future fidelity gap shows
up as a plant discrepancy, NOT a measurement one. Improving the sim plant is
separate (and already mirrored in the TODO's "sim-blessed deltas then confirm
in air" STAGE‑2 abort rule: abort if identification deviates >±30 %).

### Ship parameters to the Tx, not the log (user direction, 2026-09-01)
The point of extracting the response on board is to ship the **three floats**
(rise/overshoot/settle), not the trace. This avoids hammering flash with large
logs and needs only a tiny downlink — and it opens up running **a range of
tests in one flight** (chained per-axis / re-triggered captures, each pushing
just its 3-number result to the Tx). This makes `CriticEvaluateRateStep` the
**authoritative on-board path**: it keys on the **commanded `TraceStimValue`**
it injected (independent of the record), so it is exactly the "ship 3 floats"
extractor. The FC computes one 3-float `CriticResult` per capture; a future
Stage-1b carrier (e.g. the CRSF RPM->SD path, or a tag) forwards it. The GCS
trace viewer's `rate_step_metrics` is explicitly a ground *reference*, not the
transport — it re-checks a pulled dump against the FC's reported number rather
than competing with it. No flash/telemetry pressure from the on-board path.

### Division-rule application
AGENTS.md states: never divide by a **constant/literal** when you can multiply
by its inverse; division remains only where the divisor is a **runtime value**
that cannot be precomputed. Applied here:
- `TraceCount / 3` (structural record-count → sample-index scaling) — the
  divisor `3` is a constant → replaced by `TraceCount · (1/3)`, which is exact
  because `TraceCount` is always a multiple of 3.
- `peakAbove / setpoint` — `setpoint` is a runtime value (the injected rate
  step, which varies per airframe / rate limit); it is a genuine `÷runtime`
  kept as `/setpoint`. This is the same exception AGENTS.md already names
  ("the critic-metric overshoot % (`peak/setpoint`)").

### Sentinel design
The Python default `STEP_NOT_REACHED_S = 8.0` is the *sim's* SIM_TIME. An FC
capture is only ~2.7 s at 500 Hz (~1364 Roll samples), so a phase that never
completes must be reported against the FC's own window == capture duration —
exactly the Python contract (`not_reached_s` is a parameter). Storing
`NotReachedS` in the result makes the sentinel used explicit and self-describing.

### Bounds / single-exit / size
- Ring reads are guarded (`off + 4 <= TRACE_RING_SIZE`) and the settle lookahead
  is clamped `(s + k) < n` (rule 5).
- Single logical exit (`CriticEvaluateRateStep` has one terminal `}`); the
  early `break` from the outer scan on `settled` is a loop control, not a
  function return (matches the Python's `break`).
- Function is 82 lines, under the 100-line ceiling, with the two read helpers
  factored out.

## 3. Verification

- **All 6 FC boards build cleanly**, including the flight target:
  UAVXF4V3 (231308 B), UAVXF4V4 (230852 B), DEVEBOXF4 (231092 B),
  SPEEDYBEEF405WING (232596 B), FLYINGRCF4WINGMINI (232828 B),
  BLUEBERRYF405 (233260 B — `critic.o` now linked). `fc_build.py` echoes
  `=== <BOARD> OK ===` for all six. Re-run after the time-base fix: still all
  green.
- **Numerical parity vs the Python oracle** — no host C compiler exists in the
  Flatpak runtime, so the C algorithm could not be executed natively. Instead,
  the **exact C control flow** (interleaved ring reads, ms→s conversion
  **relative to capture open**, multiply-by-inverse `n`, thresholds)
  was re-transcribed to a Python harness that builds the same synthetic
  byte-ring `critic.c` reads and diffs against the trusted
  `step_metrics_window`. Across four step shapes — underdamped-with-overshoot,
  overdamped, fast oscillation, short capture — plus the `setpoint=0`
  no-measure guard, the C-equivalent matches the Python to <1e-6 on every
  field, **including with a non-zero capture-start tick** (the case the time-
  base fix is about).
  *TODO: when a host C toolchain is available, compile `critic.c` under a stub
  `UAVX.h` and assert equality against `ab_critic_metrics.py` (stub + `main.c`
  prepared in `/tmp/opencode/critic_test/`).*
- **Synthetic commissioning (GCS)**: `generate_trace_samples.py` (with the
  constant-step Desired fix) reproduces `src/tests/trace_samples/trace_rate.bin`;
  parsing it and running `step_metrics_window` on the Roll rate series yields a
  physically-consistent result: setpoint 2.0 rad/s, overshoot 10.0 %
  (theoretical for ζ=0.62), settle 0.34 s, rise ~0 s (fast 25 rad/s plant at
  2 ms sampling). `python3 -m py_compile src/ui/trace_viewer.py` clean (PyQt5
  runtime unavailable in the Flatpak venv, so the widget itself is not executed
  here).

## 4. Backtrack / next steps

- **Transport (Stage-1b) — user direction: the 3-float `CriticResult` is
  recorded to flash AND streamed.** The FC writes it as a 24 B critic-result
  trailer in the same disarm flash commit (so the GCS dump reads it directly —
  radio-independent, works on Spektrum) and streams it over telemetry where a
  capable radio exists (ELRS/D8). Live telemetry carrier still pending user
  approval; the flash/dump path is DONE (section 6). Enables multiple tests per
  flight and direct GCS processing with no trace re-derivation.
- **GCS side** of the transport: `rate_step_metrics` reads the trailer
  (`source: 'fc'`) and falls back to re-deriving only when no trailer exists.
- `count_zero_crossings` C port deferred until the v2 `eTraceAttitude` /
  extended record rows provide the angle series it operates on.
- The dormant `tune.c` scaffold remains the application home for Stage 1b
  (identification → proposed gains vs. the criteria), gated by the same
  externalised thresholds.
- Sim-fidelity improvement (user's broader point) is tracked separately (TODO:
  real-inertia resonance, sensor noise model, prop/thrust data) and must stay
  decoupled from measurement identity.
- Ecks 4 Hz oscillation is **parked** (2026-09-01, user): no tuning change yet.
  Next flight first assesses the integral-reset + prop-sense reflash before any
  rate-P/rate-D touch. When resumed, the prediction stands: revert rate-P
  toward old or raise rate-D ~0.020 (see `Session_Report_EcksOscillation_Sep01`).

## 5. Trace-to-flash lifecycle + dump availability (agreed, 2026-09-01)

The dump facility is **retained** as the human waveform-viewing path (separate
from the on-board 3-float extractor). Persistence/priority behaviour agreed and
recorded:

- **Flash write at disarm only** when `pTraceType != eTraceNone` **and** a
  capture actually ran (`TraceCount > 0`). Default (`None`) never writes;
  enabling trace but never firing ch8 never writes.
- **Flash is not scrubbed on arm** — the CAPTURE sector holds the last
  disarm-with-capture indefinitely until a *new* disarm-with-capture
  overwrites it.
- **Dump source priority** (`StartBBChunkedDump`): live RAM ring if
  `TraceCount > 0`, else the committed flash copy.
- **Consequence (the "good trace without your GCS" scenario):** after a power
  cycle the RAM ring is empty, so a dump reads the flash copy regardless of
  armed/disarmed — a bench dump yields the good trace. **Accidental arming is
  harmless** (arming alone neither populates the ring nor touches flash); the
  trace only disappears if you *fire a new capture (ch8 in flight) and disarm*,
  which overwrites the sector. The GCS Dump button is **not** gated on armed
  state (FC `telem.c` `miscBBDump` → `StartBBChunkedDump()` runs unconditionally).
- **Flash endurance is a non-issue at realistic rates:** F4 spec 10k
  write/erase cycles on the dedicated CAPTURE sector (0x80C0000, top-of-image
  reserve) → tens of years at even dozens of flights per outing. Power-loss
  mid-write is benign (capture is non-boot-critical, outside the config/firmware
  image). Open sub-question (user: decide separately): whether to keep persisting
  to flash at all vs RAM-ring-only, now that the dump path is human-view only.


## 6. Critic-result trailer: floats to flash + dump (implemented 2026-09-01)

### What changed

The earlier "waves→flash, floats→telemetry only" split was **revised**: the
3-float `CriticResult` is now persisted to flash (in the same disarm write) AND
streamed. Rationale (user, 2026-09-01):

- **Not all aircraft have a telemetry path.** Spektrum/SRXL2 cannot stream the
  custom floats; ELRS/D8 can. Persisting the floats to flash alongside the
  waveform gives **uniform coverage** — a dump reads them directly on any radio.
- **Direct GCS processing.** Having the floats in the dump means the GCS can
  process them further without a trace re-derivation step.
- **The trailer needs metadata to say what they apply to** — the axis and the
  commanded stimulus, so v2 multi-axis/multi-test captures disambiguate.
- **Flash cost is negligible**: the trailer is a second
  `WriteBlockArmFlash(false, ...)` program on the SAME erase/program cycle as
  the records — no extra erase, no separate write, no endurance concern (the
  sector already has ~decades of margin at realistic rates).

### Design

- **24 B trailer** (not in the 16 B record basis; lives only in the flash copy
  and the dump stream, not the CCM ring):
  `"CRIC"`(4) ver(1) axis(1) flags(1) reserved(1) StimValue f32(4)
  RiseTimeS(4) OvershootPct(4) SettlingTimeS(4).
- **NotReachedS is NOT stored** — it equals this capture's duration, already in
  the header (endTick − startTick), so the GCS derives it there. (Initial cut
  wrote 5 floats = 28 B and overflowed the 24 B buffer — caught before build;
  reduced to 4 floats.)
- `TraceBuildCriticTrailer()` serializes from `CriticLast` + `TraceStimValue` +
  `TRACE_TEST_AXIS`. `TraceCommit()` calls it, extends `TraceCommitLen` by 24,
  writes records (erase=true) then trailer (erase=false). `StartBBChunkedDump`
  streams the trailer after the records via a new `BBChunkTrailerLeft` (byte-
  wise, may straddle a chunk boundary, so NOT folded into the record loop);
  flash source re-reads the trailer into the same RAM buffer so both sources
  stream identically.

### GCS

- `parse_trace` reads the trailer when the `"CRIC"` magic/version match and
  exposes a `critic` dict (axis, flags, setpoint, rise, overshoot, settle).
- `rate_step_metrics` **prefers the trailer** (`source: 'fc'`, with `axis`);
  falls back to re-derivation (`source: 're-derived'`) when absent.
- Viewer header shows the source string + axis name.
- Synthetic generator now appends a self-consistent trailer to the rate dump
  (derived from the same series via `step_metrics_window`), so a viewer/test
  sees FC-trailer floats agree with ground re-derivation exactly.

### Verification

- **FC**: all 6 boards compile/link clean (`=== <BOARD> OK ===`) with the
  trailer + dump changes (trace.o/critic.o rebuilt). No errors/warnings from the
  trailer path.
- **GCS**: `py_compile` clean on `trace_viewer.py` + generator. Headless test
  (PyQt5 stubbed) confirms: rate dump carries the trailer, `parse_trace` reads
  it, `rate_step_metrics` reports `source: 'fc'` with setpoint 2.0 / overshoot
  10.0 % / settle 0.34 s, and the trailer values match re-derived to 0.0
  (generator derived them from the same series). Attitude dump (no trailer)
  still falls back to `source: 're-derived'`.

- The ring source emits the trailer **only when that ring has been committed**
  (`TraceCriticValid`, set in `TraceCommit`, reset in `InitBlackBox`) — a
  pre-commit live dump carries no trailer and the GCS re-derives, so it never
  pairs a stale trailer against a fresh ring. The flash source always reads its
  own stored trailer (inherently self-consistent with the flash records).

### Remaining (pending user)

- **Live telemetry carrier** for the floats (ELRS/D8 in-flight visibility) —
  transport pending user approval (CRSF RPM→SD vs a tag vs D8 burst). The
  flash/dump path is complete and radio-independent.
