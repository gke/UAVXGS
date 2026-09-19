# Session Report — Shared Critic Package + Trace ch8 Label (Aug30)

Date: 2026-08-30. Stage 1a blocking gate of the inflight auto-tuning roadmap
(AGENTS.md TODO bullet): **externalise the critic metrics from
`test_pid_sim.py` into a shared `src/critic/` module so the FC (`trace.c`)
and the sim measure a response identically.**

## 1. What changed

### NEW `uavx-python/src/critic/` package (Stage 1a gate)

- `critic/metrics.py` — pure measurement over `(times, values)` series,
  lifted verbatim from the inline blocks in `tests/test_pid_sim.py`:
  - `StepMetrics` / `DisturbanceMetrics` dataclasses (moved, unchanged field
    layout).
  - `step_metrics_window(times, values, setpoint)` — the angle-step / rate-step
    definition: 10→90 % rise, `min(200, len)`-sample windowed settle at
    `±2 %·setpoint`, overshoot above setpoint; `not_reached_s` sentinel.
  - `step_metrics_persist(times, values, setpoint, deadtime_s=1.0, dt)` —
    the altitude-hold / navigation (linear, metres) definition: first
    `≥0.9·sp` rise, persist-until-end settle survey with the incremental-loop
    `t + dt` bookkeeping, `t > 1.0` gate replicated.
  - `disturbance_metrics(times, rates, start_s, dur_s, dt)` — gust test:
    peak `abs(rate)` over the run, IE over the **inclusive** `[start, 2×end]`
    window (the original sim's `<= gust_end_idx * 2` quirk, preserved on
    purpose — see discourse), settle relative to gust end at
    `±(peak·5 % + 1e-6)`.
  - `count_zero_crossings(values, start_index)` — the oscillation counter
    (same-sign-run scoring; warmup cutoff as a sample index).
  - `STEP_NOT_REACHED_S = 8.0` — the sim's `SIM_TIME` (verified `test_pid_sim.py:766`).
    Deliberately a parameter: an FC capture is only ~2.7 s @ 500 Hz, so the C
    port passes its own window and "did not reach" means "within MY window".
- `critic/criteria.py` — pass/fail thresholds + verdicts + report layer:
  - `Criteria`, `DisturbanceCriteria`, the `CRITERIA` / `FW_CRITERIA` /
    `DIST_CRITERIA` / `AH_CRITERIA_MR` / `AH_CRITERIA_FW` / `NAV_CRITERIA_MR` /
    `NAV_CRITERIA_FW` tables (values unchanged) — moved from the sim.
  - `Verdict` row + `step_verdicts(m, c, linear)` / `disturbance_verdicts(m, c)`
    — unit-correct comparator ("display value already in compared units"),
    maps 1:1 to future C comparison rows.
  - `check`, `critique`, `critique_linear`, `critique_disturbance`,
    `recommend`, and the terminal colours `G/Y/R/B/N` were also moved here
    (they are rendered by these functions), re-exported for every consumer.
- `critic/__init__.py` — public API hub re-exporting both modules.

### `uavx-python/src/tests/test_pid_sim.py` (surgical, no behaviour change)

- Imports the moved pieces from `critic.metrics` / `critic.criteria`; the six
  inline metric blocks (angle step, rate gust, AH-MR, AH-FW, Nav-MR, Nav-FW,
  heading-nav, coupled-3-axis) now call the shared extractors on the **same
  series** the inline code consumed (decimated for the windowed defs, full-res
  for the persist/gust defs).
- Deleted local `StepMetrics`/`DisturbanceMetrics` dataclasses, the local
  `Criteria`/`DisturbanceCriteria` + all criteria tables (they were shadowing
  the imports), and the local critique/check/recommend functions. The file now
  defines **no** metric or verdict locally — single source of truth.
- Coupled sim's `n_oscillations` stays 0 (unchanged); `simulate_axis` now
  feeds `count_zero_crossings` the full-res `angles_full` (the original's
  crossing logic ran on decimated samples — see discourse).

### GCS label

- `ui/parameter_window.py` RC_MAP_PARAMS: `(RX_AUX4_CH, "Aux2", 8)` →
  `"Trace"` — the Ch8 row in the RC Configuration group labelled the *Aux2
  function*; ch8/Aux2 is now the Trace trigger (Trace section, AGENTS.md), per
  user request (user: "Ch8 … still labelled Aux2. It should be Trace.").
- `ui/main_window.py` FC-function label list (index 8 = `eAux4RC`, the trace
  trigger) `"Aux2"` → `"Trace"`. `protocol_enums.RC_MAPPING_NAMES` (the
  physical-channel option names Aux1..Aux7) intentionally left alone.

## 2. Rationale and discourse

### Why a shared module at all
The FC trace probe is deliberately now measuring the SAME physical quantities
(Rate, Desired, Out) the sim critiques. If rise/settle/overshoot/IE definitions
differ between sim and FC, the whole "sim-blessed deltas then confirm in air"
(STAGE‑2) argument collapses — you could not tell whether a discrepancy was
plant or measdurement. Externalising both the extractors and the criteria
tables is the blocking gate (Stage 1a); the C port (`metrics → C later`) then
becomes mechanical: same thresholds, same window, same sentinel.

### What "identical" meant for the refactor — and what the regression guard proves
A pure move cannot be validated by "the numbers look the same"; they had to be
**bit-faithful**. `src/tests/ab_critic_metrics.py` (kept in-repo as a permanent
guard) replays the ORIGINAL inline math, copied verbatim from the pre-refactor
source, against the shared extractors over representative synthetic series
(damped step, overdamped, heavy overshoot, never-settles, alt-hold rise/settle,
gusts of three durations, full-res and decimated oscillations). Result:
**all equivalent**, including the awkward corners:

- **`SIM_TIME` sentinel = 8.0 s, not 24 s.** The harness initially assumed 24 s
  (a stale memory of the flight-time default); `test_pid_sim.py:766` says
  `SIM_TIME = 8.0`. The "never settles" case therefore reports 8.0 s — which is
  exactly the sim's `SIM_TIME`, and the correct "out of window" value.
- **IE window `[start, 2×end]` inclusive.** The original disturbance loop
  accumulated `if gust_start_idx <= i <= gust_end_idx * 2` with raw index
  `i` — a genuinely odd definition (integration runs to *twice* the gust-end
  time). We preserved it. Rationale: the sim's baseline reports (critique
  numbers, PASS/FAIL edges) were produced by this definition; and the FC port
  implements one number either way. Changing it now would move every
  disturbance-IE metric and any downstream PASS edge — a noise change with zero
  information gain. When that metric is deliberately redefined, it is a single
  documented edit here, visible to both consumers. (The harness initially got
  this wrong — a Python-exclusive `range(start, 2*end)` vs the original
  inclusive `<= end*2` — the 1.2e-3 delta in the 0.5 s-gust case is exactly
  the missed tail sample `|rate(3.0s)|·dt`, confirming the shared function.
- **Overshoot/rise replicate the exact signed-value arithmetic** (`setpoint`
  sign-aware, abs(peak) for the linear case), so the *critique* reads exactly
  as before.

### Full-run outcome
`python3 src/tests/test_pid_sim.py` (all 15 `generic/` frames × slider
extremes) → **identical to pre-refactor**: same 3 FAIL lines, byte-for-byte —
Delta 8.0 s rise sentinel (never rose within window), and two marginal
gust-settle fails (Elevon 1.576 s, Spoileron 1.713 s vs 1.5 s). Those are
baseline tuning characteristics, not measurement artifacts.

### Zero-crossings: full-res vs decimated — and why it is acceptable
Original `simulate_axis` scored crossings on *decimated* angle samples (every
`decimate=5` steps). The refactor scores the full-res `angles_full`. A
dense-versus-thinned sampling can only disagree if the loop rings above
~½·decimated-rate (≈50 Hz at the 20 ms decimation) — physically impossible for
an attitude loop that still converges to the 0.02 % band (a crossing pair
between two thinned samples requires the ring frequency to exceed the Nyquist
of the point spacing; it would fail settle long before). The full-run FAIL set
contains no oscillation-row changes, confirming equivalence in practice. We
kept full-resolution scoring because it is what the FC trace (500 Hz records)
will inherently have — scoring FC data "as if decimated" would be measuring
identically to a fiction.

### GCS label scope
Only the *function* namespace renamed (Ch8 row in RC Configuration,
`eAux4RC` label list). The *physical-channel* option names
(`RC_MAPPING_NAMES`: "Aux1".."Aux7") are the names of the radio's channel
inputs, not functions — a different namespace; renaming those to "Trace" would
corrupt `RxAux2Ch`-style "which channel" selection UI. Note the param
`parameters.py:71` `RxAux2Ch "Aux2 channel mapping"` and
`convert_old_defaults` comments reference a *different* param (the RC-input
selection), left untouched.

## 3. Verification

- `python3 -m py_compile` on `critic/{__init__,metrics,criteria}.py`,
  `tests/test_pid_sim.py`, `tests/ab_critic_metrics.py`,
  `ui/{parameter_window,main_window}.py` — all clean.
- `python3 -c "import critic; assert …"` — package API surface verified
  (`STEP_NOT_REACHED_S==8`, table values, callables present).
- `src/tests/ab_critic_metrics.py` → **ALL EQUIVALENT** (exit 0).
- `src/tests/test_pid_sim.py` full generic sweep → exit 1 (pre-existing FAILs,
  see above), FAIL lines identical pre/post refactor.

FC build not touched in this session (no C changes).

## 4. Backtrack / next steps

- **Stage 1b trigger semantics unchanged**: capture stays armed per AGENTS.md
  (ch8 `Aux2RC`, pilot-armed; write-on-disarm only).
- **Stage 1a tail**: the actual C port of these four functions
  (`step_metrics_window` → rise/settle over the 4094-record ring, plus
  `count_zero_crossings`) on the FC, evaluated on the `eTraceRate` capture at
  `TraceCommit()`/disarm or on a dump via tag‑54. The C port is now a pure
  transcription — the Python is the specification and the regression guard is
  the oracle.
- **Pending approvals awaiting the user** (unchanged from AGENTS.md): capture
  carrier choice (USB-tag first vs D8 burst vs CRSF — see the CRSF feasibility
  report in this directory), probe auth, in-air APPLY tier vs write-on-disarm
  baseline.