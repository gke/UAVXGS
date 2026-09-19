# Inflight Plant Identification — Design Handoff (identify.c/identify.h)

**Date:** 2026-09-14
**Status:** DESIGN COMPLETE + IMPLEMENTED (see §11). This report is the spec
for a takeover session; §11 records the implementation and its deviations.
**Author of design:** previous assistant session with Greg (Prof).

This design answers the question: *"Given an extensive flight log covering RC
controls, accs, gyros and servo responses, can a model of the aircraft be
approximated?"* — **yes, continuously in flight, as a passive KF-based
identifier.** The session converged on: continuous → no fixed window is needed
if you use a scalar-Q parameter KF (whose equality with RLS+forgetting makes the
"window" question moot, see §4). The take-over session's mandate: **build
`identify.c`/`identify.h` exactly as specified here, FC-only, passive
observation, validate against the emulator plant, and wire the distilled
parameters into the trace dump so the GCS can read them.**

---

## 1. Objective

A passive, always-on, per-axis **plant model identifier** on the FC:

- Estimates a **first-order discrete plant** `Rate[k+1] = a·Rate[k] + b·Out[k]`
  per axis (Roll/Pitch/Yaw), at the 500 Hz control rate, via a **Kalman-Filter
  form of Recursive Least Squares (KF-RLS)** with a scalar process-noise factor.
- **Predicts** the rate the current plant model expects, **compares** it with
  the measured gyro `Rate`, and **tweaks** the model parameters `(a, b)` 
  accordingly — exactly Greg's "predict → compare → tweak" tracking loop.
- Runs identically in **emulation** so the emu plant (known `MotorTau`,
  authority) acts as the ground-truth validator.
- **Passive:** it never injects a stimulus, never gates on ch8, takes no flash
  action, writes nothing to the airframe. It simply reads `A[a].Out` (the
  control output actually computed this tick, post-`RateGainScale` and
  post-`conditionOut`) and `Rate[a]` (measured gyro) and refines its model.
- Exports the distilled `(a, b)` + confidence to the GCS via a **v3 trace header
  block** (rides the existing tag-54 dump — zero new protocol tags in v1).

**Out of scope (v1):** no adaptive gain application, no on-board critic, no
autotune loop, no live telemetry tag, no GCS parser (beyond reading the v3
header block).

---

## 2. Why a Kalman Filter (the architecture insight)

Greg's intuition ("predict what will happen then tweak predictions over time –
sounds almost like a KF?") is **exactly right**. RLS with a forgetting factor is
**a KF over the model parameters with random-walk dynamics**:

| RLS concept | KF equivalent |
|---|---|
| forgetting factor λ | process-noise **Q** on the parameter states |
| covariance matrix P | same P (parameter covariance) |
| Kalman gain K | same K (innovation weighting) |
| parameters θ | augmented states (or the state vector itself) |

We *do not need* a separate fixed-size sliding window: the forgetting/adaptivity
is carried by a **small diagonal Q on the two parameters** in the KF prediction
step. This is the clean, principled formulation and costs O(2²) per axis.

---

## 3. The identification model (from the FC plant)

The FC already has a faithful plant model in `emu.c` — use its *structure*, but
the identifier must work on the **real aircraft** where `MotorLagState` is not
observable. For every axis the airframe exhibits a **lumped first-order-plus-
authority** response:

```
Motor lag (emu.c:462)      ML[k+1] = ML[k] + α·(u[k] − ML[k]),  α = dT/(τm + dT)
MR rate   (emu.c:476-489)   Rate[k+1] = Rate[k] − (Kt·ML·Out + 2·|r|·r)·dT
FW rate   (emu.c:430-448)   Rate[a] −= (Out·R.Max·I + 2·|r|·r)·dT   (r/p)
                           Rate[eYaw] += (Out·R.Max·I − 2·|r|·r)·dT
```

Linearising/lumping at an operating point, each axis satisfies a
**first-order ARX(1,1) with input delay** (`Out[k]` drives `Rate[k+1]`):

```
Rate[k+1] = a·Rate[k] + b·Out[k]           (per axis)
```

- `Out[k]` = `A[a].Out` from THIS tick (already includes `RateGainScale` and
  `-conditionOut`, so no gain un-scaling is ever needed — the identifier sees
  the same signal the plant sees).
- `Rate[k]` = measured gyro rate this tick.
- **Interpretation:** `τ_eff = −dT / ln(a)` is the lumped motor/rotor/aero
  first-order lag+aero-damping constant at the current operating point; the DC
  gain `b/(1−a)` is the authority (incl. the quadratic-damping linearisation,
  so it is amplitude-dependent — the identifier tracks this drift via its Q).
- The quadratic damping term is **not** separately estimated in v1 (it is
  absorbed into the operating-point `a`). This is a documented simplification —
  a follow-on EKF could estimate it explicitly. The ARX(1,1) correctly captures
  the *effective* loop plant the controller stabilises, which is what matters
  for validating/feeding `test_pid_sim.py` and the emu.

---

## 4. Window size — the direct answer

Greg's question: "continuous process in flight, or a moving window? If so, how
big?"

**Answer: continuous — no fixed window. Adaptivity (the forgetting) is carried
by a scalar `Q` on the two parameter states.** This is the KF/REC-isomorph of
forgetting; the notion of "window" is only a derived, approximate figure:

- RLS with forgetting λ ↔ KF with parameter process-noise `Q_p`.
- Effective memory ≈ `1/(1−λ)` samples. At 500 Hz, **λ = 0.999 gives an
  effective ~2 s / 1000-sample window**; λ = 0.995 gives ~0.4 s.
- In KF terms the equivalent scalar `Q_p` is tuned so the estimate tracks
  plant drift (battery sag, airspeed change, motor heat) over a few seconds
  while still integrating across controller settle (`τ_eff` of the rate loop
  is tens of ms; the rate-loop angle settle is 0.3–0.8 s, so a ~1–2 s effective
  memory covers ≥1 full settle — the design guideline from the session).

**Recommended starting values** (tune on the emulator first):
- Measurement noise `R = gyro variance ≈ (0.000873)² ≈ 7.6e-7` (rad/s)² — the
  `cEmuGyroBiasRadS` figure in emu.c; real gyro noise is comparable.
- Parameter process noise `Q_p ≈ 1e-8 … 1e-6` per state, per tick (scaled to
  the parameter magnitudes `a≈0.98`, `b≈small`).
- Initial covariance `P0 = diag(0.1, 0.1)` (parameter-space scale).
- **Excitation gate:** only update when the regressor carries energy, e.g.
  `|Out[k]| > deadband AND |ΔRate[k]| > deadband` (manoeuvring). During a
  hover/steady-state the KF survives with no update — P stays put, no bad
  fit is baked in. This replaces a sliding-window length with a quality gate.
  (The KF covariance also naturally handles this: with no excitation the
  innovation carries no information, so K shrinks — but an explicit gate also
  prevents P contracting to an over-confident bad fit during a hover.)

---

## 5. `identify.c` / `identify.h` — API spec

### 5.1 `identify.h`

```c
#ifndef _identify_h
#define _identify_h

// Inflight passive plant identifier. KF-form RLS on a per-axis first-order
// plant Rate[k+1] = a·Rate[k] + b·Out[k], fed every 500 Hz control tick with
// the measured gyro Rate[a] and the controller output A[a].Out as computed
// this tick. The KF predicts the rate the model expects, compares with the
// measured rate, and tweaks the (a,b) pair (Greg: predict -> compare -> tweak).
// Purely observational: no stimulus is injected, no flash is touched
// (identification state is RAM-only), and emulation feeds it identically so
// the emu plant validates the estimate. Adapted 2026-09-14 by design (see
// Session_Report_InflightPlantID_Sep14.md).

enum IdentifyState {
    eIdentIdle = 0,        // not yet seen a usable (Out, Rate) pair
    eIdentTracking = 1,    // estimating; confidence below threshold
    eIdentConverged = 2    // parameter covariance small enough to trust
};

typedef struct {
    real32 P[2][2];        // parameter covariance (2x2)
    real32 a, b;           // identified plant parameters
    real32 q;              // scalar process noise on both params
    real32 r;              // measurement noise (gyro variance)
    real32 P0;             // initial covariance magnitude
    real32 sigA, sigB;     // diagonal sqrt(P) confidence bounds
    uint32  n;             // number of updates performed
    uint8   State;         // enum IdentifyState
    real32  prevOut;       // Out[k] held for the k -> k+1 pairing
} IdentifyAxisStruct;

typedef struct {
    IdentifyAxisStruct Axis[eYaw + 1];   // Roll, Pitch, Yaw
    real32 dT;
} IdentifyStruct;

extern IdentifyStruct Identify;

void IdentifyLoop(real32 dT);   // called from DoControl() after the rate loop
void IdentifyReset(idx a);      // zero a single axis (arming / mode change)
void IdentifyInit(void);        // zero all axes, set defaults

uint8 IdentifyCluster(real32 *tau, real32 *gain, uint32 *samples);
    // convenience: tau = -dT/ln(a), gain = b/(1-a), per current best axis
    // (single-exit; caller passes arrays of size eYaw+1). Not required in v1.

#endif
```

(Trim `IdentifyCluster` and the extra fields if you prefer a leaner first cut —
the mandatory exports are `IdentifyLoop`, `IdentifyReset`, `IdentifyInit`,
`Identify` struct, `IdentifyAxisStruct` with at least `a, b, P, q, r, n, State`.)

### 5.2 `identify.c` — algorithm

Per tick (`IdentifyLoop(dT)`), for each axis `a = eRoll..eYaw` (Skip
over `a` where the category does not fly that axis — but roll/pitch/yaw all
have `A[a].Out` on MR/FW; `eCatLand` is skipped entirely):

```
u = A[a].Out;            // this tick's controller output (post-scale)
y = Rate[a];             // measured gyro rate this tick
xold = prevOut;          // Out[k-1] (paired with Rate[k])

// Predict: model's expectation of today's rate using yesterdays output
pred = a_est·y_prev + b_est·xold;   // y_prev = previous tick's Rate

// Update KF-RLS (2x2; classic equations, keep one exit per function):
//   H = [y_prev, xold]            regressor
//   S = H·P·H^T + r
//   K = P·H^T / S                 (scalar divisor S allowed: runtime value)
//   a,b += K·(y − pred)           innovation correction
//   P   = (I − K·H)·P + Q,  Q = diag(q,q)

// Excite gate: skip the update (only refresh prev*) when
//   |xold| < deadband AND |Δy| < deadband     (hover / no manoeuvre)
// so a quiet period cannot collapse P onto noise.

prevOut = u;  y_prev = y;  n++;
```

Store `sigA = sqrtf(P[0][0])`, `sigB = sqrtf(P[1][1])`; set `State`:
`eIdentTracking` until `n > IDENTIFY_MIN_SAMPLES` (e.g. 200) AND
`(sigA < 0.01f) && (sigB < 0.01f)` → `eIdentConverged`.

**Numerical notes (Coding Standards compliance):**
- Single division is `P·Hᵀ / S` — `S` is a runtime scalar, so this falls under
  the allowed division exception (see AGENTS "No division operators … only
  exceptions: divisions inherent to the math and cannot be avoided").
- `ln(a)` is only needed at *export/display* time (GCS or `IdentifyCluster`),
  not per tick — keep the per-tick loop to `+`, `−`, `·`, and the one `/`.
- Use `real32` throughout (FPU, single-precision, consistent with codebase).
- All state is **static/global in RAM** (no malloc). `IdentifyStruct` is the
  single global, `CCM_RAM` optional but unnecessary — RAM is fine.

### 5.3 Hook point (exact)

Call `IdentifyLoop(dT);` at the **end** of `DoControl()` in
`UAVXArmQ/src/control.c` (function is at control.c:692-720, after the
`switch (pAFTypeCategory)` at line 699-715, i.e. **after** `A[a].Out` has been
finalised for ALL axes by `ControlRate`/`ControlRateYaw`/`DoRateDampingControl`
lines 470/482/499 — the identifier must see this tick's output to pair it
against the next tick's measured rate). Place it next to the existing
`TraceCapture()` call (line 693) but after the switch, not before. `DoControl`
is called at 500 Hz from `uavxarm-v3-gke.c:249` — no change needed there.

`IdentifyReset(eRoll..eYaw)` on arming transitions is *optional* in v1 (the Q
keeps the filter adaptive), but **`IdentifyInit()` must be called from the
one-time init path** (alongside `InitControl()` / `InitBlackBox()` in
`uavxarm-v3-gke.c` init sequence, or from `InitControl` in control.c).

Emulation (`emu.c` `DoEmulation`, called on the same loop) consumes
`A[a].Out` and writes `Rate[a]` — so **identify works unmodified in the
emulator** and should recover `τ ≈ MotorTau` (0.10 MR) and the emu authority.

---

## 6. How identify interacts with trace / replaces it

**Greg's question:** "If it works we do not need to dump traces to the GCS,
correct?"

**Short answer: correct — and that is the whole point.** Trace currently ships
~24 kB/s of raw 16 B rows (Rate/Desired/Out × 3 axes) via tag-54. Identify
distills that into **6 floats + count n per axis** (`a, b, sigA, sigB`) — the
*same information the raw waveform would give you after analysis*, but
pre-digested on the FC. When identify converges there is no need to transport
raw waveforms to the ground.

**But do NOT retire the dump path in v1.** The trace remains the ground-truth
sanity channel:
- It cross-checks the identified model — a residual overlay (predicted vs
  recorded `Rate`) is how you *prove* the ARX(1,1) form is adequate on real
  data before trusting it.
- It captures non-parametric behaviour (the emu's tumble-style limit cycles,
  rate saturation, sensor faults) that no 2-parameter model can represent.
- It is the crash-in-progress forensic backstop (already the agreed role).

**v1 transport (zero new protocol):** bake the identified params into the trace
dump as a **v3 header block** (§6.1). The dump then carries both the raw rows
(validation) and the distilled params (the deliverable). A future v2 can add a
lightweight live telemetry tag and retire the dump; this report keeps the dump
as the single path so both birds ride one wire.

### 6.1 v3 trace header layout (exact byte spec)

Current `TRACE_HEADER_SIZE = 128` (32 B base + gain block 36..119 + pad
120..127). v3 grows to **200 B** and moves the record region:

| region | offset | size | content |
|---|---|---|---|
| v2 base header | 0 | 32 | unchanged (magic/version/traceType/recordSize/flags/axisMask/period/rateGainCode/snapStart/count/ticks) — **version byte becomes 3** |
| v2 gain block | 32 | 88 | unchanged (count u16 + version u16 + 21 f32) |
| **v3 ident block** | 120 | **72** | per axis (Roll/Pitch/Yaw order), 3 × 20 B: `a f32, b f32, sigA f32, sigB f32, n u32` |
| pad | 192 | 8 | zero |
| records | **200** | … | `TRACE_DATA_BASE` moves 128 → **200** |

Define changes in `trace.h`: `TRACE_VERSION 3` (was 2),
`TRACE_HEADER_SIZE 200`, `TRACE_DATA_BASE 200`, new
`TRACE_IDENT_OFF 120`, `TRACE_IDENT_AXIS_BYTES 20` (record-region math
recomputes automatically via the existing `TRACE_DATA_LEN`/`TRACE_MAX_RECORDS`
expressions). Ring budget: 64K `(65536−200)/16 = 4083` records (vs 4089),
F411 32K `2040 → 2033` — negligible (Greg-approved scale of change).
`TraceWriteHeader()` (trace.c:136) gains a loop writing
`Identify.Axis[eRoll..eYaw]` fields at offset 120..191; `TraceSnapshotGains()`
already runs at capture-open — invoke `IdentifyLoop`'s exporter there.
**The on-flash rediscovery check** (`StartBBChunkedDump`) already validates
magic/version/recordSize via constants, so it adapts automatically.
GCS `parse_trace` branches on version (v1 ≠ v2) — add the v3 branch: read the
ident block, display `τ = −dT/ln a`, `K = b/(1−a)`, bounds, and `n`.

---

## 7. Build & verification

- **FC build:** `python3 UAVXArmQ/scripts/fc_build.py` (env `BOARD=...`
  to build a single target; default = ALL 7 targets). The build **globs
  `src/**/*.c`** (fc_build.py:101) so adding `identify.c` needs **no Makefile /
  no source-list edit**. Sanity under `BOARD=SPEEDYBEEF405WING` first
  (`SPEEDYBEEF405WINGQ_r0.bin`), then the full board list.
- **Toolchain:** xPack arm-none-eabi-gcc 15.2.1 at
  `~/toolchain/xpack-arm-none-eabi-gcc-15.2.1-1.1` (falls back to `/usr/bin`).
- **Emulator validation (primary):** run under emulation; capture, dump,
  load in the GCS viewer and read the v3 ident block. Expected —
  MR `τ_eff ≈ MotorTau` (0.10 s), `b/(1−a)` compares to the emu authority
  `MaxThrust·0.25·ArmLen·InertiaR`. Match within a few tens of % = PASS.
- **GCS:** `parse_trace` v3 branch only; `python3 -m py_compile` on the
  touched Python files.
- **Real-aircraft:** bench/`.rawlog` validation only — the identify module is
  purely observational, so no flight risk; a flight just records better data.

---

## 8. Coding Standards compliance (mandatory)

1. Classic C99/C11; exactly one exit per function; no early `return;` buried
   mid-function (the KF branch is a positive-if structure).
2. **No `/` or `%`** except the single runtime scalar `S` at the KF gain
   (runtime divisor → allowed). No constant divisions (`/2`, `/100`).
3. `switch()` for 3+ branches; cyclomatic complexity <10; functions ≤100 lines
   (split: `IdentifyUpdate` (2×2 KF math), `IdentifyFeedAxis`, `IdentifyLoop`,
   `IdentifyExporter`).
4. No malloc/free; no pointer arithmetic; static RAM state.
5. Bounds-checked array access; `eRoll/ePitch/eYaw` indexing is ≤3-loop-bound
   (exempt).
6. Enums use lowerCamelCase e-prefix (`eIdentIdle`…); `#define` constants
   UPPER_CASE (`IDENTIFY_*`); `real32` for all float math; comments explain
   WHY (the `prevOut`/delay pairing is the subtle bit — document it).
7. File header copyright block **must** be preserved/mirrored (trace.c/emu.c
   style), plus the attribution line
   `// Adapted <YYYY-MM-DD> by GKE per Session_Report_InflightPlantID_Sep14.md`
   if you reuse this design verbatim.

---

## 9. Open items for the implementer (decide before coding)

1. **Excitation deadbands** — concrete values (start `|Out| > 0.1f &&
   |ΔRate| > 0.05f` rad/s). Tune on the emulator; a tight gate is safer
   (fewer bad updates) than a loose one.
2. Whether to add `IdentifyCluster()` now or defer to the GCS-parse step.
3. Whether `IdentifyInit()` goes in `InitControl()` (control.c) or the
   top-level init in `uavxarm-v3-gke.c` — pick one; both work.
4. Confirm ARX(1,1) adequacy on real data via the residual overlay (a v3
   viewer feature) BEFORE any "retire the dump" follow-on work (§6).
5. The v1 → v2 live-telemetry-tag path is deliberately **not** started —
   no new tag numbers are consumed by this design.

---

## 10. Why this is sound (design rationale recap)

- **KF-RLS is the canonical recursive identifier**, O(2²) per axis — trivial
  at 500 Hz on a Cortex-M4; no fixed window needed (Q = adaptive forgetting).
- **The plant structure comes from the FC's own emu model** (motor lag +
  authority + quadratic damping), so the identified `(a,b)` map 1:1 onto the
  sim/emu plant in `test_pid_sim.py` — closing the "sim vs real" gap the FW
  `/4` fudge-factor question (AGENTS TODO) ultimately needs measured truth for.
- **Purely observational → zero flight risk** in v1; the emulator and the
  rawlog already supply everything needed to validate it.
- **Distilled output (≈120 B/dump) vs raw (24 kB/s)** is the direct answer to
  "do we still need trace dumps?" — not while passing through a v3 header that
  keeps the raw rows for cross-validation on the way to dropping them.

---

## 11. IMPLEMENTED 2026-09-14 (status update; supersedes the "not started" header)

The re-scope decision (Greg + assistant) replaced the v3-trace-header transport
with the **tag-57 TUNING live carrier**; the implementation is COMPLETE and all
7 FC targets build clean + GCS py_compile clean.

**Deviation record — decisions locked via the question tool before coding:**
1. **Deadbands = spec defaults**: `IDENTIFY_DEADBAND_OUT 0.1f` (fraction of
   authority), `IDENTIFY_DEADBAND_RATE 0.05f` rad/s (item 1, §9 — resolved).
2. **`IdentifyCluster()` deferred to the GCS** (item 2 — resolved): the FC
   exports raw `a, b, sigA, sigB, n`; τ/K are derived in the dock page.
3. **`IdentifyInit()` from the top-level init** in `uavxarm-v3-gke.c` (~177,
   alongside `InitControl()`/`InitBlackBox()`, plus the re-init path ~296)
   (item 3 — resolved).
4. **Flash snapshot reuses the CAPTURE sector** (`CAPTURE_FLASH_SECTOR/ADDR`,
   128 B image: magic "IDEN" + version 1 + f32 dT + 3×20 B = `a,b,sigA,sigB,n`
   per axis) — the trace/`TraceCommit()` owner is mothballed 2026-09-14 so the
   sector is free. `IdentifyCommit()` is gated on an `IdentifyDirty` flag
   (write skipped when no in-flight update happened this session — this also
   preserves the sector across disarms-without-flight) and called from BOTH
   disarm sites (`uavxarm-v3-gke.c` eLanded-disarmed ~412 and the eShutdown →
   ePreflight path ~454). `IdentifyInit()` restores the last snapshot via
   `ReadBlockArmFlash` (all-0xFF un-written sectors fail the magic check →
   defaults, the same discovery pattern as the trace flash fallback).
5. **New page = main-window QDockWidget** (item 5 of the design was re-opened):
   `ui/identify_window.py` `IdentifyDock`, right-side dock, discovered via a
   checkable "Identify" toolbar button (opt-in — hidden by default).
6. **`IdentifyReset`/arming transitions NOT wired** (design §5.3 says optional;
   the Q keeps the filter adaptive — the flash restore carries the previous
   flights' identification forward, `IdentifyReset` remains exported API).

**FC changes:** `identify.h` / `identify.c` (new); `UAVX.h` include (after
`control.h` — header uses `eYaw`); `DoControl()` tail hook `IdentifyLoop(dT)`
(`control.c:722`, after the category switch so `A[a].Out` is finalised for all
axes); `SendTuningPacket()` (`telem.c`) appends the 60 B identify block
(Roll/Pitch/Yaw order, length field 38→98); `TxESCu32` added (`telem.c`/`telem.h`).

**GCS changes:** `TuningData` + `parse_tuning_packet` extended for the trailing
60 B block (`has_ident` flag; legacy 38 B v1 payloads still parse — the old
parser's duplicated `rate_desired_yaw` copy-paste bug fixed in passing — wait,
checked: the bug was a duplicated `rate_desired_yaw` line; corrected to
`rate_actual_yaw`); connect-time tag-57 request + ~1 Hz `_identify_poll_timer`
(started while connected, stopped on disconnect, only when the dock is
visible); case-57 routes `parsed` into `IdentifyDock._refresh`.

**Verification:** all 7 FC targets (`UAVXF4V3`, `UAVXF4V4`, `DEVEBOXF4`,
`SPEEDYBEEF405WING`, `FLYINGRCF4WINGMINI`, `BLUEBERRYF405`, `MATEKF411WING`)
build clean; GCS `packet_parser.py`/`main_window.py`/`identify_window.py`
py_compile clean in source + 3 kits; tag-57 v2/v1 parse round-trip verified with
a synthetic payload. NAND-mounted emulator validation (recover `τ ≈ MotorTau`)
is the *next* step — requires a test harness feeding recorded `(Out, Rate)` or
a live emu run, not yet executed.