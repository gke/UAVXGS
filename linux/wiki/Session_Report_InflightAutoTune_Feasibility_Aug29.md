---
orientation: landscape
---

# Inflight Auto-Tuning — Feasibility Study — Aug 29

**Status:** FEASIBILITY ONLY — no code written. Stage 0 (this study) of the AGENTS.md
TODO. Decision gate for the user before Stage 1 begins.

## TL;DR

- **Yes, feasible** — and the FC already contains the two hardest prerequisites:
  a dormant `tune.c` scaffold and an emulator plant (`emu.c`) whose rate dynamics
  provably match the `test_pid_sim.py` plant (same `2·|r|·r` damping, same
  `torque = Out·R.Max·Inertia` structure). The critic must be **externalized from
  the sim first** so FC and sim measure identically — that is the single blocking
  gate and the recommended first implementation step.
- **Stage 1 = measure, stream, criticise offline.** FC computes the same metrics
  the sim critic uses (rise/overshoot/settle/integrated-error/headroom) in a
  dedicated `eAutoTune` mode and streams raw gyro + attitude + output over the
  existing softserial/D8 telemetry. No gains are touched in flight.
- **Stage 2 = onboard identification.** A one-shot frequency/step probe per axis
  identifies the rate-loop plant (EMPF/Relay style), proposes gains against the
  *externalised* criteria. Baseline applies them **on disarm** (pilot keeps full
  authority, FC never changes gains mid-air); a **paced, bounded in-air APPLY tier**
  (rate-loop only, `TuningScale` ramp, auto-revert, sim-blessed) is examined in a new
  section and is feasible without crossing the continuous-adaptation red line.
- **Order enforced by the TODO:** sim-first, MR-first. The sim plant + critic
  strings are the acceptance gates; the emulator (`emu.c`) is the free rehearsal
  stage before a real flight.
- **Three gain-application tiers, one of which is now being considered:**
  1. *Disarm-write* (baseline — safe, zero in-air risk): probe captures in RAM, all
     writes deferred to disarm via the existing `Tuning` hook (`uavxarm-v3-gke.c:406`).
  2. ***Semi-online in-air apply*** (considered here): pilot demands, FC applies the
     change to RAM-loop gains *while airborne*, revertible, single-axis, off by
     default (see §In-air online application).
  3. *Continuous adaptive* (still OUT of scope — fights the Q outer loop, unknown
     trim envelope, violates "no silent behaviour change in air").
- **Capture architecture confirmed with user:** RC-switch trigger, one test per
  switch-ON (dormant until OFF→ON), full-rate raw capture into the **64K CCM ring
  buffer** (taking over the dormant `bb.c` allocation — `bb.o` is the only `.ccm`
  consumer, `BlackBoxEnabled` is false, `UpdateBlackBox()` is a stub), disarm →
  flash write to a **dedicated sector** (top-of-image reserve, currently free),
  GCS **dump-on-command** via a new tag. See §Stage 1 design as-built.

## Grounding — what the codebase already provides

| Need | Existing artifact | Location |
|---|---|---|
| FC hook for a tuning mode | `Tune()` / `InitTune()` — dormant scaffold, `Tuning`/`TuningEnabled` flags | `UAVXArmQ/src/tune.c` |
| Plant identical to the sim | MR: `Rate -= (M·0.25·Out·Arm·J⁻¹ − 2·Sign(r)·r²)·dt`; FW: same `${2·Sign(Rate)·Sqr(Rate)}` damping | `emu.c:439-474`, `test_pid_sim.py:856-1008` |
| Convergence + persist precedent | `TrackCruiseThrottle()` — runtime state converges, persists on disarm via `ConfigChanged` | `control.c:164`, AGENTS.md §Cruise Throttle |
| Telemetry bandwidth | softserial D8 scheduled bus (~25–27 % used) + S.Port/CRSF real UART + USB tag protocol | `rc.c:394-398`, AGENTS.md §D8 |
| Proof authority for raw telemetry | existing tag stream (params, attitude, alt, battery) over `TelemetrySerial` | `*.c` `TxState(TelemetrySerial,…)` |
| Offline critic source | `Critique*`, `Criteria`, `StepMetrics` | `uavx-python/src/tests/test_pid_sim.py:2543-2656` |
| Ground-truth fleet for gate calibration | critic-validated `original/` tunes (wave-1/2, this session) + `AH_CRITERIA_MR` 2.5 s | `airframes/original/*.af`, this report |

## Stage 1 — externalise the critic, then stream

### 1a. Externalise the critic metrics (THE gate — blocking)

Today the metrics live inside `test_pid_sim.py` and only exist post-mortem on Python
lists (`StepMetrics` etc.). The FC must measure the *same quantities* from *live time
series*. Required refactor:

- Extract the metric *definitions* (rise 10→90 %, overshoot %, settle ±2 %, integrated
  error, rate headroom = peak/(2×sp), zero crossings) and the `Criteria` tables into a
  shared module, e.g. `uavx-python/src/critic/` with a stable named interface
  (`critic.metrics_from_series(t, x, sp, mode)`, `critic.criteria_for(mode, cat)`).
- `test_pid_sim.py` imports it (behaviour-identical; fleet regression run must produce
  byte-identical pass lines — this is itself the acceptance check).
- The Python module becomes the **specification** for the C implementation:
  metric-by-metric, with worked sim examples, so a later reader can prove the FC
  maths matches. (Coding standard: no division, one exit, etc. — trivial for these
  sums.)

**Why this must come first:** the whole point is that an FC-measured "settle 3.1 s"
means the same thing as a sim-measured "settle 3.1 s". If the definitions diverge,
Stage-2 gain proposals are validated against noise.

### 1b. FC-side capture (`eAutoTune` mode on `tune.c`) — AS-BUILT (user-confirmed)

- New `State` (or sub-mode) `eAutoTune`, entry gated by a config bit
  (`AutoTune` in `ConfigBits`/`Config2Bits`) **anded with an in-air RC switch** —
  never automatic. **Trigger loop:** switch ON → run the *current queued test*
  (single-axis step/sweep), FC goes dormant; switch OFF→ON → **advance to the next
  test**; repeat until all queued tests done. Fully pilot-paced, no GCS interaction
  mid-air.
- **Capture buffer = the 64K CCM ring taken over from `bb.c`** (`ccm` section,
  `bb.o` is its only consumer today; `UpdateBlackBox()` is a stub and
  `BlackBoxEnabled` is false — nothing else uses it, so takeover is clean).
  Full-rate raw sample per test axis: `(Rate, Desired, Out, t)` = 4 float32 = 16 B →
  **8 KB/s @ 500 Hz → ~8 s of capture per flight burst** (3 axes ≈ 2.7 s each fits
  with margin).
- **Overhead:** ~0.3–0.5 µs/cycle (40–80 cycles of float stores into CCM) —
  <0.03 % of the 2 ms loop. Invisible.
- Probe waveform (pilot-armed): a small rate/attitude step or a swept-sine injected
  in rate mode with full stick authority intact; FC logs `Rate`, `Desired`, `Out`.
- **Persistence:** nothing written to flash in-flight. On disarm the `eLanded` →
  disarmed branch (`uavxarm-v3-gke.c:406`) writes the RAM ring to a dedicated flash
  sector via `WriteBlockArmFlash`; GCS **dumps it on command** over a new tag.
  Flash sector: the top-of-image reserve (`MEMORY FLASH = 1024K-32K-256K` leaves
  sectors 10–11 free) — a 128 K `CAPTURE_FLASH_SECTOR`; erase ~1–4 s + program
  ~20–40 ms, one-off, on the ground, in a housekeeping write like the existing
  `RefreshConfig()`.
- The `Tune()` hook already runs in `UpdateControls()` at `rc.c:1058` on
  `F.NewCommands` — the switch-triggered probe/advance logic lives there.

### 1c. Telemetry budget

- D8 softserial is ~25 % utilised at the steady schedule; a burst of e.g. 1–2 k raw
  samples right after the probe fits the 1024-byte TX FIFO in a few slots (the
  GPS/cells 1 s burst precedent). Budget math must be re-run (AGENTS.md::D8) with
  the actual capture length.
- Cleaner alternative for development: full-USB tag stream on `TelemetrySerial`
  (board bench / tether), D8 only for on-air field use. Softserial headroom at 25 %
  says the burst is feasible but must be tested on air (TODO already lists this).

### 1d. Acceptance gate for Stage 1

- Sim: `critic` externalisation regression (identical FAIL/PASS lines, fleet).
- Emulator: run `emu.c` step with proposed capture → decoded metrics should match
  the offline Python critic on the same emulator trace (both consuming the same
  raw series — this closes the loop that Stage 2 depends on).
- Field: hover/step flight, capture, GCS decodes → compare against sim-predicted
  metrics for the same gains. Agreement within the critic tolerance band proves the
  FC/sim measurement link before any identification is attempted.

## Stage 2 — onboard identification (MR-first, sim-first)

- **Method options:**
  1. **Step/impulse + output-error fit** (simplest; the sim plant is low-order —
     torque/out, quadratic damping — a 3–4 parameter fit per axis).
  2. **EMPF / Relay (Åström–Hägglund)**: measured limit-cycle crossing → gain + a
     phase measure → ultimate gain/period. AttackKnown catalog technique, robust,
     no plant model needed. Little code; natural fit for `tune.c`.
  3. **Swept-sine + Bode fit** — most information, most code, most time in air.
- **Recommendation:** step + output-error for the *plant* (needed to validate the
  sim model against the real airframe — the missing "ground truth" this whole TODO
  keeps circling), and a **single-rate probe** for the *gains*. Propose + apply on
  disarm only; the proposal is a *suggestion* shown as a diff, not a silent write.
- **Sim-first acceptance:** run the probe on the sim plant with the critic-validated
  fleet gains → identification must recover ±20 % of the known gains. Replay on
  `emu.c` too. Only then a single MR frame, bench → hover → short flight.

## In-air (online) gain application — considered

The enforce-red-line above ("never changes gains mid-air", AGENTS.md) is the
*baseline* posture. This section examines when and how an **online** path — applying
a tuned delta while airborne — becomes defensible, and when it never will.

### 1. What "online" means (two distinct senses)

1. **Online measurement, offline apply** — the already-approved Stage-2 flow: probe
   in air, identify on the ground, apply at disarm. *Not* online tuning.
2. **Online measurement, online apply** — the FC proposes and applies a *bounded
   gain delta* during a flight, under pilot pacing, with a revert path. This is the
   genuine in-air online case examined here.
3. **Continuous adaptive** — a live gain surface modulated by continuous
   identification with no pilot pacing. Still OUT of scope (see §7): it fights the Q
   outer-loop band-schedule, has unknown trim envelope, and is exactly the "silent
   behaviour change" red line.

### 2. What makes online apply *physically* defensible here

- **The gains are plain RAM floats.** `A[a].R.Kp/Kd`, `A[a].P.Kp/Ki` are single
  4-byte stores in a running structure — no flash, no config commit, no recompile.
  Writing them "live" is architecturally trivial; the difficulty is entirely in the
  *decision and the revert discipline*, not the mechanics.
- **The two-loop cascade gives a natural staging.** Inner rate loop (500 Hz) provides
  damping and a stability floor; the outer quaternion attitude loop provides the
  bandwidth. Applying a *bounded* inner-loop delta first (rate Kp/Kd) is recoverable
  even if wrong within limits, because the outer loop still holds the attitude
  target. The risky direction is the reverse (touch `P.Kp`/`P.Ki` first) — that
  moves the attitude crossover directly. Any online path must be gated
  **rate-loop-first, outer-loop-later, one axis at a time**.
- **A "last-known-good" set lives in RAM.** Keep the gain set that is currently
  flying; a revert is a one-shot copy of 6–9 floats — sub-µs, atomic enough
  (single loop tick), no re-init of integrators needed because the floats are the
  only thing that changed.
- **`TuningScale` already exists.** `tune.c:23` declares `real32 TuningScale = 1.0f;`
  — a dormant global multiplied into the loop as the original design's master
  tuning knob. It is exactly the right *mechanism* for an online apply: instead of
  swapping gain fields (irreversible unless the old value is remembered), modulate
  `TuningScale` smoothly toward the proposed value and snail its rate of change
  (`TuningScale += (target - TuningScale) * tau`). At `TuningScale == 1.0` the
  aircraft flies the persisted gains; drift away from 1.0 is the visible "tuning
  active" state and the revert is literally `TuningScale = 1.0f`.

### 3. The pilot interface (keeps authority in the hands)

- The one-test-per-trigger sequencing already paces probes. A **3-position switch**
  extends it into the apply domain:
  - **OFF** — probe dormant, `TuningScale = 1.0`.
  - **ARM** — next test runs (capture as as-built), proposal prepared but NOT applied.
  - **APPLY** — FC blends `TuningScale` to the proposal over ~0.5 s, bounded to the
    per-stage `MAX_TUNE_DELTA` (see §4).
- On the probe *after* an APPLY, the live critic runs the exact same metric set; if
  the measured settle/overshoot degrades past a hysteresis band relative to the tier
  below, the FC **auto-reverts** to the last good set and flashes the
  `ExcessLift`-style GCS flag + alarm rather than compounding the step.
- No GCS interaction is required mid-air, but the GCS (when connected over USB)
  sees the same proposal + live metric diff streamed by the existing telemetry
  tags — the pilot can over-rule from the ground at any point.

### 4. The bounded-delta discipline (the safety budget)

| Rule | Value / behaviour |
|---|---|
| Per-apply max | `new = old × (1 ± MAX_TUNE_DELTA)`, `MAX_TUNE_DELTA` default 15 % — a loud, small step, not a jump to a proposed target far away (multi-apply to converge) |
| Per-flight max | ±40 % from the armed persisted gains for ANY axis combination — hard clamp |
| Axis sequencing | rate `Kp` → rate `Kd` → `Ki`; exactly one quantity per APPLY; never `P.Kp`/`P.Ki` in the same flight as a rate apply |
| Yaw | `ControlRateYaw` is pure PD and weathercocking-sensitive — yaw applies allowed only after MR attitude applies prove non-oscillatory on that frame |
| Revert trigger | critic bandwidth-degradation on the follow-on probe, rate-loop limit-cycle amplitude exceeding a band, or pilot switch to OFF |
| Write persistence | in-air applies mutate RAM + `TuningScale` ONLY; disk sees the *last confirmed-good set* at disarm. A failed in-air experiment never persists |

### 5. The sim/emu role changes from "rehearsal" to "blessing"

With online apply, the emulator is no longer just a pre-flight rehearsal — it
becomes the **catalog generator**: run the intended probe + the proposed delta on
`emu.c`/sim for this frame *before* the flight, and the FC only applies a delta
that the sim already validated for that (frame, axis, gain, delta) cell. In-air
identification then selects among pre-blessed cells rather than inventing new gain
values. This keeps online apply deterministic and audit-able: no value is ever
written airborne that did not first live on the shared `src/critic/` + emu path.

### 6. When in-air online apply is worth it — and when not

**Worth it (pilot-paced, bounded, revertible):**
- Live *measure-and-converge* sessions on a benign MR (or a slow, stable FW like the
  Radian) where landing each iteration costs real flight time and re-arming state
  (GPS re-acquire, alt re-origin, trim resets).
- Validating a suspicion quickly: "does raising yaw Kd really kill the 8 s settle?"
  — apply, hold attitude target, watch the follow-on probe, revert if worse.

**Never worth it (stays offline):**
- Any FW/MR frame already on the edge of the critic envelope (the whole point of the
  ζ≥0.05 spiral gate and the margin-based fleet table is that we do not tune near
  instability while flying).
- First flights of a new airframe, high-wind days, low battery, or any condition
  where a revert-then-hold still leaves an unacceptable flight.
- Production missions / cargo / beyond LOS on the tuning flights themselves.

### 7. Continuous adaptive — the line stays drawn

Live *gain-surface* adaptation (no pacing, no bounded delta, identification always
running) is not defensible on this codebase: the Q outer loop expects a fixed
band-schedule from the inner loop, silent mid-flight gain drift is unrecoverable by
any pilot who wasn't told, and the sim/emu blessing discipline collapses (there is
no cell to pre-validate). The deliberate compromise that makes online legitimate is
precisely the pacing + bounding + last-known-good + blessing — which is why §3–§5,
not raw adaptation, is the proposal.

## Costs / effort / risks

| Item | Effort | Risk |
|---|---|---|
| Critique externalisation + regression | Small–medium (half-day) | Low — pure refactor, regression-gated |
| FC capture buffers + telemetry burst | Medium | Low — new mode, off by default |
| GCS decode + metric display | Medium | Low |
| Identification fit (step/RELAY) | Medium | Medium (model-fit divergence on real aero) |
| In-air APPLY tier (rate-loop, bounded, `TuningScale` ramp + revert) | Small–medium | **Medium** — see §In-air above: revert + 15 % delta + blessing gates keep it recoverable |
| On-air field validation | — | **Medium-high** — must be pilot-armed, bench & emulator first |
| Real-airframe/sim model divergence | — | **The core risk**: sim plant is an approximation; a bad gain write is worse than none |

**Abort criteria (speak up now):** if the real-airframe identification disagrees with
the sim plant estimate beyond ~±30 % on an axis, stop and go back to plant
identification — do NOT widen data-sheet/aero guesses to force a match. This is
exactly why the TODO insists MR-first (small cheap frames, short tails to take the
hit) and sim-first.

## Recommended sequence (if approved)

1. Externalise critic → `src/critic/` shared module; fleet regression byte-identical.
2. Add the step-probe simulator mode in `test_pid_sim.py` that emits the raw trace a
   keep-identification round-trips from (prove the method can recover known gains).
3. Implement `eAutoTune` capture + disarm-write gating on `tune.c` (reusing the
   `Tuning` flag at `uavxarm-v3-gke.c:406`), with the RC-switch trigger sequence and
   the 64K CCM ring as as-built in §1b. Dump to GCS over the new tag.
4. Bench on emulator, then one MR frame hover/short-flight capture.
5. Only with 1–4 green: **application**. First *disarm* path (write-on-landing,
   pilot applies via configure page — the baseline). Then the **in-air APPLY tier**
   (§3–§5) as a separate, opt-in, rate-loop-only, bounded+revert path. Never
   auto-write without pilot action, and never touch the outer Q loop in air.

## What is deliberately NOT in scope (feasibility verdict)

- **Continuous/live gain-surface adaptation mid-flight** (unpaced, unbounded,
  identification always running) — would fight the Q outer loop, violate the "no
  silent behaviour change in air" rule, and has no sim/emu blessing channel. The
  *paced, bounded, revertible* in-air APPLY tier (§In-air) is the defensible
  middle ground and is explicitly considered.
- Multi-vehicle fleet calibration from this feature (the fleet `.af` + critic table
  already serve that; this solves *single-airframe ground truth*).
- Any write path that does not go through disarm + pilot confirmation — a flash
  write mid-flight is forbidden on this platform (sector erase ~1–4 s blocks the
  loop); the in-air tier mutates RAM/`TuningScale` ONLY, and only disk-persists a
  confirmed-good set at disarm.

## Decision needed from the user

- Approve starting **Stage 1a (critic externalisation)** as the first PR-sized step?
- Capture carrier preference: USB tag stream first (bench), D8 burst second (field)?
- Probe authorization: OpenTx switch (recommended — hands stay on sticks) vs GCS
  button over USB?
- **In-air APPLY tier** (§In-air): add it to the roadmap behind Stage 1/2 (rate-loop
  only, bounded 15 % delta, `TuningScale` ramp + auto-revert, sim-blessed)? Or keep
  the write-on-disarm baseline only, and revisit online apply after field data?