# Development Log Backfill — Reconstructed Decisions (28 Aug 2026)

## Purpose

AGENTS.md requires every decision to be traceable in this log with its *logical
discourse* (options considered, rejected alternatives and WHY, the adopted
reasoning). Several earlier work items had no running-log entry: their "why" was
spread across AGENTS.md one-liners, code comments, and GCS migration scripts.
This file backfills them in date order, reconstructed from `AGENTS.md`, code
comments, and remaining tooling. Where a rationale or date **could not be
recovered** from the repository, that is stated explicitly — see **Gaps**.

Entries already covered by existing reports are NOT duplicated here:
`wiki/docs/` (Jul-11 Aug 2025 through 24 Aug 2026 topic reports) and the session
reports `Session_Report_Flag_Cleanup_Aug27.md` / `Session_Report_D8_Mag_Yaw_Aug28.md`.

---

## 1. Quaternion attitude fork — `UAVXArmQ` (undated, earliest; pre-dates all r?? sessions)

**What/where:** The outer roll/pitch/yaw attitude loop was rewritten as a full
3-axis quaternion controller (`control.c` `DoQuaternionAttitudeControl`, ~:501,
"Full 3-axis quaternion attitude controller — gimbal-lock free — GKE"). The old
per-axis Euler-angle P loops were replaced by:

- Desired angles → `EulerToQuat(QDesired, …)`
- Dive override → `DoDiveControl(QTarget, …)` wins when a dive is active
- Error → `QError(Qe, QDesired)` then `QeToQa(Qa, Qe)` (axis-angle form)
- Rate setpoint per axis: `2.0f * Qa[a] * A[a].P.Kp` (Angle mode),
  blended in Horizon mode via `AngleRateMix` vs. direct stick-rate,
  plus an outer integral `A[a].P.IntE += 2.0f * Qa[a] * A[a].P.Ki * dT`
  clamped to `A[a].P.IntLim` and added to the P rate setpoint.

Fixed-wing shares the same machinery (`DoFWAttitudeControl`, :591) for
roll/pitch; yaw setpoint is managed separately (`UpdateYawSetpoint`, :468) and
*all three* axes pass through the quaternion controller.

**Rationale:** Gimbal-lock freedom (a pure Euler path breaks down at ±90° pitch;
the quaternion + `QeToQa` axis-angle path has no singular axis ordering) and a
single unified attitude path for multicopter and fixed-wing instead of two
divergent controllers to tune. The `2.0f` factor is not magic: for small errors
the rotation-vector magnitude ≈ `2·sin(θ/2)·θ̂` ≈ `2·q` (rad), scaled by `P.Kp`
so the product *is* a demanded rate (rad/s) — matching the inner rate loop's
units with no unit conversion on the wire.

**Discourse (options considered):**
- *Rejected: keep Euler angle-P and patch rollover handling.* It already showed
  symptoms this fork exists to fix (heading rollover asymmetry, no clean way to
  blend MC/FW, and the outer loop could not represent a total-angle error).
- *Rejected: rewrite the inner rate loops as quaternion loops too.* The inner PD
  rate loops (`control.c` `ControlRate`/`ControlRateYaw`) are well-understood
  and hardware-tuned; quaternion benefits are at the *outer* attitude stage.
  Only the outer loop was touched.
- *Adopted: quaternion error → axis-angle → per-axis rate setpoints, sharing the
  existing `A[axis].P` gain struct*. Least churn, one semantic everywhere.

**Build/verification:** compiles on all commissioned targets; tune state is
empirically validated on air. **Date/rev of the fork is NOT recoverable from the
repo** (no SVN in sandbox; see Gaps).

## 2. Q gain naming convention (r??+)

**What/where:** Param/config names carry the `Q` (`RollAngleQKp`, `RollAngleQKi`,
`RollAngleQIntLimit`; GCS keys `ROLL_ANGLE_Q_KP` etc — `protocol_enums.py:491`).
GCS scripts/gain-matching match on `ANGLE_Q_KP`/`Q_KI`/`Q_INT_LIMIT`.

**Rationale:** The FC struct fields stay **generic** — `A[axis].P.Kp` /
`A[axis].P.Ki` / `A[axis].P.IntLim` inside `PIStruct`. Deliberately NOT renamed
to `Q`: `P` also holds `Max` / `Desired` / `IntE` (angle limit, setpoint,
integrator) which are not quaternion-controller-specific, and `Alt.P.*`
(altitude) shares the same `PIStruct`. The quaternion application
(`2.0f * Qa[a] * A[a].P.Kp`) is the only semantic in C — the *name* is the
semantic for the GCS (sim + scale tuner resolve a gain's meaning by name).

**Discourse:** *Rejected* renaming the C struct fields to `Q.…` (would be
misleading for `Max`/`Desired`/`IntE` and strand the altitude struct); *adopted*
naming-by-parameter-name with GCS matching on `Q_KP`/`Q_KI`/`Q_INT_LIMIT`.
Warning recorded: matching on bare `ANGLE_KP` (the pre-fork key) **breaks** —
the rename removed it (`migrate_limits.py:115`, `fix_q_int_limit.py` preserve
that matching rule).

## 3. Unified float parameter system (r??+)

**What/where:** All 128 parameters stored as `float32` in
`Config.ParamData[i].f` (`params.c`). `ParamMetaEntry` (`FLOAT` macro: target,
type `eParamFloat`, class, scale, min, max, default, unit) is the only source of
truth; `U8` macro remains only for selector/enum params (stored as
`(float32)(uint8)` for exact 0–255 round-trip, never PID gains). `ParamTableCRC`
guards layouts separately from magic (`0xFEEDBEE3`) and checksum.

**Rationale:** One code path, no legacy uint8 conversion, exact float round-trip.
The **FC does not clamp incoming writes** — the ParamTable `max` is the *only*
agreement between FC and GCS, so `params.c` bounds and GCS `PARAM_LIMITS` must
stay in sync by rule (AGENTS "ParamTable is the source of truth for bounds").
Display uses `display = fc_float * PARAM_DISPLAY_MULT`; write is the inverse;
`.scale` in the ParamTable drives GCS spin-box construction for legacy-unit
params — never to be edited when changing limits.

**Discourse:** *Rejected / removed:* `PARAM_SCALES`, `LEGACY_TAGS`, the legacy
checkbox, `_legacy_mode`, uint8 integer display — all eliminated because they
created two display paths that could disagree and duplicated conversion logic.
*Adopted:* float-only with a single per-index display multiplier, and class
bounds (`eClassGainAngle*`, `eClassRateIntLim` …) as the bounds source
(`params.c:102-108`, normalization at `:347` clamps stale flash values into the
class ceiling before unpacking). Migration tooling (`fix_q_int_limit.py`,
`migrate_limits.py`) exists precisely to rescue old scaled values into the new
system.

## 4. Cruise throttle feedforward v2 (r15+)

**What/where:** `TrackCruiseThrottle(dT)` (control.c:164) converges the
feedforward hover baseline `pCruiseThrFF` **directly from `AltHoldThrComp`**
(not `DesiredThrottle + AltHoldThrComp − pCruiseThrFF` as before). Only gate is
`F.ThrottleMoving` (no ROC guard). `cCruiseTrackingRate = 2%/s` (was 0.5%/s),
`pMaxAltHoldThrComp` default 25% (was 15%). Integrators and `AltHoldThrComp`
are zeroed at hold engagement (control.c:327-331). Persisted to
`Config.ParamData[59].f` on disarm via `ConfigChanged`; no longer overridden by
`ApplyParameters()` at commit/condition. Emulation cruise = 0.5; all `.af`
`EST_CRUISE_THR = 0.5`.

**Rationale:** The feedforward is the hover-baseline the PI loop sits on top of.
If the baseline is wrong, the PI term carries a persistent steady-state offset
that fights the integrator; v2 makes the baseline exactly the average correction
the loop already wants, so `Ki` only handles genuine transients. 2%/s tracking
lets a 15% error converge in ~7.5 s — fast enough to follow battery sag /
airframe changes without chasing gusts. Zeroing at engagement removes stale-term
bias from the previous hold so a new engagement starts clean.

**Discourse:** *Rejected (v1 formula):* tracking `DesiredThrottle +
AltHoldThrComp − pCruiseThrFF` — the feedforward shares a term with the
controlled output, so convergence fed back on itself and absorbed part of the
integral's job. *Adopted:* track `AltHoldThrComp` alone — it is precisely "what
the loop adds over feedforward", so the FF settles to the value that makes the
correction average zero. *Rejected (ROC gating):* a ROC guard paused tracking on
any residual climb/drift; `ThrottleMoving` alone correctly stops tracking only
when the pilot actively overrides.

## 5. Control-flow convention — single exit, no early `return` (r??+)

**What/where:** Banned: bare `return;` buried mid-function (early-exit
reject/hold paths from inside deep `if`s). Required: restructure to a single
logical exit — wrap the *pass* path inside the positive `if`, keep rejections in
the `else`/fall-through. Simple `return` of a computed expression at a function
tail (e.g. `return LPF2(...)`) is explicitly fine. Applies everywhere,
including IMU driver.

**Rationale:** Real-time dispatchers (state machine, sensor updates, WDT stage
marks, `SIOTokenFree` release) rely on a uniform flow: an early exit silently
skips shared cleanup/finalization and makes stage-trip-diagnostics lie. One
visible exit per function keeps the watchdog/marker bookkeeping honest and the
code auditable.

**Discourse:** *Adopted* the positive-pass shape rather than flag-and-goto (goto
is banned) or deep-nesting the whole function (cyclomatic complexity cap <10).

## Gaps — rationale/dates NOT recoverable from this repo

These could only be backfilled from remaining evidence; a host-side `svn log`
(SVN exists at `UAVXArmQ/.svn` and `UAVXGS/.svn`, but `svn` is unavailable in
this sandbox) may recover exact revs/dates:

- **Exact date/revision of the quaternion fork** (Section 1) and of the
  "r??+" label semantics (the fork's own revision marker is not defined in
  AGENTS.md; `r??` deliberately unknown).
- **Exact revision numbers** for Q-gain naming (2), unified floats (3),
  control-flow convention (5) — only "r15+" (cruise throttle v2, Section 4) is
  pinned.
- **Who/why the `UAVXArm32F4` reference tree was frozen** — the fork decision
  to treat it as legacy is documented only as a repo-layout fact, not a dated
  decision.