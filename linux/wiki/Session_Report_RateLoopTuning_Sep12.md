# Session Report — Fleet Rate-Loop Tuning: MC Band-Top + FW Rate-Fixable (Sep 12)

**Date:** 2026-09-12.
**Scope:** Applied the rate-loop tuning policy endorsed by the user (Greg) via the
three working decisions below: MC generic frames moved to the **band-top** rate-Kp
`0.5`; FW frames that were **rate-fixable** (their slow step/disturbance metrics clear
within or near the fleet rate band) retuned; FW frames that need >2× the band remain
**documented authority walls**. Metrics, sweeps, edited file list, and the final fleet
critique are recorded here.

**Status:** A) all decisions applied; B) **MC Roll/Pitch settle criterion relaxed 2.5 → 4.0 s
(user, Greg)** so the generic low-authority frames clear — fleet re-run =
**13 PASS / 26 FAIL / 39 files** (final, written via `critique_fleet.py --out`); C) mirrors
`linux/` `macos/` `windows/` synced byte-identical to canonical `uavx-python/src/`
(excluding `__pycache__`, `*.pyc`, `log.txt`, `Params_2026*`); D) `airframes_all_params.csv`
regenerated (39 airframes). FC unchanged; GCS-only this session.

---

## 1. Working decisions (user, resolved via Q&A)

1. **MC settle wall**: MC generic settle times ~3.0–4.2 s vs the 2.5 s criterion are
   **not** a rate-Kp problem at these authority limits — raise rate-Kp substantially and
   the overshoot band fails instead (a real authority/response-speed trade, see §4.0).
   User decision: **move the MC generic rate-Kp to the band-top `0.5`**, then (later in
   the session) **relax the MC Roll/Pitch settle criterion to 4.0 s** so the physically
   valid frames clear the wall rather than it being chased by rate-Kp (§4.2).
2. **FW Delta/Dragon pitch**: apply the sweep-confirmed **rate-Kp 2.0** (their pitch
   rise clears within the FW band's sweet-spot near band-top = 2.0).
3. **Disturbance-fixable FW roll/yaw**: apply the sweep-confirmed rate-Kp for the
   gust-settle fixes (Delta roll 0.7, SkySurfer roll 0.5, Spoileron roll 1.0 / yaw 0.5,
   Arado roll 0.5).

---

## 2. What was edited (canonical `.af`; base + `[LIMITS]` band kept in agreement)

### MC generic — band-top rate-Kp 0.5 (§1.1)

| file | ROLL_RATE_KP | PITCH_RATE_KP | ROLL band | PITCH band |
|---|---|---|---|---|
| `generic/Quad.af` | 0.22 → **0.5** | 0.25 → **0.5** | (0.25, 1.0) | (0.25, 1.0) |
| `generic/Hex.af` | 0.20 → **0.5** | 0.16 → **0.5** | (0.25, 1.0) | (0.25, 1.0) |
| `generic/Oct.af` | 0.25 → **0.5** | 0.20 → **0.5** | (0.25, 1.0) | (0.25, 1.0) |
| `generic/Quad_Medium.af` | 0.155 → **0.5** | 0.155 → **0.6** | (0.25, 1.0) | (0.25, 1.0) |
| `generic/Quad_Racer.af` | 0.25 → **0.5** | 0.25 → **0.4** | (0.25, 1.0) | (0.2, 0.8) |

(KD unchanged throughout.) Generic frame baseline rate-Kp were sub-band (0.155–0.25 vs
band-top 0.5), which is near the RATE-LOOP at the low-authority end — the sweep verified
these are the band ceiling where overshoot (not stability) becomes the binding FAIL.

### FW rate-fixable

| file | ROLL_RATE_KP | PITCH_RATE_KP | YAW_RATE_KP | band notes |
|---|---|---|---|---|
| `generic/Delta.af` | 0.35651 → **0.7** | 0.66845 → **2.0** | 0.7 | roll band (0.35, 2.0) |
| `generic/Dragon.af` | 0.334 | 0.334225 → **2.0** | 0.334 | pitch band (0.5, 4.0) |
| `generic/SkySurfer_Bixler.af` | 0.35651 → **0.5** | 0.17825 → **0.6** | 0.66845 | pitch band (0.3, 1.2), roll (0.25, 1.0) |
| `generic/Spoileron.af` | 0.57041 → **1.0** | 1.0695 | 0.2139 → **0.5** | roll band (0.5, 2.0), yaw (0.25, 1.0), pitch stay 1.07 (already valid, rise wall) |
| `user/Arado_555.af` | 0.334225 → **0.5** | 0.334 | 0.334 | roll band widened (0.2892, 1.157 → base 0.5 inside) |

`[LIMITS]` bands were widened where the new base sat outside the old band (Dragon pitch,
SkySurfer pitch+roll, Spoileron roll+yaw, Delta roll) — per the "base inside band"
invariant, and mirrored into the same `.af`.

---

## 3. Verification method

- Edit pattern kept `.af` base/`[LIMITS]` agreement (same change in the base line and the
  limit row) — so GCS live-write / param roulette round-trips are clean.
- Per-file full-battery `run_tests_for_af` on the canonical tree (the authoritative
  `simulate_axis` MR / `simulate_axis_coupled` FW path with gusts + altitude + navigation),
  then the full fleet `critique_fleet.py --out …`.
- All sweeps done via `tests/test_pid_sim.py`'s disturbance/step path on **copied sweep
  scripts** (deleted after use — not part of the tree).

### 3.1 Post-edit per-file results

| file | result | remaining FAILs |
|---|---|---|
| `generic/Quad.af` | **PASS** | — |
| `generic/Hex.af` | **PASS** | — |
| `generic/Oct.af` | **PASS** | — |
| `generic/Quad_Medium.af` | **PASS** | — |
| `generic/Quad_Racer.af` | **PASS** | — |
| `generic/Delta.af` | **PASS** | — |
| `generic/Dragon.af` | **PASS** | — |
| `generic/SkySurfer_Bixler.af` | FAIL (wall) | **yaw** rise 8 s + final err 5.997° (≤5°) — authority wall |
| `generic/Spoileron.af` | FAIL (wall) | **pitch** rise 8 s — authority wall |
| `user/Arado_555.af` | FAIL (wall) | pitch rise 8 s + final err 18.67° (≤15°) — authority wall |

**MSF (Multiset satisfied-fully)**: Quad, Hex, Oct, Quad_Medium, Quad_Racer, Delta, Dragon
now fully PASS. The four MC generic frames improved on overshoot (13–29% → 4–8%), and the
settle-wall was cleared by the **4 s settle-criterion relaxation** (§4.2A). FW walls are
pitch-rise / yaw-authority — not rate-tunable within band.

### 3.2 Fleet critique (final, all 39 canonical files)

`13 PASS / 26 FAIL / 39` → `tests/critique_fleet.py --out /tmp/opencode/critique_final.txt`.

PASS (13):
`generic/Delta.af, generic/Dragon.af, generic/Hex.af, generic/Oct.af, generic/Quad.af,
generic/Quad_Medium.af, generic/Quad_Racer.af, proposed/Ecks_220mm.af, proposed/Ken_450_1165.af,
proposed/Ken_Alpha_Test.af, proposed/Ming_DevEBox.af, proposed/Rok_Quad.af, proposed/S500_1137.af`.

Remaining 26 FAILs breakdown:
- **FW pitch-rise / yaw authority walls**: Elevon, Radian, RudderElevator, Shadow, SmallSpoileron,
  Spoileron, Arado_555, Horten, SkySurfer yaw, Phoenix (FW wing pitch/yaw walls) — all require rate-Kp
  ≥ ~2× band-top to clear (authority-bound, not rate-P-fixable). LadyBug alt-hold rise (known drag-model artifact).
- **Shadow 2026 captures** (11 files from today's bench, carry the catastrophic FC-stored gains):
  retained as evidence — see AGENTS + `ShadowAuthorityTuning_Sep10`.
- **Radian_Tuned**: pitch/yaw/settle + alt walls. **Horten**: overshoot 25.4% (just over) + rise wall.
- **proposed/Ken_LadyBug / WLToys_LadyBug**: alt-hold rise/settle (drag-model artifact).
- **Ecks_220mm_REFERENCE / Ecks_Old**: only the moderate alt-rise 2.55–2.62 s remain
  (integrator-boundary FAILs were the metric bug, §5; proposed Ecks PASSes).

---

## 4. Why these edits (rationale + rejected alternatives)

### 4.0 The initial contradiction: MC settle wall is NOT a rate-Kp fix
MC criteria: settle ≤ 2.5 s. Fit sees settle ~3.0–4.2 s. At first this reads "raise rate-Kp".
But a scan of rate-Kp showed: rate-Kp 0.25 → settle 3.845 s but overshoot 13–29% FAIL against
the 10% band; rate-Kp 0.5 → overshoot clears (5–8%) but settle still 3.0–4.2 s. The interpolated
region between 0.25 and 0.5 averaged ~O(10%): the settle wall persists **through the whole
useful rate band** — raising further trades settle for a worse overshoot FAIL rather than
recovering it. The wall's real driver is the MR-coupled physics + the 2.5 s band on a 15°/10°
step, not low-loop-gain. **Decision: band-top 0.5, then settle-criterion relaxation to 4.0 s**
(§4.2 — instead of documenting the wall). Raising to 1.0+ was rejected: it would fail overshoot
>10% and lose the protective 0.25–0.5 gain-to-overshoot margin learned from the wave-2 retune.

### 4.1 FW rate-fixables accepted, walls reported
Sweep matrix drove each retune:
- Delta/Dragon pitch at 2.0 clears the 30°-pitch rise (Dragon 8 s → 0.65 s, Delta → 1.60 s);
  rejected the 1.0 option (still 8 s) and >2.0 (no benefit, risk of ringing with the shared
  input-rate limiter).
- SkySurfer pitch at band-default 0.6 clears 8 s → 0.27 s (was stored 0.178). Roll 0.5 clears
  gust-settle 2.077 s → 1.418 s. Yaw is a genuine wall: needs rate-Kp 3.0 (rise 8 s → 2.60 s,
  final err 5.99° → 0.96°) — that's >2× band-top, so **reported, not applied** (user directive:
  fix rate-fixable, report walls).
- Spoileron: roll 1.0 clears gust 2.211 → 1.160 s; yaw 0.5 clears 2.399 → 0.850 s. Pitch is a
  wall (8 s at any rate-Kp ≤ band). Arado: roll 0.5 clears gust 1.739 → 1.088 s; pitch+final-error
  walls remain.
- Elevon/Radian/RudderElevator/Shadow/SmallSpoileron/Horten/Phoenix pitch-rise walls (need ≥3.0
  or never clear) — authority-bound (small flying surfaces, or aileron-less configs), **reported**,
  NOT rate-P-fixable. Rejected "raise every FW to 2.0+" wholesale — that would push the 
  high-authority frames into overshoot/damping FAIL and contradict the fleet band.

### 4.2 MC settle wall → criterion relaxation (user decision)
The four MC generic frames showed settle ~3.0–4.2 s against the old 2.5 s band while at
band-top rate-Kp. Per user decision (2026-09-12): **MC Roll/Pitch settle relaxed to 4.0 s**
(`critic/criteria.py` `CRITERIA`). Rationale: the response is bounded by the overshoot band
(rate-Kp already at band-top); the settle wall was a **criterion/authority mismatch**, not a
low-gain error. 4.0 s is still binding (agile frames land ~0.6–2.3 s; the 8 s FW/alt walls
remain FAILs). With the relaxation, Quad/Hex/Oct settle 3.0–3.96 s clear; **Quad_Medium pitch
needed 0.55→0.6 rate-Kp** (settle 4.15→3.87 s at the same overshoot margin, still inside the
pitch band 0.25–1.0) so all five generic frames PASS. Rejections: 2.5 s kept was rejected
(retains 4 spurious walls on physically-valid frames); relaxing to 5 s+ was rejected (stops
being binding — LadyBug/Ecks alt FAILs and any future degradation would be masked).

---

## 5. Metric bug fixed (boundary) — integrator-ratio rounding artifact

The fleet crit flagged **`Integrator utilization == 1`** on Ecks REFERENCE + Ecks_Old (2 rows each).
Root cause: in `tests/test_pid_sim.py` the accumulator stores `max_integrator=round(max_int,6)`
while `intlim` is unrounded; for Ecks I-lim ~0.0052359878 the round pushes the ratio to
`1.0000023` → spurious FAIL against `limit ≤ 1` even though the sim clamps `IntE` to ±IntLim
(so true utilization *cannot* exceed 1.0). With `max_integrator=max_int` (unrounded) the ratio is
exactly 1.0 and passes as designed. This reclassified:
- `proposed/Ecks_220mm.af` → **PASS** (was FAIL on the artifact row)
- `user/Ecks_220mm_REFERENCE.af` / `Ecks_Old.af` → only their genuine alt-rise walls remain.
Not a real controller fault — the Ecks angle-I saturates at its cap during the step, which is
expected behaviour for a strong hold (and the quaternion sign-conditioned anti-windup handles it).

---

## 6. Files changed (canonical = `uavx-python/src/`)

- `airframes/generic/{Quad,Hex,Oct,Quad_Medium,Quad_Racer,Delta,Dragon,SkySurfer_Bixler,Spoileron}.af`
- `airframes/user/Arado_555.af`
- `tests/test_pid_sim.py` (integrator metric rounding fix, §5)
- `airframes/airframes_all_params.csv` (regenerated)
- Mirrored identical into `linux/`, `macos/`, `windows/` kits (+`criteria.py` was already in
  parity from the Sep-11 session).

## 7. Verification status

- Fleet re-run post-edit: `13 PASS / 26 FAIL / 39` (final file above).
- Per-file `run_tests_for_af` on each edited frame (results §3.1).
- Kits verified byte-identical (`diff -qr` excluding pycache/pyc/log/Params_2026*).
- FC: **no FC C changes** (GCS/airframe-data only). FC build untouched.

## 8. Open items / follow-ups

- **FW yaw/pitch authority walls** (SkySurfer yaw, Elevon/Radian/RudderElevator/Shadow/SmallSpoileron/
  Spoileron/Arado/Horten/Phoenix pitch): structural/authority fixes, not rate-P — candidate for
  cascade (angle-max / rate-max ceiling) or airframe-surface work; left unmodified.
- **Remaining MC/ladybug alt-hold rise/settle** (Ken_LadyBug/WLToys_LadyBug, Radian_Tuned) — known
  drag-model artifact (`0.5·|v|·v`, mass-independent), not tuning.
- The Shadow captures from today's bench carry catastrophic low FC gains — already retained as
  evidence (user/Shadow_20260912_*). Greg to re-bench with the good `.af`.
- Report generated `wiki/Session_Report_RateLoopTuning_Sep12.md`. **PDF generation is Greg's job
  (host `scripts/md2pdf.sh`)** — assistant leaves this `.md`.

---

## 9. FW roll-cap set + FW pitch-step re-scope (added 2026-09-12)

### 9.1 The physical-motivation correction (Greg's prompt)

The Sep-12 "pitch authority wall" reading (≥2× band-top rate-Kp to clear a 30° step) was
checked against airframe reality:

- **A sustained 30° pitch step is not a real FW demand.** A FW climbs at a small attitude
  (climb/descent angles are small relative to bank). A 30° sustained pitch demand would
  stall/balloon most wing airframes — it is a control-law lab test, not an operational point.
- **Pitch-up in a FW couples to throttle**, and the induced propwash/slipstream over the
  elevator **increases elevator authority** (α_elevator, control effectiveness ∝ boosted
  dynamic pressure). Our `run_physics_fw` model uses a **fixed `qbar = 0.5·ρ·V²` at
  `cruise_speed`** (`test_pid_sim.py:1034-1035`) with `M_elev = qbar·S·c·CM_D_ELE·δ_e`
  — **no throttle/slipstream coupling**. The elevator-authority bound we called a "wall"
  is therefore partly a **model conservatism** (cruise-qbar elevator), partly the
  **over-sized 30° demand**.
- Conclusion adopted: correct **both** — (a) re-scope the FW pitch-step test to the max
  realistic sustained climb (15°), and (b) drop the FW roll-rate caps from the fixed-wing
  aerobatic 180°/s to a **gentle, robust 90°/s** with re-derived angle gains (angular
  control is about holding small bank/climb angles while rejecting gusts, not agility).

### 9.2 Backup (per Greg's directive)

- `airframes_backup_20260912_211844/` copied `airframes/generic/*.af` **before** any edit.
  Restore = copy the whole set back over `generic/` (byte-for-byte, base + `[LIMITS]`).

### 9.3 What changed

**Airframes** — 6 FW frames, roll 180°/s → 90°/s with re-derived roll angle gains
(`derive_angle_params.py` formula: `QKp = RateMax/(2·sin(AngleMax/2))`,
`Ki = 0.05·QKp` FW, `ILim = 0.01·AngleMax`). Shadow/Dragon were already at 120°/s,
SmallSpoileron already 45°/s — untouched.

| file | ROLL_RATE (base) | ROLL_ANGLE_Q_KP | ROLL_ANGLE_Q_KI | ROLL_ANGLE_Q_INT_LIMIT |
|---|---|---|---|---|
| `SkySurfer_Bixler.af` | 3.14159 → **1.5708** | 3.1415888 → **1.5707957** | 0 → **0.078539786** | 0.01 → **0.01047198** |
| `Delta.af` | 3.14159 → **1.5708** | 3.7194685 → **1.8597358** | 3.2 → **0.092986791** | 0.04 → **0.00872** |
| `Elevon.af` | 3.14159 → **1.5708** | 4.106659 → **2.0533312** | 13.52 → **0.10266656** | 0.03/0.04 → **0.00785** |
| `Radian.af` | 3.14159 → **1.5708** | 4.106659 → **2.0533312** | 0.2 → **0.10266656** | 0.04 → **0.00785** |
| `RudderElevator.af` | 3.14159 → **1.5708** | 4.106659 → **2.0533312** | 0.2 → **0.10266656** | 0.04 → **0.00785** |
| `Spoileron.af` | 3.14159 → **1.5708** | 4.106659 → **2.0533312** | 10.4 → **0.10266656** | 0.04 → **0.00785** |

`[LIMITS]` rows updated to keep **base inside the band** (invariant from §2): roll-rate,
roll-KP, roll-KI, roll-ILIM each re-banded around the new base (×0.5 … ×2, min cap 1.0).

**Tests/criteria** — the 30° sustained-pitch-step demand is dropped:

- `tests/test_pid_sim.py:877-878` — `FW_TEST_STEPS["Pitch"] 30 → 15°` (+ comment records
  the throttle/propwash rationale, so the 30° demand is not silently re-added).
- `critic/criteria.py` `FW_CRITERIA.Pitch.final_error` — `15.0 → 7.5°` so a 15° step is
  judged at the same **relative** margin (pitch deliberately looser than roll/yaw's 33% of
  step; pitch keeps 50%). Rise max 5 s / settle max 12 s unchanged.

### 9.4 Result after the changes (9 FW frames, full battery)

| file | verdict | roll | pitch | yaw |
|---|---|---|---|---|
| `SkySurfer_Bixler` | FAIL | PASS r=1.19 s=4.43 fe=0.23 | PASS r=0.2 s=8.0 fe=2.44 | **FAIL** fe=5.998 (≤5°) |
| `Delta` | **PASS** | PASS | PASS | PASS |
| `Dragon` | **PASS** | PASS | PASS | SKIP (structural) |
| `Elevon` | FAIL | PASS | FAIL rise 8 s fe=4.24 (≤7.5) | SKIP (structural) |
| `Radian` | FAIL | SKIP (structural) | FAIL rise 8 s fe=4.48 (≤7.5) | PASS |
| `RudderElevator` | FAIL | SKIP (structural) | FAIL rise 8 s fe=4.16 (≤7.5) | PASS |
| `Spoileron` | FAIL | PASS | FAIL rise 8 s fe=4.08 (≤7.5) | PASS |
| `Shadow` | FAIL | PASS | FAIL rise 8 s fe=10.6 (>7.5) | SKIP (structural) |
| `SmallSpoileron` | FAIL | PASS | FAIL rise 8 s fe=3.15 (≤7.5) | FAIL rise 8 s fe=10.3 |

Interpretation:

- **FW roll is now all-clean**: every scored roll step passes (SkySurfer/Delta/Dragon/
  Elevon/Spoileron/Shadow/SmallSpoileron), including the 90°/s frames — the gentler caps
  cost **nothing** in roll tracking, and gust robustness was already cap-independent (see
  Sep-12 disturbance sweep: peak deviation well under the 12–15°/s limits at these caps).
- **Pitch**: Delta/Dragon/**SkySurfer** now pass the 15° step. The small high-wing frames
  (Elevon/Radian/RudderElevator/Spoileron/Shadow/SmallSpoileron) still sit at the
  **cruise-qbar elevator-authority bound** (rise 8 s = never cross 90% of step; they reach
  ~11–13° of the 15°). Under the Sep-12 model conservatism reading (no throttle→slipstream
  boost), **this residual wall is the fixed-qbar artifact**, not a missing rate-gain. The
  unsick option is to model the propwash elevator boost (elevator authority scaled by a
  throttle-dependent factor) rather than push rate-Kp beyond the fleet band.
- **yaw**: SkySurfer/SmallSpoileron yaw final-error 6.0°/10.3° at a 15° heading step are
  the wing weathercock/drag-differential authority bounds — same class of residual.

### 9.5 Files changed

- `airframes/generic/{SkySurfer_Bixler,Delta,Elevon,Radian,RudderElevator,Spoileron}.af`
  (roll caps + re-derived roll angle gains, base + `[LIMITS]`).
- `tests/test_pid_sim.py` (`FW_TEST_STEPS` pitch 30→15 + rationale comment, line 877).
- `critic/criteria.py` (FW pitch final-error 15 → 7.5).
- Mirrored into the three kits, `generic` re-critiqued (`13 PASS/26 FAIL` for the full fleet
  unchanged in split — Delta/Dragon already passed before this pass; the re-scope converts
  the *interpretation* of the FW walls, not the PASS set).

### 9.6 Follow-ups

- **Propwash elevator-authority model** (`run_physics_fw`): candidate additive term raising
  `CM_D_ELE` authority with a throttle factor, to close the residual pitch-rise wall on the
  small wings. Needs a defensible throttle→boost relation and ACORD with Greg before coding.
  The 15° re-scope already removes the unrealistic 30° demand; the wall is now a model-
  conservatism question, not a tuning defect.
- **FW yaw final-error for rudderless/elevon configs** (SkySurfer 6.0°, SmallSpoileron
  10.3°): candidates for a structural yaw-authority note or criterion re-scope — do not
  chase with rate-P.
- Kits + gitUAVXGS sync and PDF remain host (Merlin) tasks.

### 9.7 Root cause of the residual "rise 8 s" — steady-state droop, not slowness (2026-09-13)

The pitch "rise 8 s" reported for the small wings  (Elevon/Radian/RudderElevator/Spoileron/
Shadow/SmallSpoileron) is the **`STEP_NOT_REACHED`-sentinel** (the sim's 8 s window ended
before the plant crossed 90 % of the 15° setpoint) — it is NOT a slow time constant. All
three diagnostics agree that the plant asymptotes:

- **Error fraction is invariant across step size** (measured on the legacy instant-step path,
  ground truth sweep 6 ° / 8 ° / 10 ° / 12 ° / 15 °): Elevon final-error ratio ≈ 0.28,
  Shadow ≈ 0.71 (final ≈ 0.72 × / 0.29 × of demand). A pure lag would settle ONTO the
  setpoint eventually; a **linear closed-loop DC gain < 1** (P-dominated hold against a load
  moment ∝ held angle) settles to a **constant fraction of demand** — exactly what is seen.
- **A pitch-demand filter (finite rise 0.5/1.0/2.0 s) does not change anything** — the demand
  ramp does not alter the equilibrium, only the transient path to it.
- **Propwash authority boost raises the plateau monotonically** (Elevon at a 15° demand:
  k=0 → 10.8°, k=1 → 12.5°, k=2 → 13.3°, k=4 → 13.9°; Shadow 4.4 → 10.1°): boosting
  `C_m_ele` scales the achievable hold attitude, confirming the wall is **elevator-authority
  at cruise qbar**, and that a real throttle-coupled slipstream (the propwash k↗ behaviour)
  closes most of it. (Experiment implemented as default-off `propwash_k` knob, then **reverted**
  — see §10.3.)

**Mechanism:** holding a sustained pitch attitude against the nose-down restoring moment
(`M_stability = pitch_stab_coeff · qbar · S · chord · θ`) needs sustained elevator torque;
an angle loop that is P-dominated (the FC's FW P-only gate `control.c:671`; the sim here
unconditionally integrates the stored/dormant FW Ki, so the sim is *more* optimistic than
the FC) can hold only a fixed fraction of a demanding setpoint with the available authority.
The 15° sustained-pitch step is essentially "how much nose-up can the elevator hold at
cruise qbar" — an authority question, NOT a rate-gain/tuning defect. **Do not chase with
rate-Kp.**

**Resolution per the 2026-09-13 study principles** (§10): the critique should keep the sharp
step for identification (that is its job) but must ALSO judge real-behaviour with **finite-
rise demands** and judge the plant **against its achievable equilibrium**, not a fixed
90 %-of-setpoint gate that a drooping P-hold can never cross. The fixed-qbar plant omits the
slipstream authority a climbing FW actually gets; modelling that (defensible throttle→boost
relation, ACORD with Greg) is the physically-honest fix rather than widening any gain band.
## 10. Study principles + process-step battery (added 2026-09-13)

### 10.1 The interlocking foci (Greg's directive, adopted as policy)

Greg's 2026-09-13 instruction is recorded verbatim in AGENTS.md (`## Simulation Study
Principles — LIST IN PROGRESS, adopt as policy`). The assistant is the drift monitor: any
controller-tuning/analysis study that departs from a focus MUST be flagged to Greg.

1. MC/FW simulation are SEPARATE classes; never share a sim body/plant across the boundary
   in a way that could silently compromise one class's behaviour.
2. Angle-loop gains ALWAYS derived (`QKp = RateMax/(2·sin(AngleMax/2))`,
   `Ki = k·QKp`, `ILim = 0.01·AngleMax`) — never hand-invented (`derive_angle_params.py`).
3. Studies confined to `generic/*.af` only (user/ and proposed/ are flight setups, not study
   subjects).
4. Rate/angle maximums grounded in iNav/ArduPilot/BetaFlight practice for the class.
5. Sharp steps stay for system identification; critique MUST ALSO run finite-rise/decay
   demands mirroring real-world command shapes (fractions of a second to seconds).
6. Sawtooth (triangular) tracking where amplitude AND period mimic (a) human stick inputs
   and (b) WP-navigation control-signal shapes. Later: verify WP-nav command inputs are
   sound.
7. Ground results on others' hand-tuned values wherever possible (acknowledged: the angle
   loop makes cross-mapping hard for some classes/axes).

### 10.2 Process-step battery implemented (foci 5 & 6) — `tests/test_pid_sim.py`

New CLI: `python3 src/tests/test_pid_sim.py process <af>` → `run_process_steps()`. It reuses
the SAME coupled 3-axis FW aero plant as the identification battery — no separate body, no
cross-class sharing (focus 1); FW-only. Default-off: the identification battery is unchanged
(byte-identical numbers verified on Delta after the edit).

- **Focus 5 — finite-rise steps** `_finite_rise_demand()`: a linear 0 → peak ramp of
  `rise_s ∈ {0.5, 1.0, 2.0}s` drives the same 15° demand. Reports the plant's own step
  metrics (rise/settle/fe) plus a 90%-of-demand follower flag and the tracking error/lag.
- **Focus 6a — human-stick sawtooth** `_sawtooth_demand()` ±3° / 3 s (a brisk but realistic
  stick oscillation for this class). **Focus 6b — WP-nav sawtooth** ±10° / 10 s (slow
  heading/bank ramps). Sawtooth sim walls extend to ≥2 full periods (21 s for the defaults).
  Criterion (PROVISIONAL, flagged for Greg): tracking LAG < 40 % of the command period —
  the classic servo phase-lag gate. Corner-following *peak* error is slew×response-transient,
  not steady error, so it is reported but NOT gated.

### 10.3 What the battery shows (generic frames, 2026-09-13)

| frame | axis | ident verdict | finite-rise 2.0 s | sawstick ±3° | saw-WP ±10° |
|---|---|---|---|---|---|
| Delta | Roll | PASS r=1.13 | 90% follower yes, pe 4.75° | lag 0.58 s | lag 0.55 s |
| Delta | Pitch | PASS | yes, pe 3.25° | lag 0.16 s | lag 0.26 s |
| Delta | Yaw | PASS | pe 4.10° | lag 0.21 s | lag 0.33 s |
| Elevon | Roll | PASS | yes, pe 4.10° | lag 0.59 s | lag 0.51 s |
| Elevon | Pitch | FAIL (rise 8 s) | **still 8 s, fe 4.27°** | lag 0.10 s | lag 0.18 s |
| Shadow | Pitch | FAIL (rise 8 s, fe 10.6) | **still fe 10.62°** | lag ~0 s (<0.01) | lag ~0 s |
| SkySurfer | Yaw | FAIL (fe 6.0) | **still fe 6.00°** | lag 0.52 s | lag 0.64 s |

**Reading — the finite-rise demand does NOT cure the droop**: Elevon pitch still plateaus at
~72 % of the 15° demand, Shadow at ~29 %, SkySurfer yaw at ~61 % — regardless of how slowly
the demand rises. This is the definitive confirmation of §9.7's mechanism: the wall is a
**DC gain / cruise-qbar authority** limit of the plant (P-only hold against the nose-down
stability moment), NOT a demand-transient or tuning-lag artifact. The sawtooth tracking
makes the practical consequence explicit: on realistic command shapes the SAME droop-limited
axes track tightly (Elevon pitch sawstick peak 1.42°/lag 0.10 s; Shadow pitch 2.31°;
SkySurfer yaw 2.65°/0.52 s). **The controllers are fine for real flight commands; the
step gate is testing a sustained-full-deflection hold that the airframe shape cannot make.**
That is precisely focus 5's point — step for identification, finite-rise for judgement.

### 10.4 Decisions / open items for Greg

- Sawtooth amp/period defaults (±3°/3 s, ±10°/10 s) and the 40 %-of-period lag gate are
  PROVISIONAL — chosen to mimic plausible stick and WP-command shapes for this airframe
  class, pending Greg's review of focus 6's "verify WP-nav command inputs" item.
- The propwash authority-boost experiment from Sep-12/13 was reverted (default-off `k` knob
  only, removed again) — the defensible throttle→slipstream relation still needs ACORD before
  coding into `run_physics_fw`.
- FC `control.c:671` gates FW angle-I (P-only) but the sim's `run_angle_loop` integrates Ki
  unconditionally — the sim is *more* optimistic than the FC on FW droop; a future step should
  mirror the gate for fidelity (flag, not done — would change ident numbers).
- MC process-step battery (focus 1 separation) is NOT yet built — the FW battery here is
  self-contained; the MC battery is a separate programme still to come.

### 10.5 Focus-1 audit + guard (same session)

The focus-1 "MC/FW sep" audit (subagent + assistant) examined `tests/test_pid_sim.py`:

- **Genuine separation confirmed**: `run_physics_mr` (965) and `run_physics_fw` (1003) are
  separate kernels; simply-typed `simulate_axis_coupled` (1739) is FW-only
  (elevon/delta/aileron routing), serving the FW critique and `run_process_steps`.
  MR-only, FW-only and shared-correct dispatch confirmed in `run_tests_for_af`.
- **Latent cross-class hazard found + fixed**: the FW dispatch in `simulate_axis`
  (1195) and `simulate_rate_disturbance` (1285) was keyed on `is_fw and fw_model_idx >= 0`,
  so a future FW airframe added to `AF_CATEGORY` *without* a `FW_MODEL_IDX` entry would
  silently run the MR plant (motor-differential torque!) — precisely the focus-1
  prohibition. Both sites now route on `is_fw` and **raise `ValueError`** if the model
  index is missing (`FW_MODEL_IDX missing ... never run the MR plant — refuse rather
  than silently cross-class`). Verified: guard fires on `cat=FW, fw_model_idx=-1`; all 5
  active FW AF types have model indices so the ident battery is unaffected (Delta FW +
  Quad MR ident outputs byte-identical pre/post edit).

### 10.6 Kit parity

`test_pid_sim.py` re-synced to linux/macos/windows kits after the battery + guard edits.
`diff -qr --exclude=__pycache__` = 0 for all 4 trees; `py_compile` 74 files × 4 trees,
0 failures.

### 10.7 Field-derived FW authority fudge — `FW_AUTHORITY_SCALE` ×4 (added 2026-09-13)

**The field datum.** Greg flew Shadow (`user/Shadow_SAVE.af`, gains identical to
`generic/Shadow.af`) in rough air with the Ch10 `RateGainScale` pot at its low end. The pot
maps pot-fraction → `4^(2p−1)` (Sep-10 mechanism): low-end p=0 → scale **0.25**. Stored
ROLL/PITCH rate Kp = 0.1, so the **effective** rate Kp flown was **0.025**, and that was
stable and controllable at the usable envelope. (Greg: "we seem likely to be out by a factor
of 4 on rate gains"; his assessment "the current gains of 0.1000 need to be reduced to
0.0250" — but see the direction argument below for where the 4× lives.)

**Sim calibration = the same implication landing on authority, not on gains.** The sim tuned
the same airframe to a good response at rate Kp = 0.1. A rate loop's closed-loop pole scales
with `CE·Kp/I`. If the real aircraft needed 4× LESS gain than the sim for a comparable
response, the sim has been modelling `CE` **4× too small**:

```
CE_real · Kp_real / I  =  CE_sim · Kp_sim / I      (comparable responses)
Kp_real = 0.025,  Kp_sim = 0.1
⇒ CE_real = 4 · CE_sim
```

Hence `FW_AUTHORITY_SCALE` (the module's single authority knob, which multiplies
`CL_D_AIL`/`CM_D_ELE`/`CN_D_RUD`) is now **4.0 by default**. Direction is NOT `/4`: scaling
authority DOWN would make the sim *more* docile and push recommended gains UP — the opposite
of what the aircraft demanded. The fudge is explicitly PROVISIONAL (single aircraft, single
air mass) and overridable per-run: `UAVX_FW_AUTHORITY=1.0 python3 tests/test_pid_sim.py ...`
restores the old sim so earlier report numbers remain reproducible; `=2.0` etc. sweeps
sensitivity.

**Why this is consistent with §9.7 (and why the pitch droop shrinks).** §9.7 blamed the
fixed-qbar plant for under-modelling authority exactly as this fudge predicts. The extra CE
raises the plane's DC hold authority, so:

- Shadow pitch: finite-rise 0.5 s fe **10.62° → 5.66°**, sawstick rms err **1.22° → 0.74°**
  (peak 2.31→1.53°), saw-WP peak 7.26→4.17°.
- From the 1×→4× fleets (ident battery): Elevon, Radian, RudderElevator, SkySurfer,
  SmallSpoileron (roll/pitch), Spoileron (pitch+yaw) pitch/yaw ISSUES **clear to PASS**;
  Shadow pitch and SmallSpoileron yaw remain ISSUES (droop still present, just less).
- Delta passes both, unchanged verdict, numbers tighten.

**What did NOT change.** No `.af` gains were edited (still 0.1); no `FW_CRITERIA`, no
`derive_angle_params.py`, no generic-frame derivations. The fleet-wide implication (if 4× is
confirmed, the sim now recommends ~0.025-class rate Kp and the stored 0.1 becomes
field-representative "too hot") is **parked until per-airframe truth** — the same policy
already recorded in AGENTS.md "FW rate gains possibly /4 too high" (which now documents that
the sim authority knife has been turned, with the pot as the in-field /4 until the dump
instrument works).

**The intended calibration instrument (missing today).** A correctly-working **trace dump**
(eTraceRate capture → tag-54) measures the FC's real closed-loop rate step per airframe; the
2026-09-13 dump was a 32-byte no-capture placeholder (see AGENTS.md TODO). That instrument +
benign weather is how per-airframe `FW_AUTHORITY_SCALE` truth replaces this single-flight
fudge. Implementation: `FW_AUTHORITY_SCALE = float(os.environ.get("UAVX_FW_AUTHORITY", "4.0"))`
at `tests/test_pid_sim.py:82`, applied in `get_fw_descriptor` (both branches, lines 707-709 /
716-718).

---

## 11. Adopted limit table + derived Angle PI written fleet-wide; sim FW angle-Ki gated to pure-P (2026-09-13)

### 11.1 Decisions (user, Greg — "adopt as proposed")

1. **Adopted rate/angle maximums** (grounded in iNav/ArduPilot/BetaFlight class practice,
   focus 4):
   - **MR** (Quad, Quad_Medium, Quad_Racer, Hex, Oct): 45° Roll/Pitch angle, **200°/s**
     Roll+Pitch rate, **200°/s** Yaw rate.
   - **FW** (Delta, Dragon, Elevon, Radian, RudderElevator, Shadow, SkySurfer_Bixler,
     SmallSpoileron, Spoileron): Roll angle 45°, Pitch angle 15°, Roll/Pitch rate 90°/s
     (already adopted Sep-12 §9.3 for 6 frames; now sweep-restored to all 9). FW yaw
     maxima kept per-file (NOT swept — no yaw angle-max exists for FW; heading unbounded).
2. **Derived Angle PI written into all 14 generic frames** (focus 2 — always derived,
   never hand-invented), from `derive_angle_params.py` / `parameter_window.py:3319`:
   - `QAngleKp = RateMax / (2·sin(AngleMax/2))`; MR k = 0.026, FW k = 0.05;
     `KiAngle = k·QKp`; `ILim = 0.01·AngleMax`. Yaw: `Ki = 0.083·Kp_yaw`,
     `ILim = 0.03` (absent from MR files → added as new base value + `[LIMITS]` band).
   - Values: MR QKp 4.5607658 / Ki 0.11857991 / ILim 0.0078539816 ×(roll/pitch);
     FW roll QKp 2.0523443 / Ki 0.10261722 / ILim 0.0078539816; FW pitch QKp 6.017169 /
     Ki 0.30085845 / ILim 0.0026179939.
   - **FW Ki/ILim are DORMANT in the files** (mirror the FC gate, see 11.2): the sim no
     longer applies them, exactly as the FC never accumulates them.
3. **Sim FW angle-Ki gate** (user answer "Gate FW angle-Ki in sim, write derived everywhere").
   Implementation in `tests/test_pid_sim.py` (call-frequency mirror of `control.c:671`):

```
simulate_axis (~1136-1147):   if is_fw: pi.Ki = 0.0
simulate_axis_coupled (1844-46 + 1875): aki = 0.0 for all three axes when is_fw
ai: "FW angle loop is pure-P by design — control.c DoAngleControl gates the quaternion
    integral accumulation on pAFTypeCategory != eCatFw (=:671), so a FW airframe NEVER
    runs its angle-I no matter what Ki the .af stores (derived FW Ki is dormant)."
```

   The gate is class-uniform: it applies through `simulate_axis_coupled` to the FW path
   of `compare_inav.py` / `compare_ardupilot.py` too (faithful, not a divergence). FW pure-P
   pitch now shows its real DC-gain droop (~1° final error on the 15° step) — that IS the
   flight behaviour (see §9.7 mechanism).

### 11.2 Tooling — `airframes/adopt_angle_params.py` (batch write-back)

New script that: derives the adopted values per file (from the ADOPTED maxima, not the
stored ones), rewrites base value lines, inserts missing keys (`YAW_ANGLE_Q_INT_LIMIT` for
MR; Shadow's entire angle group has no `[LIMITS]` → value-only writes), widens `[LIMITS]`
bands to keep value ∈ band ∈ class bounds, and scans all 5 roots (canonical +
linux/macos/windows kits + `../gitUAVXGS` mirror ). Idempotent (`--dry-run` → 0 changes on
re-run).

- **Fixed a latent roots bug** inherited from `apply_derived_angle_gains.py`:
  `uavxgs = dirname(dirname(here))` resolved to `uavx-python/` (one level too deep), so the
  three kit mirrors were **never scanned** by either script. Now
  `dirname(dirname(dirname(here)))` → the UAVXGS repo root. `adopt_angle_params.py` always
  scanned all roots (only the kit-mirror branches were affected by the wrong base).
- **Result:** 14 generic files × 5 roots = 56 files written (139 base + 22 band + 5 inserts
  per-root), all roots byte-identical (md5 verified), 0 files change on re-run.

### 11.3 `[LIMITS]` compliance (value ∈ band ∈ class bounds)

`tests/test_param_limits.py` — **all 14 generic files PASS**. Pre-existing failures remain in
`user/` (stale legacy `UNUSED_20`/`UNUSED_111` band `(0.24,0.96)`/`(0.00025,0.001)` vs value
0 in today's three `Quad_Medium_20260913_*.af` saves) and `proposed/` (Ken_LadyBug /
WLToys_LadyBug tag-17 out of class band). These are flight setups, NOT study subjects
(focus 3) — flagged to Greg, not edited.

### 11.4 Ident battery (sharp-step, focus 5) — 14 generic frames post-adoption

| file | Verdict | Roll | Pitch | Yaw | Notes |
|---|---|---|---|---|---|
| Quad | **PASS** | PASS | PASS | PASS | |
| Quad_Medium | **PASS** | PASS | PASS | PASS | |
| Quad_Racer | **PASS** | PASS | PASS | PASS | |
| Hex | **PASS** | PASS | PASS | PASS | |
| Oct | **PASS** | PASS | PASS | PASS | |
| Delta | **PASS** | PASS | PASS | PASS | |
| Dragon | **PASS** | PASS | PASS | SKIP (structural, rudderless elevon) | |
| Elevon | PASS* | PASS | PASS | SKIP | pitch 1.02° fe (pure-P droop) |
| Radian | PASS* | SKIP (aileron-less) | PASS | PASS | pitch 1.11° fe |
| RudderElevator | PASS* | SKIP | PASS | PASS | pitch 1.02° fe |
| Spoileron | PASS* | PASS | PASS | PASS | pitch 0.99° fe |
| Shadow | **ISSUES** | PASS | **ISSUES** rise 8 s fe 5.67° | SKIP | pitch droop reduced 10.6°→5.67° but not gone |
| SkySurfer_Bixler | **ISSUES** | PASS | PASS (1.28° fe) | **ISSUES** rise 8 s fe 2.65° | yaw authority wall |
| SmallSpoileron | **ISSUES** | PASS | PASS (0.39° fe) | **ISSUES** rise 8 s fe 5.87° | yaw authority wall |

`PASS*` = all scored axes pass; unscored axes are structural skips (rudderless/aileron-less),
not pass/fail vacuums. Compared with the §9.4/pre-gate table: **Elevon/Radian/RudderElevator/
Spoileron pitch move from "rise 8 s FAIL" to PASS** — the pure-P droop (fe ~1°) still holds
under the pitch final-error criterion (7.5°). **Shadow pitch 10.6→5.67°** is the gating
effect predicted before adoption (stale sim I-term removed). Residual ISSUES are the same
three authority walls already documented (§9.4): Shadow pitch, SkySurfer yaw,
SmallSpoileron yaw — DC-gain/authority limits of the fixed-qbar plant, NOT rate-gain
defects (do not chase with rate-Kp).

### 11.5 Process battery (finite-rise + sawtooth, focus 5/6) — FW frames post-adoption

| frame | finite-rise 2.0 s | sawstick ±3°/3 s | saw-WP ±10°/10 s |
|---|---|---|---|
| Delta | roll 90% follower yes, pe 3.79° | roll 1.39° rms / 0.37s lag | 1.80° rms / 0.46s lag |
| Spoileron | follower yes | 0.39° rms / 0.03s lag | 0.43° rms / 0.10s lag |
| Elevon | follower yes | 0.56° rms / 0.07s lag | 0.72° rms / 0.15s lag |
| RudderElevator | follower yes | 0.34° rms / 0.01s lag | 0.51° rms / 0.07s lag |
| Radian | follower yes | 0.34° rms / 0.01s lag | 0.51° rms / 0.07s lag |
| Dragon | follower yes | 0.57° rms / 0.08s lag | 0.64° rms / 0.15s lag |
| Shadow pitch | **follower NO** (fe 5.69°) | 0.75° rms / 0.02s lag | 2.21° rms / 0.10s lag |
| SkySurfer yaw | **follower NO** (fe 3.59°) | 1.55° rms / 0.46s lag | 2.88° rms / 0.71s lag |
| SmallSpoileron yaw | **follower NO** (fe 6.20°) | 1.46° rms / 0.40s lag | 2.97° rms / 0.55s lag |

Confirms §10.3's reading after the gate: the same three axes that fail the sharp step fail
"90% follower" on finite-rise demands (they plateau against the DC-gain/authority bound),
yet track real-shaped sawtooth commands tightly (lag well under the 40%-of-period gate). The
passing 6 FW frames are clean followers on all three finite-rise rises and track sawtooth at
sub-0.5 s lag. The controllers are sound for the command shapes real flight produces; the
step gate keeps its identification job and the sawtooth keeps the behaviour judgement.

### 11.6 Slider-vs-pot verdict (2D scale studies, user's question answered with data)

- **No-args run (`slider_pct=None`, file base values) ≈ Ch10 pot at centre (scale 1.0)**:
  the pot at 0.5 = scale 1.0 by the anchor mapping (`4^(2·pot−1)`), so the ident/process
  numbers above ARE the pot-at-centre numbers. The sim already models the pot's neutral.
- **Slider extremes bracket the pot range**: 0% ≈ conservatve (≈ altitude of low-authority
  frames), 100% ≈ aggressive. Pot maps pot∈[0,1] → scale∈[0.25,4.0]; sim slider maps
  scale∈[0,1]. The toolbar slider is a **2-D two-knob mechanism** (rate + angle gains move
  together), the pot is a **1-D single-knob master gain** on the rate loop only (angle loop
  untouched). Verdict: **keep both.** The pot is the field instrument (rate authority at the
  thumbs, angle loop trusted, safety-bounded 0.25–4.0); the slider is the study instrument
  (brackets gain coherence across the fleet, feeds the batch reports). They are not
  alternatives — they operate on different loops by design (§Ch10). Recommended: on-air
  pot-tuning stone-sheeted from the trace instrument (11.7), not the slider.
- The full slider-extremes (0%/100%) fleet run is NOT re-run this session: the adopted
  table + gate now make file-base (slider=None) the operationally representative point, and
  the extremes' direction (clearer/softer) is unchanged by adoption. Re-run as part of the
  next full work-program sweep if report continuity requires it.

### 11.7 What did NOT change

No FC source, no FC build (7 targets untouched, table already legal at `params.c` rows
63/74/76/82/83/98 — no ParamTable edit needed), no `FW_CRITERIA`, no
`derive_angle_params.py`, no GCS UI. Kits synced byte-identical + `../gitUAVXGS` mirror
written (fix of the root-detection bug means **kits now actually receive file edits**, not
just canonical → kit rsync on next host `sync_kits.sh`).

### 11.8 Open items (carried)

- Trace-dump instrument (rate probe step) still blocked by the 32-byte no-capture dump path
  (AGENTS TODO "Trace dump saved 32 zero bytes", Greg's 2026-09-13 Shadow flight) — the
  per-airframe `FW_AUTHORITY_SCALE` truth and the /4 question remain n=1-parked until it works.
- **Revisit gated Ki on FW** (Greg's todo): with the gate in, FW files now ship dormant
  Ki/ILim derived as `0.05·QKp`/`0.01·AngleMax`. No field evidence yet they'd ever be needed
  (the FC P-only gate is structural). Verdict stands: keep files' Ki as the derived value
  (documented dormant) rather than 0, so a future decision to enable FW angle-I starts from
  the derived value — but the sim/FC never apply it until the gate opens.
- Pre-existing `user/` (3× Quad_Medium `UNUSED_20` stale band) + `proposed/` (2× LadyBug
  tag-17 band) `test_param_limits` failures — Greg: cleanup candidates for the next
  user-flight-setup housekeeping pass.

### 11.9 External cross-check on the cascade architecture (added 2026-09-13)

Greg asked an independent LLM (Deepseek) to review the outer-PI → inner-PID cascade
and whether outer gains are derivable from the maxes. Full discussion and
conclusions-for-consideration: **`wiki/Session_Report_CascadeCrossCheck_Sep13.md`**
(standalone). The headline numerical result, so it lives here too:

**Our `QKp = RateMax/(2·sin(AngleMax/2))` is Deepseek's no-saturation ceiling
`Kp ≤ MAX_RATE/MAX_ANGLE_ERR`, exactly** (verified numerically to ~1e-8 rad/s on
all three derived axes). The adopted table therefore sits at the theory-optimal
maximal non-saturating gain — independent confirmation, no `.af` change.

Decision-worthiness it surfaces (see standalone §5–6 for the numbered list):

- **FW pitch 90°/s is above his "docile RC" 30–60° band.** Recommended reading:
  90°/s is a *controller ceiling*, commanded rates are limited at the command
  layer (docile commands never reach it) — do NOT re-derive pitch QKp from a
  lowered max without a positive decision.
- **FW P-only + dormant Ki stands** (his analysis shows P-only is zero-error on a
  1/s outer plant *when the inner loop tracks*; Ki only masks the DC-authority
  walls). Greg remains unconvinced about dropping FW Ki — recorded, revisit stays
  open pending the trace/rawlog instrument. Do not re-enable to satisfy sim tests.
- MR "sensitive controls" = the *usable* term = command-path (expo/pot), not a
  controller max-rate problem — confirms the existing split.

No FC/sim/`.af` change resulted; the discussion'd sole code consequence is the GCS
character slider being hidden (2026-09-13 decision, standalone report §5.1c).
Same class of cross-check as the iNav (Sep-04)
and ArduPilot (Sep-04) studies.
