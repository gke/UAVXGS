# UAVX Tuning Report — Original Airframe Revisit (raw + legacy)

**Date:** 2026-08-29
**Version:** 1.0
**Scope:** Revisit of `wiki/docs/critique_report.md` v4.0 (2026-08-08) — retune every
`original/` airframe from the clean baseline (the retired `user/_Tuned` fleet is **not**
resurrected). Delivers the current-vs-proposed tuning table airframe by airframe, with
every changed parameter shown as **raw FC float** (what the `.af` stores / what the FC
ParamData holds) **and** its **legacy integer-encoded** equivalent where one exists.
**Simulator:** current `tests/test_pid_sim.py` (MFD criteria below). PDF conversion is the
USER's job (host "Merlin", `scripts/md2pdf.sh`) — the assistant has no pandoc/xelatex.

**Result:** the 7 larger frames go **original 3–7 FAILs → proposed 0 FAILs**; the two
micros go **9/7 FAILs → 2/1**, and their sole remaining FAILs are a **hard Alt-hold
physics floor**, not a tuning deficiency (analytic match, see §6).

---

## 1. Were these frames tuned before, and did they pass?

No. The documentation record:

| Source | Claim | Truth for *these* originals |
|--------|-------|------------------------------|
| `tuning_study.md` v2.1 (2026-08-04) | "all 12 FW + 5 generic MR frames PASS" | That is the **generic/** fleet, not `original/`. |
| `MC_TUNING_REPORT.md` (2026-07-26) | 7 MR `_Tuned` identical behavior, attitudes PASS | Same generic/base-gains cohort; the `_Tuned` copies of *these* frames were 3–8 FAIL. |
| `critique_report.md` v3.1 (2026-08-06) | regenerated `user/_Tuned` fleet validated | `Phoenix_Tuned` **0/45 PASS**, `Radian_Tuned` 0/45, `Arado_555_Tuned` 1/44 (artifact). Every MR copy **failed**: Ecks/Ken450/KenAlpha/Ming/S500 3/42, Rok_Quad 4/41, Ken_LadyBug 7/36, WLToys 8/35. |
| `critique_report.md` v4.0 (2026-08-08) | `_Tuned` fleet retired | All `_Tuned` frames carried the cascade I-limit windup bomb (legacy raw `10` in `ANGLE_Q_INT_LIMIT`). Prior tuned results are invalid — retune restarts from `original/`. |
| `critique_report.md` v3.0 | `original/Phoenix.af` | **PASS** — the only one of the 9 ever passing before. |

So of these 9 frames only **Phoenix** had previously passed; everything else was a
retired/invalid retune or the generic fleet. This campaign is the first from clean
`original/`.

---

## 2. Simulation convention and the Q gain

The attitude outer loop is **quaternion**, FC applies `2.0f * Qa[a] * A[a].P.Kp`
(`control.c`). The critique sim reads the `*_ANGLE_Q_*` KP directly as a regular
proportional gain (`rate_desired = angle_error × KP`). This is valid for tuning because
for **small angles** `2·Qa·QGain ≈ angle_error·QGain` — so `RollAngleQKp`,
`PitchAngleQKp`, `YawAngleQKp` **are the quaternion gains**, and their numeric value is
the small-angle gain.

Consequence for this campaign:

- **The tuning lever is the RATE loop** (`RollRateKp/Kd`, `PitchRateKp/Kd`,
  `YawRateKp/Kd`) **and the altitude chain** (`AltPosKp`, `AltROCKp`, `AltVelKi`,
  `AltThrottleCompLimit`) — **none of these are Q-scaled**; they are plain PD/PI on rate
  error / altitude ROC, identical for MC and FW.
- **Q (angle) gains were deliberately left at their source values in every frame but the
  two micros**, where `YawAngleQKp` `2.25 → 5.0` (still below the MR envelope default 8;
  the outer yaw loop was under-gaining the insufficient yaw-rate loop underneath it).
  Every other `*_ANGLE_Q_*` value is unchanged (Rok_Quad's PitchAngleQKp experiment
  ended back at source 5.75 — net zero).

---

## 3. Criteria and limits

Scored by current `tests/test_pid_sim.py`:
`python3 -m tests.test_pid_sim proposed/<Frame>.af` (run from `uavx-python/src`).

- Attitude steps: Roll @ 15°, Pitch @ 10°, Yaw @ 45° — rise/overshoot/settle/ss-error;
  rate headroom; disturbance rejection gusts (peak deviation roll 15°/s, pitch 12°/s,
  yaw 12°/s).
- AH 5 m: rise ≤ 2.0 s, overshoot ≤ 20 %, settle ≤ 5.0 s, ss-error ≤ 1.0 m.
- Nav 10 m: rise ≤ 2.0 s, overshoot ≤ 20 %, settle ≤ 5.0 s.
- FW (Phoenix): separate FW criteria; aileron-less roll non-scored.

**All proposed values lie inside the current FC `ParamTable` / GCS `PARAM_LIMITS`
agreement** (verified against `parameters.py`), so **no ParamTable / PARAM_LIMITS edits
are required** for this retune:

| Param | idx | GCS limit | highest used | |
|-------|-----|-----------|--------------|---|
| RollRateKp | 0 | (0, 3.0) | 0.34 | ✓ |
| PitchRateKp | 5 | (0, 3.0) | 0.42 (Rok) | ✓ |
| YawRateKp | 10 | (0, 3.0) | 0.25 (micros) | ✓ |
| RollRateKd | 11 | (0, 0.2) | 0.008 | ✓ |
| PitchRateKd | 27 | (0, 0.2) | 0.012 (Rok) | ✓ |
| YawRateKd | 90 | (0, 0.05) | 0.015 (Phoenix) | ✓ |
| AltPosKp | 6 | (0, 3.66) | 2.2 | ✓ |
| AltVelKi (AltROCKi) | 101 | (0, 3.66) | 0.005 | ✓ |
| AltThrottleCompLimit | 102 | (0, 0.25) | 0.25 (= FC max) | ✓ |
| AltROCKp | 120 | (0, 3.66) | 1.3 | ✓ |
| YawAngleQKp | 96 | (0, 20.0) | 5.0 (micros, Q gain) | ✓ |

> Legacy column legend: `legacy = raw / PARAM_SCALES[idx]` (the historical
> integer-encoded meaning). `—` = the index is **not** legacy-tagged (unified-float-only
> parameter, no scale bridge exists — `AltROCKp` 120, `AltThrottleCompLimit` 102,
> `YawAngleQKp` 96). Scales: idx 0/5/10 `0.005`, 11/27 `0.0001`, 90 `2.5e-5`, 6 `0.0183`,
> 101 `0.00027`. See `wiki/Legacy_Scaling_Map.md`.

---

## 4. Per-airframe current vs proposed

Format: `idx  Param    raw old → raw new   [legacy old → legacy new]   comment`.

### 4.1 Ecks_220mm — 3 FAILs → 0

| idx | Param | raw old → new | legacy old → new |
|-----|-------|---------------|------------------|
| 0 | RollRateKp | 0.2000 → 0.3400 | 40.0000 → 68.0000 |
| 5 | PitchRateKp | 0.2500 → 0.3400 | 50.0000 → 68.0000 |
| 10 | YawRateKp | 0.03333 → 0.1100 | 6.6667 → 22.0000 |
| 11 | RollRateKd | 0.0045 → 0.0080 | 45.0000 → 80.0000 |
| 27 | PitchRateKd | 0.0045 → 0.0080 | 45.0000 → 80.0000 |
| 90 | YawRateKd | 0.0003375 → 0.0020 | 13.5000 → 80.0000 |
| 6 | AltPosKp | 1.8300 → 2.1000 | 100.0000 → 114.7541 |
| 101 | AltVelKi | 0.0005 → 0.0050 | 1.8519 → 18.5185 |
| 120 | AltROCKp | 1.0000 → 1.2000 | — → — |

Clears: yaw step (OS 17.5 %/settle 8 s), roll-gust (17.8°/s) & yaw-gust (17.2°/s), AH rise
(2.03 s → 1.95 s). AltThrottleCompLimit already 0.2 in the source (unchanged).

### 4.2 Ken_450_1165 — 3 FAILs → 0 (tuned in a prior session today)

| idx | Param | raw old → new | legacy old → new |
|-----|-------|---------------|------------------|
| 0 | RollRateKp | 0.2080 → 0.3400 | 41.6000 → 68.0000 |
| 5 | PitchRateKp | 0.2080 → 0.3200 | 41.6000 → 64.0000 |
| 10 | YawRateKp | 0.06667 → 0.1100 | 13.3333 → 22.0000 |
| 11 | RollRateKd | 0.0060 → 0.0080 | 60.0000 → 80.0000 |
| 90 | YawRateKd | 0.0003375 → 0.0020 | 13.5000 → 80.0000 |
| 6 | AltPosKp | 1.8300 → 2.1000 | 100.0000 → 114.7541 |
| 101 | AltVelKi | 0.0005 → 0.0050 | 1.8519 → 18.5185 |
| 102 | AltThrottleCompLimit | 0.1200 → 0.2000 | — → — |
| 120 | AltROCKp | 1.0000 → 1.2000 | — → — |

Carries one non-tuning delta from the Mag-Gating session: `CONFIG1_BITS` `eUsingMag →
6.0` (`UseMag` master switch on).

### 4.3 Ken_Alpha_Test — 3 FAILs → 0

| idx | Param | raw old → new | legacy old → new |
|-----|-------|---------------|------------------|
| 0 | RollRateKp | 0.2375 → 0.3400 | 47.5000 → 68.0000 |
| 5 | PitchRateKp | 0.2375 → 0.3400 | 47.5000 → 68.0000 |
| 10 | YawRateKp | 0.04167 → 0.1100 | 8.3333 → 22.0000 |
| 11 | RollRateKd | 0.0020 → 0.0080 | 20.0000 → 80.0000 |
| 27 | PitchRateKd | 0.0060 → 0.0080 | 60.0000 → 80.0000 |
| 90 | YawRateKd | 0.0003375 → 0.0020 | 13.5000 → 80.0000 |
| 6 | AltPosKp | 1.8300 → 2.1000 | 100.0000 → 114.7541 |
| 101 | AltVelKi | 0.0005 → 0.0050 | 1.8519 → 18.5185 |
| 102 | AltThrottleCompLimit | 0.0500 → 0.2000 | — → — |
| 120 | AltROCKp | 1.0000 → 1.2000 | — → — |

Clears the worst roll-gust of the fleet (19.31°/s) and AH rise (3.34 s → 1.85 s).

### 4.4 Ming_DevEBox — 6 FAILs → 0

| idx | Param | raw old → new | legacy old → new |
|-----|-------|---------------|------------------|
| 0 | RollRateKp | 0.2375 → 0.3400 | 47.5000 → 68.0000 |
| 5 | PitchRateKp | 0.2375 → 0.3400 | 47.5000 → 68.0000 |
| 10 | YawRateKp | 0.04167 → 0.1100 | 8.3333 → 22.0000 |
| 11 | RollRateKd | 0.0020 → 0.0080 | 20.0000 → 80.0000 |
| 27 | PitchRateKd | 0.0060 → 0.0080 | 60.0000 → 80.0000 |
| 90 | YawRateKd | 0.0003375 → 0.0020 | 13.5000 → 80.0000 |
| 6 | AltPosKp | 1.8300 → 2.2000 | 100.0000 → 120.2186 |
| 101 | AltVelKi | 0.0005 → 0.0050 | 1.8519 → 18.5185 |
| 120 | AltROCKp | 1.0000 → 1.3000 | — → — |

The only frame with step **overshoot** failures (roll 10.4 %, pitch 10.5 %, yaw 16.4 %).
Rate PD rebuild cleared all three; AH needed the slightly higher AltPosKp/AltROCKp
(2.2/1.3) to pull rise 2.014 → < 2 s.

### 4.5 Phoenix (FW) — 1 FAIL → 0

| idx | Param | raw old → new | legacy old → new |
|-----|-------|---------------|------------------|
| 10 | YawRateKp | 0.2852 → 1.1000 | 57.0400 → 220.0000 |
| 90 | YawRateKd | 0.00066 → 0.0150 | 26.4000 → 600.0000 |

FW yaw was critically under-gained (rate KP 0.285 vs the FW needs ≥ ~1). Yaw @ 15° rise
8 s → ≤ 5 s. Everything else (pitch, AH, Nav, structural roll gate) already passed.

### 4.6 Rok_Quad — 7 FAILs → 0

| idx | Param | raw old → new | legacy old → new |
|-----|-------|---------------|------------------|
| — | **AF_TYPE** | `eQuadAF` → `eQuadXAF` | fix: the file's own type (3) is not even in the sim `AF_CATEGORY`; `original/` only loads courtesy of a hard-coded loader override (`AF_FILES["original/Rok_Quad.af"]→eQuadXAF`). The proposed file must carry `eQuadXAF` (=4) itself to load/score and to select the right mixer on the FC. |
| 0 | RollRateKp | 0.2375 → 0.3400 | 47.5000 → 68.0000 |
| 5 | PitchRateKp | 0.2375 → 0.4200 | 47.5000 → 84.0000 |
| 10 | YawRateKp | 0.04167 → 0.1100 | 8.3333 → 22.0000 |
| 11 | RollRateKd | 0.0020 → 0.0080 | 20.0000 → 80.0000 |
| 27 | PitchRateKd | 0.0060 → 0.0120 | 60.0000 → 120.0000 |
| 90 | YawRateKd | 0.0003375 → 0.0020 | 13.5000 → 80.0000 |
| 6 | AltPosKp | 1.8300 → 2.2000 | 100.0000 → 120.2186 |
| 101 | AltVelKi | 0.0005 → 0.0050 | 1.8519 → 18.5185 |
| 102 | AltThrottleCompLimit | 0.0600 → 0.2500 | — → — |
| 120 | AltROCKp | 0.0500 → 1.3000 | — → — |

Worst overshoot frame (16.3/16.8/21.5 % across steps — new 1344 g / 9×4 prop mass, hover
≈ 55 %). Needed the heaviest rate-KD damping (**pitch KD 0.012**, pitch rate KP 0.42 —
the sim's "increase RateKp to cut overshoot" is the cascade counterintuitive: faster rate
loop, less angle integration past command) plus full `AltThrottleCompLimit` 0.25 to climb
the 5 m step in ≤ 2 s (rise 3.79 s → 2.00 s). Q gains untouched (PitchAngleQKp stayed
5.75; YawAngleQKp stayed 3). **This is the new-mass respray the critique's item 4 called
for, and the only frame needing a physicals retune.** (physicals were already updated in
`original/`.)

### 4.7 S500_1137 — 4 FAILs → 0

| idx | Param | raw old → new | legacy old → new |
|-----|-------|---------------|------------------|
| 0 | RollRateKp | 0.2000 → 0.3400 | 40.0000 → 68.0000 |
| 5 | PitchRateKp | 0.2500 → 0.3400 | 50.0000 → 68.0000 |
| 10 | YawRateKp | 0.03333 → 0.1100 | 6.6667 → 22.0000 |
| 11 | RollRateKd | 0.0045 → 0.0080 | 45.0000 → 80.0000 |
| 27 | PitchRateKd | 0.0045 → 0.0080 | 45.0000 → 80.0000 |
| 90 | YawRateKd | 0.0003375 → 0.0020 | 13.5000 → 80.0000 |
| 6 | AltPosKp | 1.8300 → 2.1000 | 100.0000 → 114.7541 |
| 101 | AltVelKi | 0.0005 → 0.0050 | 1.8519 → 18.5185 |
| 102 | AltThrottleCompLimit | 0.0500 → 0.2000 | — → — |
| 120 | AltROCKp | 1.0000 → 1.2000 | — → — |

Same lever set; yaw overshoot (17.8 %) and all gusts cleared.

### 4.8 Ken_LadyBug (micro) — 9 FAILs → 2

| idx | Param | raw old → new | legacy old → new |
|-----|-------|---------------|------------------|
| 96 | **YawAngleQKp (Q gain)** | 2.2500 → 5.0000 | — → — |
| 0 | RollRateKp | 0.2080 → 0.3000 | 41.6000 → 60.0000 |
| 5 | PitchRateKp | 0.2080 → 0.3000 | 41.6000 → 60.0000 |
| 10 | YawRateKp | 0.06667 → 0.2500 | 13.3333 → 50.0000 |
| 11 | RollRateKd | 0.0030 → 0.0080 | 30.0000 → 80.0000 |
| 27 | PitchRateKd | 0.0090 → 0.0080 | 90.0000 → 80.0000 |
| 90 | YawRateKd | 0.0000375 → 0.0020 | 1.5000 → 80.0000 |
| 6 | AltPosKp | 0.2000 → 2.1000 | 10.9290 → 114.7541 |
| 101 | AltVelKi | 0.0005 → 0.0050 | 1.8519 → 18.5185 |
| 102 | AltThrottleCompLimit | 0.0200 → 0.2500 | — → — |
| 120 | AltROCKp | 0.0500 → 1.2000 | — → — |

The critique's v3.0 NaN/inf divergence **no longer reproduces** in the current sim — the
frame now runs clean. 9 → 2 FAILs; the 8 cleared include the yaw step (rise 7.15 s →
2.51 s, settle 8 s → 3.58 s), roll-gust (19.2 → ≤ 15°/s), yaw-gust (13.5 → ≤ 12°/s) and
rate headroom (0.085 → 0.23). The 2 remaining are AH rise + settle — see §6 (physics
floor, ~6.9 s optimum).

### 4.9 WLToys_LadyBug (micro) — 7 FAILs → 1

Same 11 deltas as Ken_LadyBug (identical gain family): YawAngleQKp 2.25 → 5.0 (Q gain),
rate PD set, AltPosKp 0.2 → 2.1, AltVelKi → 0.005, AltThrottleCompLimit → 0.25,
AltROCKp → 1.2.

Clears the same 6 (yaw step, both gusts, headroom, all attitude). 1 remaining FAIL = AH
rise — §6.

---

## 5. Common tunings (the critique §Summary acknowledged pattern)

The `original/` MR failures distribute exactly as the critique predicted: **under-gained
yaw-rate loop** (settle 8 s), **roll/yaw gust peaks 16–19°/s**, **AH rise 2.0–3.8 s**.
The adopted solution per frame is a consistent lever set on the **rate PD** (roll/pitch
rate KP to 0.34, rate KD to 0.008, yaw-rate KD to 0.002 = old legacy 80) + the altitude
chain (`AltROCKp` 1.2–1.3, `AltPosKp` 2.1–2.2, `AltVelKi` 0.005, comp-limit 0.2–0.25),
with two local adjustments:

- **Ming** needed AltPosKp/AltROCKp one step higher (2.2/1.3) for the AH bar;
- **Rok_Quad** needed pitch-specific heavy damping (pitch KD 0.012, pitch rate KP 0.42)
  for its new-mass step overshoots + full comp-limit 0.25 for climb.

YawRateKp 0.11 is below the sim's MR envelope floor (0.20, guidance-only — the envelope
is for slider characterisation, not scored) and validates cleanly; the micros kept 0.25.
Q gains untouched except the micros' YawAngleQKp (5.0, still < envelope default 8).

---

## 6. Why the two micros cannot clear Alt Hold (physics floor, not tuning)

MR AH plant: `thr_total = hover + thr_comp`, `accel = thr_total·max_thrust/m − g − 0.5·v²/m`,
with `thr_comp` clamped to `AltThrottleCompLimit`. At the FC hard limit (0.25) the climb
reaches terminal velocity when drag `0.5·v²` equals excess thrust:

| Frame | mass / hover | excess thrust @ comp 0.25 | terminal v | 4.5 m optimum rise | sim |
|-------|--------------|---------------------------|------------|--------------------|-----|
| Ken_LadyBug | 30 g, hover 34 % (88 g thrust) | ≈ 0.216 N | 0.657 m/s | ≈ 6.9 s | **6.913 s** |
| WLToys_LadyBug | 80 g, hover 33 % (240 g thrust) | ≈ 0.589 N | 1.085 m/s | ≈ 4.3 s | **4.25 s** |

The sim matches the analytic optimum to 3 digits. The generic drag model (`0.5·v²`)
penalises these tiny brushed frames' terminal climb rate; no legal gain or comp-limit
(≤ 0.25 = FC max) can reach the 2 s rise bar for a 5 m step. **Documented as a model
floor** — the earlier critique already flagged the brushed divide; the remainder is now
confined to Alt Hold, with all attitude/yaw/nav-cleared.

Rok_Quad sits close to the same physics (hover ≈ 55 %), which is why it needed the full
0.25 comp-limit — its rise lands at 2.00 s, at the bar.

---

## 7. Verification

```bash
cd UAVXGS/uavx-python/src
python3 -m tests.test_pid_sim proposed/Ecks_220mm.af        # 0 FAIL
python3 -m tests.test_pid_sim proposed/Ken_450_1165.af      # 0 FAIL
python3 -m tests.test_pid_sim proposed/Ken_Alpha_Test.af    # 0 FAIL
python3 -m tests.test_pid_sim proposed/Ken_LadyBug.af       # 2 FAIL (AH physics floor, §6)
python3 -m tests.test_pid_sim proposed/Ming_DevEBox.af      # 0 FAIL
python3 -m tests.test_pid_sim proposed/Phoenix.af           # 0 FAIL
python3 -m tests.test_pid_sim proposed/Rok_Quad.af          # 0 FAIL
python3 -m tests.test_pid_sim proposed/S500_1137.af         # 0 FAIL
python3 -m tests.test_pid_sim proposed/WLToys_LadyBug.af    # 1 FAIL (AH rise, §6)
```

Original baselines (current sim, `original/<frame>.af`): Ecks 3 · Ken450 3 · KenAlpha 3 ·
Ken_LadyBug 9 · Ming 6 · Phoenix 1 · Rok_Quad 7 · S500 4 · WLToys 7.

> Path note: reference frames as `original/<name>.af` / `proposed/<name>.af` (relative to
> `src/`) — a bare `airframes/…` prefix breaks the loader's basename descriptor mapping.

## 8. Actions / decisions

1. **proposed/ = flight-ready candidates**, visible in the GCS load menus. `original/`
   stays as the untouched record (hidden).
2. No `params.c` ParamTable or GCS `PARAM_LIMITS` edits — every value sits inside the
   existing FC–GCS agreement (§3).
3. Never re-instantiate the retired `_Tuned` fleet (I-limit windup). This campaign's
   baseline is `original/` as committed.
4. Micro AH: captured as a model floor, not silently "fixed" by editing the sim drag —
   any future sim drag-scaling change must be justified separately and re-run through
   §7.