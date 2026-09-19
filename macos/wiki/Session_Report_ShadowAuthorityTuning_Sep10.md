# Session Report — Shadow Deflection→Rate Authority & REF4 Gain Critique (2026-09-10)

## Executive Finding

The sim model's assumed **deflection→rate authority** is ~**9–13× too weak** for the
Shadow REF4 airframe. Critiquing the REF4 gains at reduced authority (÷1, ÷2, ÷4 —
as Greg requested) FAILS on every flight axis, but the FAIL mechanism is **plant
authority starvation, not the gains**: even at 100% deflection the assumed plant can
only sustain ~9–20°/s. Greg's real-aircraft observation ("max pitch/roll rate at
~20% deflection"; "5% deflection causes hard loops on planks") is the calibration
backstop: at 20% deflection the real Shadow achieves ~120°/s where the model
predicts ~9–13°/s → **model is 9.3× (roll) to 13.1× (pitch) too weak**. The tuning
that "feels right" in the air only starts passing the sim once authority is scaled
*up* (**pitch converges fully at ~40×**). 

**Therefore the whole FW I/D tuning exercise is hostage to an uncalibrated authority
constant.** Before any further gain work we must (a) fly the trace capture to
measure true `rate_ss / Out_ss` per axis, and (b) research external wing/plank
authority modelling (see §Research Direction) to independently bound the
deflection→rate constant.

---

## What Was Done

### 1. REF4 tuning absorbed into `generic/Shadow.af` (per Greg)

Greg: "I do not want you to register Shadow_REF4.af" — so instead of adding the
file to the sim's airframe tables, **`generic/Shadow.af` was overwritten with the
`user/Shadow_REF4.af` content** (the tuning Greg validated in the air). The old S800
generic Shadow physics lives on in `airframes/backup_angleunits/Shadow.af`; user
tunings kept in `airframes/user/Shadow_REF3.af` (bench) and `Shadow_REF4.af`.

Key REF4 values brought in:
- `PHYS_AUW_G=1000`, span 1800 mm, chord 250 mm, area 0.45 m²,
  `I_roll=0.2700`, `I_pitch=0.275208`, `I_yaw=0.540`, cruise 8.4 m/s, qbar 43.6 Pa
- `ROLL/PITCH_RATE_KP=0.1`, `KD=0`; `ROLL/PITCH_ANGLE_Q_KP=4`, `KI=0`,
  `INT_LIMIT=0.01`; `YAW_RATE_KP=0.005`, `YAW_ANGLE_Q_KP=0.75...` (as saved)
- `CONFIG1_BITS=eUsingMag|eEnforceDriveSymmetry`; `TRACE_TYPE=1` (rate probe)
  armed on the REF4 (though the sim's descriptor does not consume these).

### 2. Sim descriptor → REF4 physics

`test_pid_sim.py::FW_AIRFRAMES["generic/Shadow.af"]` updated from the old S800
(0.5 kg, 0.82 m, 13 m/s) to the REF4 platform: mass 1.0, span 1.8, S 0.45, cruise
8.4, elevator/aileron area 0.0132 m² @ arm 0.55 m, max deflection 20°/axis,
`CL_D_AIL=0.025`, `CM_D_ELE=0.6`, pitch damp −8.0, yaw damp −0.08,
`pitch_damp_lin=0.02`, `servo_tau=0.08`. Sim inertia matches REF4 PHYS
(I_roll 0.2700, I_pitch 0.2752).

### 3. `FW_AUTHORITY_SCALE` knob

Per Greg's intent to sweep the deflection→rate assumption, a single multiplier
`FW_AUTHORITY_SCALE` (default 1.0) now scales `CL_D_AIL`, `CM_D_ELE`, `CN_D_RUD`
inside `get_fw_descriptor()` (both the merged-descriptor and resolved-physicals
paths). This isolates the "surface-to-torque" constant from the rest of the aero
descriptor so we can sweep it independently of tuning gains.

### 4. `critique_authority.py` driver

New GCS test driver: runs the full `run_tests_for_af` critique at a list of scales.
Usage:

```
python3 src/tests/critique_authority.py [AF] [scale...]     # default 1 0.5 0.25
```

Synopsis: 1× → FAIL, 0.5× → FAIL, 0.25× → FAIL (see §Results).

---

## Results — Reduced-Authority Critique (the ÷1/÷2/÷4 sweep)

| Scale | Roll @15° | Pitch @30° | Final (roll) | Peak roll rate | Final (pitch) | Peak pitch rate |
|---|---|---|---|---|---|---|
| 1.0 (÷1) | ISSUES | ISSUES | 14.2° | 7°/s | 7.7° | 8°/s |
| 0.5 (÷2) | ISSUES (18.8° fin) | ISSUES | 18.8° | 6°/s | 4.5° | 5°/s |
| 0.25 (÷4) | **PASS** (18.4° fin) | ISSUES | 18.4° | 5°/s | 2.5° | 3°/s |

At 1×:
- **Roll**: overshoot **31.1%** (limit 25%) — FAIL; rise 2.06 s PASS; settle 8.0 s
  PASS; final error 0.8° PASS; rate headroom 0.23 PASS. Peak rate only **7°/s** on
  a 15° demand → the loop is moving the surface into a weak-torque regime.
- **Pitch**: rise 8 s FAIL (limit 5 s); **final error 22.3° FAIL (limit 15°)**;
  peak rate **8°/s** on a 30° demand — the plant can't even reach setpoint.
- Disturbance gust response: settle-after-gust FAIL on both axes at every scale
  (8 s vs 1.5 s limit at 1×) — a starved plant recovers slowly.
- Alt hold + nav + free-flight + lateral modes unchanged or PASS (those don't
  depend on control surface torque).

**The trend is the diagnosis**: *less* authority → more FAIL. Roll only "passes" at
0.25× because it now moves so gently it never overshoots — but that is a sickly
plant, not a good loop. Reducing authority does not rescue the REF4 gains; it makes
the sim disagree worse with the real aircraft, which climbs at ~120°/s at 20%
deflection.

### Scale-up calibration (where the real aircraft lives)

| Scale | Pitch final | Pitch peak rate | Verdict |
|---|---|---|---|
| 5× | 18.9° | 18°/s | still <30° setpoint |
| 10× | 23.2° | 24°/s | ISSUES |
| 13× | 24.5° | 27°/s | ISSUES |
| 20× | 26.2° | 33°/s | ISSUES |
| 40× | 28.0° | 44°/s | **PASS** |

Pitch reaches its 30° setpoint only with authority scaled **~40×**. Roll already
looks healthy by 5× (rise 1.1 s, overshoot 24.7 %, final 0.27°) and 13× (rise
0.75 s, overshoot 14.3 %, settle 3.7 s).

## Modeled vs Real deflection→rate (the quantitative gap)

Sustained rate at 20% deflection for the *assumed* descriptor (qbar = 43 Pa):

| Axis | C_ctrl (Nm/rad·δ) | damping C_d | sust. rate @20% δ | real Shadow @20% δ | model factor off |
|---|---|---|---|---|---|
| Roll | `qbar·S·b·CL_D_AIL` = 0.875 | 1.20 (0.5·ρ·V·b³·0.04) | **12.9°/s** | ~120°/s | **9.3× too weak** |
| Pitch | `qbar·S·c·CM_D_ELE` = 2.917 | 8.00 | **9.1°/s** | ~120°/s | **13.1× too weak** |

The ~**10×** Greg sensed is real. The sim's assumed authority constant is the
single dominant source of error in the FW tuning analysis, and it is **directionally
opposite** to the gain tweaks being considered (KI/KD). Tuning I/D against a plant
that can't reach the commanded rate produces garbage — the loop saturates and the
metrics read as "slow/undershooting."

## Cross-Project Bracket — iNav / ArduPilot well-tuned rate gains (Greg 2026-09-10)

Greg's idea: we extracted the hand-tuned rate gains of iNav and ArduPilot (both
field-proven to fly well). If two well-tuned rate loops target the same closed-loop
bandwidth, `Kp·Ka ≈ 1/τ` is a constant (`Ka` = angular acceleration per unit output
fraction = exactly the authority in question), so the ratio of well-tuned Kp's
inverts the ratio of real-to-model authority: `S = Kp_model_close / Kp_theirs`.

### Method validation on MC (authority known-good)

| source | mapped Kp (per rad/s) |
|---|---|
| iNav MC Roll P=40 → (40/31)·(180/π)/500 | **0.1479** |
| ArduPilot Copter rate P=0.135 | **0.1350** |

Two independent well-tuned MC rate loops agree to ~**9.5%** in our units. The
mapping convention is therefore sound and our MR authority model is confirmed.
This is the sanity check — our MC tuning is OK, per Greg.

### FW result — the scatter is the finding

| source | mapped Kp | S = Kp_ours / Kp_theirs |
|---|---|---|
| our Shadow REF4 | 0.100 | — |
| iNav FW P=5 → (5/31)·(180/π)/500 | 0.01848 | **5.4×** |
| ArduPlane roll kp_ff 0.345 → ·(180/π)/4500 | 0.00439 | **22.8×** |
| ArduPlane pitch kp_ff 0.385 | 0.00490 | **20.4×** |

The two well-tuned FW projects disagree with each other by **4.2×** in mapped
units. **The FW rate-P comparison cannot pin the def→rate constant**: the scatter
is not our model's fault — iNav FW's P=5 rides on FF=50 (feedforward carries the
authority) and ArduPlane's kp_ff is a *derived* rate term inside an angle-P +
trim-I + aero-self-damping structure. Both are entangled with each project's FW
control philosophy, exactly the portability caveat the `compare_inav.py` /
`compare_ardupilot.py` reports documented. MC rate-P is a clean authority meter
(9.5% agreement); FW rate-P is not.

### What the bracket does tell us

All three independent estimates point **one way** — the model's def→rate is too
weak — and bracket it at **5–23×**:

- direct observation (max rate @ ~20% deflection): **9.3× roll / 13.1× pitch**
- iNav S = **5.4×**
- ArduPlane S = **20.4–22.8×**

"~10× too weak" sits comfortably inside the bracket, but 5.4 vs 22.8 is too wide
to commit a calibration constant from the gains alone. Tightening would need each
project's reference-airframe geometry (unavailable reliably) and is blocked by the
FF/angle-loop entanglement. **Role assigned: cross-project gains = confidence
bracket, not calibration source.** The flight trace remains the only measurement
that resolves it; once `rate_ss/Out_ss` is measured, iNav/ArduPilot mapped gains
become a *validation* check on the re-tuned sim instead of an extraction source.

## External Literature Search — deflection→rate authority on plank/elevon airframes (completed 2026-09-10)

Greg-directed external survey for the control→rate ratio, prioritising flying-wing /
elevon airframes closest to our generic Shadow (plank) and Shadow/Aero (tailed FW).
Sources: peer-reviewed plank/elevon FW-MAV papers, VLM/DATCOM derivations, flight-test
system-identification (SISO Bode gains and EKF control-derivative sets).

### Control-power derivatives (dimensionless, per rad) — the direct comparison

Our sim's assumed authority lives in `CL_D_AIL` / `CM_D_ELE` (per rad, applied to the
`get_fw_descriptor()` torque): roll **0.025**, pitch **0.6**.

| Source (type) | Clδa aileron/roll power | Cmδe elevator/pitch power |
|---|---|---|
| **our sim assumed** | **0.025** | **0.6** |
| Flying-wing FW-MAV, XFLR5 VLM (NUST, Micromachines 2020) | 0.121 | 0.380 |
| Belgrade 1.5 m flying-wing UAV, VLM + analytical (Milenković-Babić 2024) | 0.32 (biplane-analytical 0.303; paper: "typical 0.1–0.25/rad") | 0.554 (analytical 0.559) |
| "Freya" rudderless flying wing, DATCOM (2024) | — | 0.327 |
| Identified 6-DoF UAV via EKF (Grillo & Montano, JATM 2015) | 0.173 | 0.401 |
| Tecnam P92 flight test (2024) | ~0.21 | — |
| MUTT “Fenrir” mini (NASA flight-ID) | — | (q/δe ≈ 16 rad/s short-period, high scatter) |

**The split is the finding — and it is asymmetric:**

- **Roll authority is the sim's primary error source.** External aileron power band
  is **0.12–0.32 /rad**; our 0.025 sits **5–13× below** it. That matched the 9.3× roll
  deficit (and the 5–23× cross-project bracket) independently. Flight-ID'd values
  (0.17–0.21) sit mid-band → a defensible sim-scale is ~**0.15–0.20**.
- **Pitch authority is NOT the error.** External elevator power band is **0.33–0.56 /rad;
  our 0.6 is already at/above the band.** So the 13.1× pitch deficit is a **damping-model
  artifact** (`pitch_damp = -8.0` throttling the sustained rate), not an authority
  coefficient shortfall. Do not "fix" pitch by inflating `CM_D_ELE` further.

### Flight-identified deflection→rate static gains (the only true control-to-rate ratios found)

- **MAV40 elevon MAV, 40 cm span (Niño et al., J. Bionic Eng. 2007)** — frequency-domain
  ID from flight: **Δq/Δδe static gain 15.74 dB (≈ ×6.1, negative sense)** and
  **Δp/Δδa static gain 17.6 dB (≈ ×7.6)** in (rad/s)/rad, good coherence <2 Hz / <1.5 Hz.
  That is ~6–8°/s per ° elevation at MAV scale — a real measured anchor, though at a
  scale/speed far below our regime.
- **SkyHunter UAS 2 m / 4.5 kg EKF ID (Benyamen, KU 2019)** — flight-identified
  longitudinal derivatives showed **large flight-to-flight scatter driven by unsteady
  aero + sensor noise** (CLδe ≈ 0.157, Cmδe ≈ 0.40 typical). Caveat transferable to our
  trace capture: single-trip derivative estimates carry real uncertainty; treat the flight
  trace as a band, not a point.
- **Belgrade 1.5 m flying-wing design rules:** elevon span 66.7% / chord 25%, static
  margin 9.67%; trims with <12° elevon at >10 m/s; **take-off differential >3° covers the
  whole engine-torque roll compensation** → planks carry large elevons for a reason; the
  same paper notes flying wings need LARGER relative control surfaces (why its 0.32
  exceeds the 0.1–0.25 "typical" band).

### Method caveats on the predicted numbers

- **VLM/predicted authority reads LOW vs real:** panel/VLM tools underestimate control
  power up to ~43% at AoA vs RANS/windtunnel (Asaro et al., 2024) — so the XFLR5 and
  analytical numbers above are the *floor* of each airframe's real authority.
- **DATCOM values are conventional-tail-derived**, only partially transferable to
  elevon/config thresholds — used here as a floor only.
- **Direction is unanimous:** every external estimate of aileron/roll power is
  **≥ 4.8× above** the sim's assumed 0.025. No source supports the sim's roll number.

### What this resolves for calibration

1. Adopt a sim `CL_D_AIL` in the **0.15–0.20 /rad** band (external data bounded by
   the flight-ID anchor and the VLM-analytical values below/above, backing out the
   0.025 straggler) — this closes the roll deficit to
   ~1× and re-anchors the whole FW roll critique.
2. Leave `CM_D_ELE ≈ 0.6` (already in/above band) and instead **reduce the quadratic
   `pitch_damp`** toward a real-plank value as part of the re-baseline — the pitch
   "needs 40× authority" conclusion was a damping artifact.
3. The trace capture remains the referee: fit `rate_ss / Out_ss` against the corrected
   plant, keeping the SkyHunter scatter caveat in view.

## Recommended Next Steps

1. **Fly the trace** (immediate, agreed): REF4 has `TRACE_TYPE=1` armed and ch8
   go-ahead. Capture a rate step (and ideally a manual stick doublet) per axis →
   `rate_ss / Out_ss` gives the true deflection→rate per axis, per airspeed. This
   replaces the assumed `C_ctrl` — the only honest calibration.
2. **External research** (Greg-directed): **DONE** — surveyed how other flight-modelling
   work bounds the deflection→rate constant for plank/elevon airframes (VLM, DATCOM,
   flight-ID SISO gains, EKF derivative estimation). Result: our roll authority (0.025/rad)
   sits 5–13× below the external aileron-power band (0.12–0.32/rad); our pitch authority
   (0.6/rad) is already in-band (pitch shortfall is a damping artifact). See
   §External Literature Search. Remaining: fold the derived constants into the sim
   `get_fw_descriptor` as a re-baseline cross-check vs the flight trace.
3. Only then re-run the KI/KD sweep for roll/pitch at fixed P gains with a
   calibrated plant.
4. Then optionally re-baseline `generic/Shadow.af`'s aero coefficients once the
   flight-derived authority is known (adopt `CL_D_AIL ≈ 0.15–0.20`, reduce quadratic
   `pitch_damp`).

## Files Touched

- `uavx-python/src/airframes/generic/Shadow.af` — overwritten with REF4 tuning.
- `uavx-python/src/tests/test_pid_sim.py` — descriptor updated to REF4 physics +
  `FW_AUTHORITY_SCALE` knob (in `get_fw_descriptor`).
- `uavx-python/src/tests/critique_authority.py` — new authority-sweep driver.
- `wiki/Session_Report_ShadowAuthorityTuning_Sep10.md` — this report.

## Build/Verification

- GCS: `py_compile` clean on the touched Python files (test_pid_sim.py
  ran end-to-end at 8 scales; critique_authority.py ran end-to-end).
- No FC C changes in this session (no FC build run).

## Cross-References

- `Session_Report_ShadowTuning_Sep07.md` — the REF4 tuning campaign that produced
  these gains.
- `Session_Report_ShadowSoftenedLiveSense_Sep09.md` — servo-sense/live-tuning
  changes feeding the "feels right" judgment.
- `wiki/docs/critique_report.md` + `Session_Report_CritiqueRetune_Aug29.md` —
  the original FW/MR critique framework these verdicts come from.

## External Sources (control→rate authority)

- Niño et al., "Model Identification of a Micro Air Vehicle", J. Bionic Engineering
  4(4):227–236, 2007 — MAV40 (40 cm elevon MAV) frequency-domain flight ID: Δq/Δδe
  static gain 15.74 dB, Δp/Δδa 17.6 dB. https://jbe.jlu.edu.cn/EN/Y2007/V4/I4/227
- Benyamen, "Stability and Control Derivatives Identification for an Unmanned Aerial
  Vehicle with Low Cost Sensors Using an EKF Algorithm", MSc thesis, University of
  Kansas, 2019 — SkyHunter 2 m/4.5 kg flight-ID'd longitudinal derivatives, σ high
  from unsteady aero/noise. http://dissertations.umi.com/ku:16357
- Milenković-Babić, Flying wing UAV design (Belgrade, FME Trans. 2024) — elevon
  power Clδa 0.32 /rad, Cmδe 0.554 /rad, "typical aileron 0.1–0.25 /rad", elevon
  span 66.7%/chord 25%, take-off differential <3°.
- Grillo & Montano, "An EKF-Based Technique for On-Line Identification of UAS
  Parameters", JATeM, 2015 — identified 6-DoF set: Clδa 0.173, Cmδe 0.401, Cmq 10.28,
  Clp 0.386. https://doi.org/10.5028/jatm.v7i3.412
- Flying-wing FW-MAV XFLR5 VLM control derivatives (NUST, Micromachines 2020):
  Clδe 0.121, Cmδe 0.380 /rad, Cnδe 0.0205 /rad, CLδe 0.536 /rad.
- "Freya" rudderless flying wing, DATCOM: Cmδe ≈ 0.327 /rad.
- Tecnam P92 flight-test aileron power CTB/Clδa ≈ 0.21 /rad (conventional GA).
- Asaro et al. 2024 (VLM-vs-RANS): panel/VLM tools underestimate control power up to
  ~43% at AoA — predicted values are authority floors.
- MUTT "Fenrir" mini, NASA flight-ID (Udall thesis 2015, University of Minnesota):
  control→rate identification practice; q/δe short-period ~16 rad/s, scatter high.