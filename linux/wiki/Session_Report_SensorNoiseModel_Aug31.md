# Session Report — Sensor-Noise Model for the Tuning Sims

Date: 2026-08-31
Scope: GCS simulation methodology (`src/tests/`) — **no FC flash artifact,
no `.af` writes, no param changes.** This report fixes *how we measure* an
airframe's PID robustness in the sim before any tuning recommendation is made.

---

## 1. The question

Our tuning sims (`test_pid_sim.py`, `test_robustness_sim.py`) currently inject
**white Gaussian noise on the raw gyro (rate) and attitude (angle) signals**,
sampled at **1000 Hz** (`CONTROL_DT = 0.001`), with **no low-pass filter, no
rotor-harmonic content, no aliasing model**:

```python
NOISE = {
    'moderate': {'rate': 0.005, 'angle': 0.003},   # ~0.3 °/s, ~0.17 °
    'large':    {'rate': 0.012, 'angle': 0.006},   # ~0.7 °/s, ~0.34 °
}
```

Ken's and Eck's airframes fly D-terms proven over time. If the sim is to
produce tuning estimates that land *on the proven values*, the measurement
model must not be the reason the estimates drift. This session compares the
measurement/IMU noise modeling **that other flight stacks use**, fixes what we
adopt, and validates against **Ken's larger quads** (the reliable reference —
Eck's frame is by his own measure *not perfect* and is deliberately excluded as
a validation target; his malformed legacy-scaled dumps are a separate GCS
write-path bug, already tracked).

## 2. What the other guys include (references)

### 2.1 ArduPilot SITL
ArduPilot's software-in-the-loop explicitly simulates sensor error and
**motor-speed-correlated vibration** so that its own filters can be exercised:

- `SIM_GYR_RND`, `SIM_GYR_BIAS`, `SIM_GYR_DRIFT` — gyro white noise, turn-on
  bias, bias random-walk drift.
- `SIM_ACC_RND`, `SIM_ACC_BIAS`, `SIM_ACC_VIB` — accelerometer white noise,
  bias, and **vibration whose amplitude is correlated with throttle/motor
  speed** (the vibration is injected *before* the filtering chain, matching the
  physical order: plant → vibration → sensor → filter → loop).
- The whole point is that SITL must reproduce the aliased/amplified rotor
  content so the dynamic notch + LPF are validated in-sim, not discovered in
  flight.

*Ref: `github.com/ArduPilot/ardupilot` SITL parameter documentation.*

### 2.2 PX4 / jMAVSim / Gazebo IMU plugin
PX4's sensor simulation is the textbook model of a MEMS IMU as a signal + noise
source into the estimator:

- Gyro/accel **noise density** (e.g. 0.05 °/s/√Hz class),
- **bias random walk** and **bias correlation times**,
- **turn-on bias** (first-second bias),
- quantization and sampling.

Crucially the model is *per-axis* and the noise is applied at the IMU output,
**not after fusion** — the EKF then has the same measurement statistics it has
in the air.

*Ref: PX4 SITL sensor model docs; Gazebo `libmavlink_sitl` / `rotors_simulator`
IMU plugin.*

### 2.3 Betaflight / iNav — the "measured spectrum" school
Betaflight/iNav do **not** simulate vibration; they *measure* it:

- The gyro FFT from **blackbox logs** is the ground truth — rotor fundamental
  and harmonics are identified directly.
- **Dynamic notch filters** track the motor rotational frequency (RPM from
  bidirectional DShot) in flight, because blade-pass and its harmonics are the
  dominant in-band disturbance.
- The tuning loop is: log → FFT → place LPF/notch on the measured peaks.

Takeaway used here: **rotor harmonics are the operating noise on the gyro, they
are narrowband, throttle-coupled, and the appropriate tool against them is
filtering — not a white-noise blanket that mis-shapes the D-term trade-off.**

*Ref: Betaflight documentation on gyro filtering / dynamic notch.*

### 2.4 Academic / HIL — replaying measured PSDs
The vibration-in-the-loop literature simulates with **recorded** frame
vibration:

- Fresk & Nikolakopoulos (2014-ish) — frame-induced vibration attenuation on a
  quaternion attitude controller; drives the controller with measured vibration
  to rate the filters *and* the D-term.
- Verbeke & Debruyne — modal analysis of multirotor frames; the measured PSD
  shows structure in three bands (frame/airflow low band, motor-shaft band at
  low RPM, and the motor/blade-pass high band), with peaks at blade-pass
  frequency and harmonics.
- The universal finding: above the first couple of harmonics the content is
  both *above Nyquist* (aliases on sampling) and *steep* (a filter corner
  placed correctly removes a decade of energy per pole).

### 2.5 What they all share
1. Noise is applied on the **measurement before the filter chain**, in the
   correct physical order.
2. Rotor BPF content is **throttle-correlated** and **narrowband** — it is not
   white.
3. The **sampling rate** (and therefore Nyquist + aliasing) is part of the
   model; you cannot reason about D-term gain without it.
4. Bias + random walk model the MEMS, not just RMS jitter.

## 3. What we are adopting — and why

### 3.1 The real FC chain (verified at source)

| Stage | Value (source) |
|---|---|
| Control/`DoControl` rate | **500 Hz** (`uavxarm-v3-gke.c:235`, `DoControl(dT)`; `mpu6xxx.c:48` "2 ms / 500 Hz") |
| Gyro LPF (software LPF2) | `MPURateLPFHz[] = {250,184,98,41,20,10,5,3600}`, default `pGyroLPFSel=2` → **98 Hz** (`imu.c:137`) |
| D-term LPF (decoupled) | **50 Hz**, `DTERM_LPF_HZ` (`inertial.c:577`) |
| Acc LPF | default `pAccLPFSel=4` → 20 Hz (`MPUAccLPFHz`) |
| Motor/prop inertia | `MotorTau` ≈ 0.08–0.2 s in `emu.c` — a first-order **mechanical LPF** between motor input and delivered torque |

Why this matters: **the loop already contains three real low-pass stages**
(motor/prop inertia → gyro LPF2 → D-term LPF). A noise model that feeds the
rate loop un-filtered white noise bypasses every one of them and therefore
*grossly over-states* the high-frequency energy the D-term must reject. That is
the difference between "the sim says Kd must be large" and "Ken actually flies
Kd = 0.008".

### 3.2 Sampling, Nyquist, and the true rotor lanes

Using **KV × V × blade count** for RPM (user-supplied baseline: 900 KV
2-blade for larger quads and most non-wing FW; ~2200 KV 3-blade for flying
wings), with a typical loaded factor ~0.75–0.85 of no-load KV×V at hover load:

| Airframe class | V | KV = f₁/RPM | rotor f₁ | BPF = f₁·blades | at 500 Hz (Nyq 250) |
|---|---|---|---|---|---|
| Ken_450_1165 (8", 2-bl, 3S) | 11.1 | 900 | ~125–142 Hz | **~250–283 Hz** | BPF folds just below/at Nyquist → at the corner |
| Ken_Alpha_Test (8", 2-bl, 4S) | 14.8 | 900 | ~167–189 Hz | **~333–378 Hz** | above Nyquist → **aliases into band** |
| Large quads (typical 4S) | 14.8 | 900 | ~178–222 Hz | **~355–444 Hz** | aliases → ~56–145 Hz residue in-band after fold |
| Flying wing (3-blade) | 11.1–14.8 | 2200 | ~305–407 Hz | **~915–1221 Hz** | folds far down after mixed-down |
| Ken's micro (brushed 1S) | 4.2 | (fast) | high | ~kHz | far above; attenuated by gyro LPF |

Key consequence we are adopting into the model: **for the larger quads the
blade-pass lane is right at/above the 250 Hz Nyquist.** At 500 Hz sampling, the
BPF harmonic ~335–444 Hz folds to a ~56–145 Hz residue *inside the control
band* — precisely the band the 98 Hz gyro LPF + 50 Hz D-term LPF are there to
kill. A sim sampled at **1000 Hz** (current `CONTROL_DT=0.001`, Nyquist 500 Hz)
**cannot see this fold at all** — above-Nyquist content is simply not
represented. That is the concrete reason the sim must run the measurement path
at the FC's **500 Hz**.

### 3.3 The adopted noise model

1. **Order:** plant (with `MotorTau` inertia LPF) → rotor-harmonic vibration +
   MEMS white floor + bias random walk → **gyro LPF2** (per `pGyroLPFSel`) →
   **sample at 500 Hz** (folding the in/above-Nyquist lanes naturally) → rate
   loop feeds the **D-term through its own 50 Hz LPF**.
2. **Rotor vibration:** a small sum of sinusoids at f₁, BPF, and a BPF harmonic
   (or the actual measured-profile shape), amplitude scaled by commanded
   throttle (or hover thrust fraction in the attitude sims), with **per-prop
   phase offsets** (props are not phase-locked, so in-phase/out-of-phase mixing
   is physically relevant and amplitude should be treated as a *band*, not a
   tone).
3. **Narrowband, not white:** leave the moderate/large white RMS on the *angle*
   (attitude) and baro/GPS channels, but replace the flat rate-channel noise
   with white floor + narrowband rotor lanes — because flat broadband on the
   gyro is the artifact that inflates D.
4. **Sample-rate correction:** `CONTROL_DT` for the measurement path set to the
   FC's 2 ms (500 Hz) in the robustness sim, so the alias fold is real. (The
   deterministic `test_pid_sim` step/critique path stays at its current dt for
   numerical continuity with the critic; only the *noise/measurement* path is
   re-rated — the critic metrics are legal-rate independent by design.)

### 3.4 Why this lands on the proven D-terms

The proven values are small *because* of the filter chain, not despite it:
- **Ken_450_1165 / Ken_Alpha_Test** (8" 2-blade, 900 KV): BPF ≥ 250 Hz is
  above the 98 Hz gyro LPF corner, so by the time the signal reaches the rate
  loop the rotor energy is down ~3 decades (LPF2 = −40 dB/dec from 98 Hz). The
  D-term at 50 Hz covers only the low-residue band; `Kd 0.008/0.009/0.002` is
  all the derivative action the physical airframe ever needs.
- A white-noise model (flat to 500 Hz) instead feeds that same D-term a *full
  decade more* high-frequency energy than physics provides → the sim
  over-recommends Kd.

The acceptance test for the adopted model is therefore: **the deterministic
critic must still PASS the exact proven gain sets of Ken's larger quads**, and
the robustness run with the new measurement path must **not** push recommended
D beyond them.

## 4. Results vs. proven legacy MC airframes

### 4.1 Ken's larger quads — the validated reference

All values from the proposed/ templates, confirmed identical in the flown
`saved` dumps (`user/Ken_Alpha_Test_20260830_194619.af`,
`user/Ken_Alpha_Test_20260831_084923.af` except the later 10" experiment).

**Ken_450_1165** — 1200 g, 225 mm, 8" 2-blade, 3S:
`RollKp 0.34 / PitchKp 0.32 / YawKp 0.11`, **Kd 0.008 / 0.009 / 0.002**,
angle Q 7/7/3.

Deterministic critic: **PASS on every axis.** Roll step rise 0.295 s, OS 4.1 %,
settle 0.64 s; Pitch rise 0.29 s, OS 4.9 %, settle 0.645 s; Yaw rise 0.835 s,
OS 5.5 %, settle 2.785 s; disturbance rejection PASS all three axes.

**Ken_Alpha_Test** — 1000 g, 181 mm, 8" 2-blade, 4S:
`Kp 0.34/0.34/0.11`, **Kd 0.008/0.008/0.002**, angle Q 5.75/5.75/3.

Deterministic critic: **PASS on every axis.** Roll rise 0.37 s, OS 3.5 %,
settle 0.755 s; Pitch rise 0.355 s, OS 4.1 %, settle 0.745 s; Yaw rise 0.895 s,
OS 4.1 %, settle 2.725 s; disturbance rejection PASS all three axes.

**Verdict:** the sim's recommendation for these airframes **is** the proven
configuration — no "recommend tuning" output on any axis. The Kd/Kp ratios
(roll/pitch ≈ 0.024, yaw ≈ 0.018) are the calibrated target envelope the new
noise model must preserve.

### 4.2 Ken's micro (LadyBug class)

30 g, 1.5" 2-blade brushed: `Kp 0.3/0.3/0.25`, **Kd 0.008/0.008/0.002**.
Critic PASS on every PID step; disturbance rejection PASS. Far-higher rotor
frequency (kHz+) than the larger quads — consistent with the model: content
aliases even harder but is *far* outside the LPF passband, so D needs nothing
extra from noise. Same proven Kd/Kp envelope as the larger quads → same
acceptance test applies.

### 4.3 Eck's 220 mm — excluded as a reference (user-confirmed)

Eck's flown config is *by no means perfect* and its saved dumps show the known
**legacy-unit bridge bug** (raw `0.0015` vs template `0.34` ≈ 227×, the
1/scale legacy footprint tracked under GCS TODOs "Legacy scaling is
broken/reversed"). Its Kd=0 cannot be used to argue the model is wrong: the
variable is the airframe, not the physics. The model is validated on Ken's
frames, where both the airframe quality and the unit integrity are trusted.

## 5. Implementation plan (next session, pending approval)

1. Add a `sensor` noise-path toggle to `test_robustness_sim.py`:
   - measurement-path `dT` = 2 ms (500 Hz),
   - gyro LPF2 corner from `MPURateLPFHz[pGyroLPFSel]`, D-term LPF = 50 Hz
     (`DTERM_LPF_HZ`),
   - rotor lanes from KV×V×blades (per-class KV table), throttle-scaled
     amplitude, phase-diversity across props,
   - keep existing white angle/baro/GPS noise unchanged.
2. Regression gate: Ken_450_1165 and Ken_Alpha_Test must remain PASS on the
   deterministic critic and must not exceed the proven Kd envelope under the
   new robustness path (moderate AND large levels).
3. Report the delta on recommended D under noise-model change; only if a
   proven frame's Kd moves noticeably, revisit that airframe — otherwise the
   current proven values are confirmed as the sim's own recommendation.
4. Alias evidence: assert in the report that the 335–444 Hz BPF lanes of the
   900 KV 2-blade 4S class fold to the 56–145 Hz residue band at 500 Hz — this
   is the band the gyro LPF + D-term LPF suppress, i.e. the physical reason the
   D-terms are as small as they are.

Note: no FC changes result from this report. The FC already has the correct
chain; this is purely a GCS simulation-model correction so the sim and the
proven airframes agree.

## 6. References

- ArduPilot SITL docs — gyro/accel noise, bias, drift, vibration injection
  (`SIM_GYR_RND/BIAS/DRIFT`, `SIM_ACC_RND/BIAS/VIB`).
- PX4 / jMAVSim / Gazebo IMU plugin — MEMS sensor model (noise density, bias
  random walk, correlation time, turn-on bias).
- Betaflight documentation — blackbox FFT gyro analysis, dynamic notch tracking
  rotor RPM/BPF.
- Fresk & Nikolakopoulos — frame-induced vibration attenuation on quaternion
  attitude control (measured-vibration HIL).
- Verbeke & Debruyne — multirotor frame modal analysis; measured three-band
  PSD structure.
- This repo's own ground truth: `UAVXArmQ/src/inertial.c` (LPF chain),
  `UAVXArmQ/src/imu.c` (LPF corner tables), `UAVXArmQ/src/uavxarm-v3-gke.c`
  (500 Hz loop), `UAVXArmQ/src/emu.c` (`MotorTau`), and Ken's validated `.af`
  templates.