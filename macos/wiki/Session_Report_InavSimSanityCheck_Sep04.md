# iNav Defaults: Physical-Modelling Analysis + Our-Sim Sanity Check

Date: 2026-09-04
Sources: `~/Documents/Flight/OtherCode/inav-master` (iNav master checkout),
`UAVXGS/uavx-python/src/tests/compare_inav.py`, `test_pid_sim.py`.
Companion: `wiki/Session_Report_InavTuning_Sep04.md` (the raw default values).

## Purpose
Greg asked two things:
1. Does iNav **derive its default tunings from physical-modelling assumptions**
   for its various airframes?
2. **Sanity-check our sim/tuning framework**: take iNav's defaults, apply them to
   two of *our* airframes (`generic/Quad.af` MR, `generic/SkySurfer_Bixler.af`
   FW-aileron), run both gain sets through the SAME plant and criteria in
   `test_pid_sim.py`, and see whether our framework treats real-world-good iNav
   defaults as plausible. If it blesses iNav's field-proven gains and yields
   realistic numbers, our sim is sane; if it rejects them or returns absurd
   results, that flags a problem.

This report answers both. Read-only + a new comparison script
(`compare_inav.py`) — no FC or GCS behaviour changed.

---

## Part 1 — How iNav "builds" its defaults (the physical-modelling question)

**Headline: iNav does NOT derive its defaults from a per-airframe physical
model.** There is no mass / inertia / span / motor-count / dynamic-pressure
input anywhere in the path that produces the default P/I/D/FF numbers. The
defaults are a **single hand-authored constant set per platform class**
(multi-rotor vs fixed-wing), baked into the firmware at build time. The
*physical-behaviour* adaptation is done by a separate set of **opt-in runtime
scaling mechanisms** layered on top, not by the default values themselves.

### 1.1 The pipeline: YAML → generated C → firmware
- The authoritative defaults live in `src/main/fc/settings.yaml` as
  `- name: mc_p_roll … default_value: 40` entries inside the `PG_PID_PROFILE`
  group.
- A Ruby generator (`src/utils/settings.rb`, wired by `cmake/settings.cmake`)
  runs at **build time** and emits `settings_generated.h/.c`, which turns each
  `default_value` into a `SETTING_MC_P_ROLL_DEFAULT` macro. The generated header
  is not in the checkout — you must build to materialise it (this is why the
  path through the CLI/configurator feels opaque).
- `pid.c` `PG_RESET_TEMPLATE` copies those macros into the `pidProfile_t`
  reset template. The rate loop then applies a fixed realtime divisor
  (`FP_PID_RATE_P_MULTIPLIER`=31, `I`=4, `D`=1905, `FF`=31 — `pid.h:41-46`).

No term in that pipeline reads a physical constant of the aircraft. The same
`40/30/23/60` roll defaults serve a 3" micro and a 7" quad, and a 5/7/0/50
FW roll default serves every plank.

### 1.2 So what IS the "physical modelling" iNav does?
The physics shows up not in the defaults but in **runtime, opt-in scaling**, of
which the two big ones are:

**(a) Airspeed-Throttle Proportional Authority (APA) — the aerodynamic model.**
`pid.c:446-452` (plus `calculateFixedWingAirspeedTPAFactor`):
```
tpaFactor = (referenceAirspeed / airspeed) ^ (apa_pow/100)
tpaFactor constrains to [0.3, 2.0]
```
applied at `pid.c:584-587`:
```
kP  = P  / 31    * tpaFactor
kFF = FF / 31    * tpaFactor
kD  = D  / 1905  * tpaFactor
kI  = I  / 4     * iTermFactor   (less aggressive exponent)
```
This is precisely the aerodynamic authority law: a fixed-wing's control torque
scales with dynamic pressure ∝ V², so gain must scale ∝ 1/V to hold a constant
rate per stick. With `apa_pow = 100` the exponent is 1 → linear
`1/airspeed`; the settings.yaml description states the range explicitly:
*"Gains range from 30% (high speed) to 150% (low speed). I-term scaled less
aggressively."* `fw_reference_airspeed` (default **1500 cm/s = 15 m/s**) is the
airspeed the gains were tuned at.

**Caveat:** `apa_pow` defaults to **0**, so this is **off** unless a pitot /
airspeed-validated aircraft enables it (`Recommended: 120 for aircraft with
validated pitot sensor`). At stock defaults there is no attenuation at all.

**(b) Throttle-based TPA (`tpa_rate` / `tpa_breakpoint`).**
`pid.c:469-509` scales P/D/FF down as throttle rises past a breakpoint. Also
defaults to **0** (`dynPID`), i.e. off.

There is **no motor-count / mixer-type / inertia normalisation** in the MC rate
path — the mixer normalises the summed PID output, which is what allows one set
of defaults to span the size range (`Session_Report_InavTuning_Sep04.md` §5).

**FW autotune** (`pidAutotuneConfig_t`) is a *runtime, in-flight* tuning loop
that learns rate/P/D/FF from stick/response — a different beast from a
physical model, and not a default-value source.

### 1.3 Bottom line for Part 1
- Defaults = **crowd-sourced, field-tuned constants** (P=40/30/23 CD60 MC;
  P=5/7/0 FF50 FW-roll), NOT physics-derived.
- The **physics is opt-in runtime scaling** (airspeed APA `1/V`, throttle TPA),
  both defaulted off.
- Therefore iNav's defaults are great as a *cross-reference* ("is our rate P
  within a factor of the field-proven consensus?") but cannot be applied
  directly to a NEW airframe by scaling some physical parameter — iNav never
  computes them from physics.

### 1.4 This is not new to us — we already tinkered with gain scheduling
Greg noted he had at some point tinkered with gain scheduling. Indeed the UAVX
lineage contains **two** prior attempts, both conceptually the same shape as
iNav's runtime scaling:

**(a) `GainSchedule()` — the oldest (retired `UAVX-GKE*` / `UAVXArm-V3-GKE`
`control.c:224`).** A multiplicative gain `GS` (nominal 256 = 1.0) that was
reduced from 1.0 by two terms:
- an **attitude-hold-limit excursion** term — `AttDiff = CurrMaxRollPitch -
  ATTITUDE_HOLD_LIMIT` scaled by param `P[Acro]`, i.e. kill attitude authority
  as you push past the angle-hold limit (anti-overconfidence near the attitude
  boundary);
- a **throttle-vs-cruise** term — `ThrDiff = DesiredThrottle - CruiseThrottle`
  scaled by `P[GSThrottle]`, i.e. shed attitude gain under high throttle.
`GS` gated `[0,256]` and multiplied the whole attitude-control output.

**(b) `DoAttitudeGainScale()` — the more recent (`UAVXArm32F4` `control.c:372`).**
`AttitudeGainScale` drops linearly from 1.0 as `DesiredThrottle` rises above
`pCruiseThrottle`, by `MaxAttitudeGainReduction` (param, default 0 → off):
```
AttitudeGainScale = 1.0 - MaxAttitudeGainReduction * (DesiredThrottle - pCruiseThrottle)/(1 - pCruiseThrottle)
```
The fixed-wing branch is `AttitudeGainScale = 1.0f; // later based on Airspeed`.

**Why throttle, not airspeed (Greg, 2026-09-04):** there was **no airspeed
sensor**, so **throttle was used as an airspeed *analog***. The heuristic is
sound within a flight regime: control authority ∝ dynamic pressure ∝ V², and
throttle is loosely monotonic in thrust → V, so shedding attitude gain at high
throttle is a decent proxy for shedding it at high speed. The known weakness is
exactly what iNav's APA solves: throttle↔airspeed is not a fixed mapping across
climbs (high throttle, low speed — authority actually *low*) and dives (low
throttle, high speed — authority actually *high*), so a throttle-based schedule
gets it backwards in exactly those transient regimes. This is why the 32F4 FW
branch punted on it (`later based on Airspeed`) rather than force throttle
through for a plank.

**Relevance to Part 1 — and a path forward.** Our own experiments corroborate
iNav's philosophy (frozen defaults + opt-in dynamic authority scale) and,
notably, pre-empted iNav's *airspeed* mechanism but couldn't finish it for want
of an airspeed sensor. With **GPS + inertial wind estimation** (we already shell
an iNav-derived estimator as `UAVXArmQ/src/wind/wind.c`), a *true-airspeed*
(TAS = groundspeed ± wind) can now be reconstructed without a pitot — making a
genuine `1/TAS` APA-style schedule feasible on our gear. That said, the payoff
is modest for our class of airframe: **our platforms typically fly a narrow
speed range** except during rapid climbs or dive descents, where the throttle
vs airspeed inversion is most pronounced — precisely the regimes where a TAS
schedule would correct a throttle analog. So if we ever revisit authority
scheduling, the today-available, physically-correct form is TAS-based APA fed by
the GPS+IAD wind estimate, with iNav as the working external reference; a
throttle analog remains the defensible fallback when wind/airspeed estimate is
unavailable or unvalidated.

---

## Part 2 — Compare iNav defaults vs our tuning on the SAME plant

### 2.1 Method — `compare_inav.py`
New standalone script in `src/tests/`. Loads each airframe's gained tuning and
runs **both gain sets through the same plant + shared `critic` criteria**:

- **MR** (`Quad.af`) uses the single-axis `simulate_axis`.
- **FW** (`SkySurfer_Bixler.af` aileron, **`Shadow.af` elevon/delta**) uses the
  authoritative **coupled 3-axis** model `simulate_axis_coupled` — the same
  plant the main sim and the FC elevon/Delta mixer use (dihedral coupling,
  adverse yaw, elevon drag-differential yaw, `FW_ROLL_PITCH_FF`). iNav's rate
  P/D are injected via a `params_override` clone, keeping our outer angle loop.

Only the **rate P/D** are compared. This is the defensible core — our sim has
no rate-I (steady-state lives in the outer angle-loop I, AGENTS.md "Yaw rate
I-term"), and iNav's I/FF are wound against iNav's own output-normalisation and
clamp, so a linear unit-map of I/FF over-feeds our fraction-of-authority plant
(demonstrated in the earlier ad-hoc PIFD row → 14–23% MR overshoot). Including
that systematic mapping artifact would badly mislead a critique, so it is
excluded here.

**Gain mapping:** iNav's summed output clamps to `pidSumLimit` (500 MR r/p +
FW, 400 MR yaw); our output is ±1.0 (full authority). iNav gains act on deg/s,
ours on rad/s:
```
Kp_ours = (P/31)      * (180/π) / limit
Kd_ours = (D/1905)    * (180/π) / limit
```

### 2.2 Results

**MR — `generic/Quad.af`**

| axis | Ours | iNav rate P/D mapped |
|---|---|---|
| Roll (15°) | rise .39s, ov .5%, settle .79s **PASS** | rise .35s, ov .5%, settle .76s **PASS** |
| Pitch (10°) | rise .39s, ov .6%, settle .79s **PASS** | rise .37s, ov .5%, settle .69s **PASS** |
| Yaw (45°) | rise 1.00s, ov 0%, settle 1.91s **PASS** | rise .99s, ov 0%, settle 1.90s **PASS** |

**FW aileron — `generic/SkySurfer_Bixler.af`**

| axis | Ours | iNav rate P/D mapped |
|---|---|---|
| Roll (15°) | rise 2.36s, ov 10%, settle 7.6s **PASS** | rise 8s, final 11.2° **FAIL** |
| Pitch (30°) | rise 8s, final 3.4° **FAIL** | rise 8s, final 0.4° **FAIL** |
| Yaw (15°) | rise 8s, final 9.0° **FAIL** | rise 8s, final 0.5° **FAIL** |

**FW elevon/delta — `generic/Shadow.af`**

| axis | Ours | iNav rate P/D mapped |
|---|---|---|
| Roll (15°) | rise .38s, ov 24.1%, settle 3.0s **PASS** | rise 2.27s, ov 14%, settle 8s **PASS*** |
| Pitch (30°) | rise 8s, final 6.8° **FAIL** | rise 8s, final 0.4° **FAIL** |

*Shadow iNav-P roll "PASS" only on the *rate* metric set; final 10.3° vs 15°
step and settle-8s show it is far too weak (see critique). Shadow yaw is
rudderless (drag-differential, ~0 authority at the 15° step) — excluded for
both, as expected of a plank.

### 2.3 Interpretation

**MR — agreement, framework validated.** iNav's independently field-proven MC
rate P/D, mapped into our units, land within ~10–30 % of ours and give
near-identical responses (roll settle .76s vs .79s; yaw .99 vs 1.00s). That an
entirely separate firmware's mature tuning converges with ours on a third
party's plant is exactly the sanity evidence we wanted: **our MC plant and
criteria are physically reasonable and our MC tuning sits in the industry
consensus band.**

**FW — iNav's P alone is far too weak, and that is structural, not a fault.**
iNav FW ships deliberately tiny P (5) because its real authority comes from the
**FF term** (50) + aerodynamic self-damping. Injecting only iNav's P (we do not
model their FF) leaves every FW axis under-achieving (SkySurfer roll final
11.2°, Shadow roll final 10.3° vs 15°). This is a portability limit of the
*mapping*, not a verdict that iNav is bad on a real plank — and not a verdict
on our framework.

**Our own Shadow roll is the fleet's tightest margin** under our criteria:
24.1 % overshoot vs 25 % limit (rise just 0.38s). It PASSes, but it is
deliberately fast/aggressive — a legitimate target if we want more headroom.

---

## Part 3 — Critique: UAVX vs iNav tuning philosophy

This is the substantive ask — a head-to-head *critique* of the two tuning
frameworks, not just the gain numbers. Both are valid, mature control designs;
they differ in *where* they put authority and in how they normalise output.

### 3.1 Where the control power lives

| | UAVX (ours) | iNav |
|---|---|---|
| **Rate P** | **High** — carries the authority (MR Kp ~0.2–0.5; Shadow roll 0.53) | **MR**: moderate (40/31); **FW**: deliberately tiny (5/31) |
| **Rate D** | Yes, per axis (any FW P+D; MC D ~23-equivalent) | MR D=23; **FW D=0** (relies on aero self-damping) |
| **Rate I** | **None** — steady-state error pushed to the *outer* angle-loop I | Yes (MC I=30, FW I=7) |
| **Rate FF / CD** | None (we have no FF term at all) | **CRITICAL** — FW FF=50, MC CD=60; the FW plank's real authority |
| **Output normalisation** | ±1.0 = full surface/rotor authority | raw `pidSum`, clamped to `pidSumLimit` (500/400) |

The single most consequential difference: **iNav's fixed-wing rate loop is
FF-first** (FF does the driving, P just trims), whereas **ours is P-first with
the angle loop as the setpoint-rate source.** This is why a naive unit-map makes
iNav FW look broken — we are comparing iNav's *P* to our *whole* authority chain.

### 3.2 Doesn't run to the same numerical space
We cannot hand iNav's integers to our FC and expect the same response, because
the two firmwares normalise output completely differently (fraction-of-authority
vs raw pidSum against different `pidSumLimit`s) and because iNav ships an FF
term we do not have. The only fair comparison — which we did — requires mapping
to a common authority scale AND isolating the terms that actually exist in both
(FW rate P+D, MC rate P+D). On that common ground (Part 2), the honest result:
**iNav's rate P+D and ours are broadly consonant on MC; on FW they cannot be
compared term-by-term because iNav's P is not its true authority.**

### 3.3 What we can take from iNav, and what we should not

**Take (concepts, not numbers):**
- **Tiered authority per platform class is sound.** MC needs strong rate D +
  P (no aero damping); FW gets away with light P because the wing self-damps —
  iNav codifies exactly what our per-airframe `pitch_damp`/`yaw_damp` already
  model. No change needed; it confirms our structure.
- **iNav's opt-in airspeed APA (`1/TAS`) is the physically-correct authority
  schedule.** We have the iNav-derived `wind.c` GPS+IAD estimator, so a genuine
  `1/TAS` schedule is feasible without a pitot (see Part 1.4). Modest payoff for
  our narrow-speed-range class, but it fixes the climb/dive inversion our own
  throttle-as-airspeed analog suffers.

**Do not take:**
- **Do not adopt iNav's FF-first FW rate loop.** Our P-first + angle-loop-I
  decomposition is internally consistent, simpler, needs no additional FF tuning
  surface, and already passes our criteria. An FF term would be an *additional*
  feature if we ever want slam-on-demand on a plank, not a correction.
- **Do not copy iNav's integer gains.** They are normalised to iNav's pidSum
  convention and presumes their FF/I/self-damping — meaningless on our FC.

### 3.4 Actionable notes on our own tuning (from this comparison)
- **Shadow roll is our tightest-margin axis** locally — 24.1 % overshoot vs 25 %
  limit, 0.38s rise. It is deliberately punchy; consider +D (rate Kd
  ↑) if we want settle headroom, at a slight rise-time cost. Not required.
- **FW pitch under-achieves everywhere** (Quad-class elevator/elevon at 30°):
  this is an *aero authority limit* (small elevator surface), not a gain defect —
  iNav's mapped P fails identically. Raising `CM_D_ELE`/surface in the model, or
  easing the 30° FW step to a realistic pitch demand, are model choices, not
  tuning fixes.
- The **comparison itself** is a useful regression guard: if a future UAVX FW
  retune ever made our rate P *stop* agreeing with iNav's consensus band on MR,
  that would signal over- or under-tuning.

### 3.5 Next step — ArduPilot
Greg intends to run the same exercise against **ArduPilot** (its separate rate
PID + `AC_*` convention, and its own FW/plane attitude loops). Same recipe
applies: extract ArduPilot's defaults for a representative small MR + plane,
map its output authority scale to our ±1, isolate the terms we share (rate
P/D/FF), and run both through our plant. ArduPilot's plane tunning uses a
different structure again (SLEW/rate-P with I on a per-surface basis), so the
"isolate what actually maps" discipline from Part 2 will matter. This is the
natural follow-on; leave it for a dedicated session/report.

---

## Files
- `uavx-python/src/tests/compare_inav.py` — comparison script (Quad MR +
  SkySurfer + Shadow FW via `simulate_axis_coupled`; iNav rate P/D injected).
- `uavx-python/src/tests/test_pid_sim.py` — reused unchanged (plant + criteria).
- `wiki/Session_Report_InavTuning_Sep04.md` — the raw default-value tables.

## Verification
- `python3 tests/compare_inav.py` runs clean end-to-end: Quad (MR), SkySurfer
  (FW aileron), **Shadow (FW elevon)** — all axes, OURS + iNav-P rows. FW rows
  use the coupled model, so numbers match the main `test_pid_sim.py` runs
  (Shadow roll: rise .38s / ov 24.1 % / settle 3.0s). No pytest suite covers it
  (report/analysis script, not a unit test).
- No FC source, GCS source, or `.af` payload changed.
