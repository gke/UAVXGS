# Liu, Egan & Santoso (2015) — SWAT review

**Date:** 2026-09-08 · **Author:** Greg + assistant
**Type:** Literature review / external cross-reference (no FC or GCS code
changed — report-only, like the iNav/ArduPilot sanity-check entries).

---

## 1. Reference

Liu, M., Egan, G. K., & Santoso, F. (2015). *Modeling, Autopilot Design, and
Field Tuning of a UAV with minimum control surfaces.* IEEE Transactions on
**Control Systems Technology**, 23(6), 2353–2360. Article 7050322.
DOI: 10.1109/TCST.2015.2398316 · IEEE Xplore: document/7050322.
ISSN 1063-6536 (matches the citation); note the journal is TCST
("Control **Systems** Technology"), not "Control Technology".

## 2. What the paper does (summary)

Full engineering pipeline for a small fixed-wing UAV with **only two elevon
control surfaces** (no separate rudder/elevator — an underactuated MIMO
system: 2 inputs → 3 controlled outputs).

1. **Aerodynamic analysis** → deduce the MIMO underactuated linear model
   structure.
2. **System identification from flight data** — human-pilot test flight,
   then fit a two-input/three-output linear model (transfer functions in the
   airspeed, heading, and altitude loops).
3. **Root-locus design** of **five PID controllers in three loops**, tuned
   offline.
4. **Field tuning** — implement, fly, refine.

**Thesis:** with proper precautions, classical linear control (PID + root
locus + flight-data system ID) is sufficient for a problem often attacked
with nonlinear control algorithms — justified by limited onboard compute and
telemetry bandwidth on model-scale autopilots.

## 3. SWAT

### 3.1 Strengths
- **Workflow is sound and transferable:** system-ID-from-flight → offline
  linear design → field re-tuning. Produces explainable, auditable
  controllers (you can reason about a root-locus PID; you cannot always
  reason about a learned policy).
- **Honest engineering framing:** "limited on-board computing and
  communication bandwidth" is a real constraint on this class of airframe
  and genuinely motivates controller simplicity — not mere conservatism.
- **Independently validates our core philosophy** (see Mag/Inav notes in
  AGENTS): transparent classical controllers, offline/sim design first, then
  measured field tuning with human-traceable gains.

### 3.2 Weaknesses
- **Linearisation is only valid near its trim operating point.** The
  root-locus/PID result inherits every classic limit: high bank angle, near
  stall, turbulence, large airspeed excursions. The paper's "with proper
  precautions" is quietly carrying a lot of payload.
- **System-ID quality is the whole game.** A single human-flown test flight
  fitting a low-order linear model is sensitive to excitation content,
  unmeasured states, and wind disturbance. Without persistence-of-excitation
  control and wind handling the identified TFs can be optimistic.
- **Manual tuning path:** five gains / three loops by root locus then field
  trim does not scale to a fleet or rapid iteration.

### 3.3 Applicability to UAVX (what transfers)
- The **pipeline order** (model → identify → design offline → field trim)
  is exactly our own sim-first + Trace/critic-metrics + field-tune chain.
  It is a published, peer-reviewed precedent for keeping that workflow.
- The **elevon-only airframe** maps directly onto our elevon FW class
  (Shadow) — same control-surface architecture, same underactuation.

### 3.4 Transferability limit (the key lesson for us)
- This paper **stays entirely within the linear envelope**. Its methods would
  NOT have caught the inverted-yaw accelerant (`control.c:598` gate), the
  high-tilt coupled pitch/roll limit-cycle "initiator", or the emu
  accel-estimate loop — all of which are linear-design blind spots that only
  showed up because our quaternion loop + emulator push the analysis past
  small angles.
- **Verdict direction:** our deliberate divergence (quaternion attitude
  loop, singularity-free `AttitudeCosine()` inverted gate, condition-quat
  anti-windup, emulator root-cause process) is the *correct* extension, not
  scope creep. Liu/Egan/Santoso validates the classical core; our flight
  regime demands the nonlinear machinery they explicitly declined.

## 4. Cross-reference
- Same author lineage (Greg K. Egan) appears in the iNav wind-estimator
  lineage notes (`UAVXArmQ/src/wind/wind.c`, `inertial.c:135` Ihlein/AQ
  attribution); the paper is the elevon/FW autopilot-design companion to
  that MC/attitude work.
- See also the iNav (`Session_Report_InavTuning_Sep04.md`,
  `Session_Report_InavSimSanityCheck_Sep04.md`) and ArduPilot
  (`Session_Report_ArduPilotSanityCheck_Sep04.md`) cross-references for the
  two *runtime* autopilot comparisons; this is the literature/academic
  member of the set.

## 5. Companion paper — Santoso, Liu & Egan (2015), JIRS μ-synthesis

### 5.1 Reference
Santoso, F., Liu, M., & Egan, G. K. (2015). *Robust μ-synthesis Loop Shaping
for Altitude Flight Dynamics of a Flying-Wing Airframe.* Journal of Intelligent
and Robotic Systems: theory and applications, 79(2), 259–273.
DOI: 10.1007/s10846-014-0059-0. Springer.
**Publication year note (Greg's citation says 2014):** accepted 2014-03-24,
published online 2014-06-24, but the issue record is **Vol 79 No 2, Aug 2015**
(CSU + Springer both date it 2015). Cite as 2015 with "online 2014".

### 5.2 Summary
Same author trio (Monash Aerobotics), companion paper at the **opposite end of
the design spectrum** from the TCST PID paper. Designs a **centralised
flight-by-wire autopilot via μ-synthesis** for the **longitudinal/altitude**
dynamics of the same elevon-only flying-wing class (P15035 series). Plant =
**trimmed linear longitudinal model, constant throttle, identified
experimentally**. Rationale for μ-synthesis: (1) minimise the effect of
modelling uncertainty by maximising tolerated uncertainty within bandwidth
(minimise the structured singular value μ of the robust-performance block);
(2) keep the **whole MIMO model intact — no per-loop partitioning**, claimed as
a tuning-flexibility benefit. Compared against an **H∞ mixed-sensitivity**
autopilot; reported better time- and frequency-domain performance — quick
settling **without overshoot** and a better robust stability margin.

### 5.3 SWAT

**Strengths**
- Rigorous robustness machinery: explicitly models structured uncertainty and
  certifies stability **with** a performance margin, something root-locus/PID
  can never promise.
- Centralised MIMO design avoids the loop-pairing guesswork of per-axis PID.
- Honest comparative study (vs H∞) — time AND frequency domain, not just a
  single demo trace.

**Weaknesses**
- **Linear trimmed + constant throttle**: the whole analysis lives inside a
  small linear envelope — same blind spot as the TCST paper (no high tilt, no
  inversion, no dynamic throttle coupling). Its "robust" certificate is only
  as good as the *assumed* uncertainty model, which on such systems is usually
  guessed/fitted, not measured.
- **Controller order blow-up**: μ-synthesis (D–K iteration) yields
  high-order centralised controllers — the classic reason it struggles on
  model-scale autopilots with limited onboard compute (the very constraint the
  TCST paper used to justify PID).
- **Opacity**: a synthesized centralised controller is hard to reason about on
  the bench or trim in the field — the opposite property of our per-gain
  naming/GCS scheme.

**Applicability/transferability to UAVX**
- As an **external benchmark**, it's the counterpoint that validates our
  deliberate choice: on our airframe class (100 MHz F4, elevon/FW, field-tuned
  fleet) a centralized μ-synthesis controller would be high-order, opaque, and
  hostage to an uncertainty model we'd have to fabricate — while giving a
  *linear-envelope* certificate our aircraft then flies through (→ inverted,
  tumble, VRS). Our emulator + Monte-Carlo/nonlinear verification already
  provides a **stronger** robustness argument for the regimes we actually fly,
  at the price of no formal certificate.
- If we ever wanted a formal robust-stability cross-check of the quad/rate
  margins, μ-synthesis on the *emulator-derived* plant would be the rigorous
  tool — but that is a research-grade exercise, not a fleet-feature.

**Bottom line:** the pair of papers is a nice two-sided literature marker —
same airframe, same authors: classical PID (pragmatic, field-tuned, linear) vs
centralised robust synthesis (formal, batch-designed, still linear). UAVX sits
deliberately in between-with: classical controller structure + **nonlinear
quaternion machinery** + emulator-based root-cause — taking the pragmatics and
field-traceability of the former and the robustness *thinking* (structured
uncertainty, margins) of the latter, while rejecting both papers' linear-only
envelope.

### 5.4 Cross-files
- Same wiki report file covers the TCST companion paper (§1–4). See also the
  iNav (`Session_Report_InavTuning_Sep04.md`,
  `Session_Report_InavSimSanityCheck_Sep04.md`) and ArduPilot
  (`Session_Report_ArduPilotSanityCheck_Sep04.md`) runtime autopilot
  cross-references.

## 6. Files / source
- Reviews written from the CSU research-output records + IEEE/Springer
  abstracts (both full texts paywalled; ResearchGate preprints 403-blocked
  from this sandbox — fetch on host for a deeper citation-by-citation read).

## 7. Verification
- Literature review only — **no FC source, GCS source, or `.af` payload
  changed**, so no FC build / GCS `py_compile` applies (same standing as the
  iNav/ArduPilot report-only entries).