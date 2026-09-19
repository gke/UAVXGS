# UAVX Flight Test Plan — progressive bench → flight campaign (Spring 2026)

Date: 2026-08-31. Author: Prof. Greg Egan (+ assistant). Companion docs:
`GCS_Guide.md`, `Session_Report_InflightAutoTune_Feasibility_Aug29.md` (Trace
roadmap + Stage-1 capture), `wiki/Session_Report_TraceV1_Aug30.md` (viewer),
`wiki/AGENTS.md` Trace section. `wiki/docs/FW_SIMULATION_REPORT.md` /
`MC_TUNING_REPORT.md` for the per-frame sim-predicted response tables.

**Rendering:** the user turns this into PDF on host "Merlin" via
`wiki/docs/md2pdf.sh` (assistant never generates PDFs).

---

## 0. Purpose and ground rules

The strategy, stated by the user: **do as much as we can on the bench, then
work through the inflight tests progressively as weather permits** (coming
into Spring slowly). The feature backlog (Trace V2 layouts, critic C port,
CRSF "last-breaths" carrier, in-air tuning) is being **deferred** on purpose
until at least one real, trace-verified flight exists — no more features on an
unvalidated capture pipeline.

Also: **friends fly** ("all care and no responsibility") and send logs back for
analysis. This plan therefore has two tracks:

- **Track A — benchmark aircraft** (the `original/` fleet frames: Ecks_220mm,
  Ken_450_1165, Ken_Alpha_Test, Ken_LadyBug, Ming_DevEBox, Phoenix, Rok_Quad,
  S500_1137, WLToys_LadyBug — each has a sim-predicted response to compare
  against).
- **Track B — guest aircraft** (friend-owned builds). Same capture protocol,
  but responsibility for the airframe stays with the pilot; we supply the
  checklist, the analysis, and honest advice. Guest aircraft get validated
  only to the level the pilot is comfortable with.

**Non-negotiables (all flights, both tracks):**

1. Propellers clear, crew/observers behind the line, roles named. First
   battery of the day on every new airframe config is a **no-manoeuvre
   validation** (below).
2. Fresh, fully-charged flight batteries. Independent BEC for the flight
   controller where the ESC-integrated 5 V is marginal; NEVER power the FC
   from a servo rail with a moving airframe.
3. Failsafe armed and understood per track: RC failsafe presets (NavQual →
   AltHold+Angle, throttle idle), RTH configured if WP nav is enabled on that
   frame. Confirm the FS values land as expected on the bench (GCS readback)
   before the first flight.
4. The GCS raw log is **always-on** from Connect to Disconnect (per AGENTS:
   `YYYYMMDD_HHMMSS_<Airframe>.rawlog`). A flight without a `.rawlog` did not
   happen.
5. Trace captures (ch8) are for **measurement only** — the FC never writes
   tune changes in the air (write-on-disarm + pilot confirmation only; the
   in-air APPLY tier is not commissioned).
6. **Abort criteria at any moment**: `F.LQCri`/`LQCrit` (link), unexpected
   `ExcessLift` beeper outside an intended climb, oscillation you did not
   command, rapid cell sag, smoke/burn smell, props-stopping in a way you did
   not command. Land immediately; the GCS kept the log.

---

## 1. Benchmark gates — everything that must pass BEFORE any motor turns

Each gate that fails sends us back to the bench. No gate is skipped.

| G | Gate | Pass criterion | Where captured |
|---|------|----------------|----------------|
| G1 | Firmware build | `fc_build.py` all 6 targets clean; `.bin` flashed to the test FC; revision label confirms the flashed build on connect | FC + GCS |
| G2 | Param .af round-trip | Load the frame `.af`, Write, reconnect (tag 72 apply) → GCS readback matches `PARAM_DEFAULTS`; `AirframeName` persists (Flash: label + tag-63) | GCS Params |
| G3 | RC mapping + failsafe | Ch6 NavQual/CBW mapping, ch8 → Trace; stick centre values; FS values confirmed present (SBus fail flag or timeout injection) | RC tab readback |
| G4 | Trace capture — anechoic bench | ch8 while State != eInFlight **must not** open a capture; simulated in-flight settle-step path exercised if the emulator allows; on-disarm `TraceCommit` writes CAPTURE sector only when armed | FC + GCS Dump |
| G5 | Trace dump + viewer | A committed snapshot (or a synthetic `.bin`) opens in the viewer; strips render on host; parse_trace round-trip clean | GCS Dump / File → Open Trace Dump |
| G6 | D8 link / cells / vario | (if softserial D8 used) all cells arrive in the 1 s burst; vario scale plausible; no FIFO overrun warning; ~25% softserial utilisation | TX + rawlog |
| G7 | USB-link resilience | Pull/re-plug USB while connected → `${telem}` line + reconnect ≤ 5 s; one rawlog per session, named with airframe | GCS |
| G8 | AltHold/level on bench | AltHold engages only when configured and armed; no spur on arm; ExLift lamp clear | GCS flag bytes |

**Depends on weather**: the G4/G5 bench item can be done after a rain-out;
G1–G3, G6–G8 are indoor/porch jobs. First Spring flyable days should be spent
on the **two easy flights**, not the diagnostic ones.

---

## 2. Progressive flight sequence (Track A — benchmark frames)

Each flight N is a *level*; you do N+1 only after N passed fully. Every flight
has: preconditions → procedure → capture → pass criteria → record line.

### Flight 1 — "No-manoeuvre validation" (5–7 min)

**Preconditions:** G1–G3 pass. Wind < 12 km/h, clear field, no other model in
the box. All axes at default. GPS position+fix (if nav enabled). Arm + disarm
repeatedly on the bench until the airframe behaviour is boring.

**Procedure:** Take off to a low hover (MR) / light hand-launch & climb
(FW), 20–30 m. **Fly a gentle racetrack only.** No rate steps, no AltHold,
no nav. Land. Do not arm again.

**Capture:** rawlog (whole flight). GCS screenshot with the tag-13/14
numbers at end.

**Pass criteria:**
- Link held for the whole battery (=3 min minimum), no `LQCrit`, no USB
  reconnects on the GCS side.
- Temperatures sane (ESC/motor hand-<45 °C; SnB reasonable for the craft).
- No IWDG reset / unexpected FC behaviour; beeper only where opinion has a
  reason.
- rawlog one file, named `…_<Airframe>.rawlog`, replays clean (Ctrl+R) and
  ends on a disconnect.
- (MR) rotor tips visibly stable; no persistent oscillation.

**Record line:** `F1 <date> <craft> <s/n> PASS/FAIL <notes>` in the log table
(§5).

### Flight 2 — Altitude hold flight (5–7 min)

**Preconditions:** F1 PASS. AltHold configured on this frame; pot/sw mapping
per arming rules (`rc.c`: switch arming → AltHold always on; Tx arming →
AltHold > 40 %).

**Procedure:** Climb to ~30 m. Engage AltHold. Cruise at constant throttle in
hold; then give a definite stick step (climb) and release — watch the ROC
return to < 0.5 m/s over ~2 s and the altimeter stop. One descent step the
same way. Fly home, land.

**Capture:** rawlog; if this is the very first AltHold flight, also bench-note
`AltHoldThrComp`/`DesiredThrottle` convergence from the tag-13 stream (the
`TrackCruiseThrottle` 2 %/s feed-forward tracking is visible there).

**Pass criteria:**
- Hold holds: altitude within a few metres at constant power; no porpoise.
- Climb/descent steps return to hold cleanly (no 2 s + overshoot, no
  oscillation).
- `ExcessLift` never fires during a normal cruise (only during an intended
  climb on soaring-capable frames, and never by accident).
- No wind-up stall: after a long forced climb the throttle returns to the
  previous hold trim without a dip.

**Record line:** `F2 <date> <craft> PASS/FAIL: hold<±m> climb/sink settle<Xs>`

### Flight 3 — Trace rate probes (measurement flight)

**Preconditions:** F1/F2 PASS. G4/G5 PASS on the bench. Test axis = Roll (v1
probe). One test per ch8 edge; do not re-arm during a controlled manoeuvre.

**Procedure:** Steady, level, quiet air. Arm ch8 (OFF→ON) at altitude; FC
settles (rates < 0.10 rad/s ~100 ms), opens capture, runs 100 ms pre-roll, then
steps `0.25 × R.Max` into the Roll rate setpoint for the rest of the window;
release ch8 after ~1 s. Repeat 2–3 times in separate sustained-but-settled
windows. Land. Dump each capture to the GCS viewer (Dump button → viewer).

**Capture:** rawlog (whole flight) + each trace dump saved. This is the FIRST
validated airborne response data ("the foundation of the whole feature stack").

**Pass criteria:**
- Capture opens only on the ch8 edge in flight; settle gate holds; the step
  appears in `Desired` and `Rate` responds.
- **Compare the measured rise/settle/overshoot/zero-crossings against the
  sim-predicted values for that frame** (`wiki/docs/PID_Simulation_Construction_Report.md`;
  the `critic` code is the same on both sides). Within ~±30 % and same FAIL set
  → **the sim plant is validated, the whole tuning story opens**.
- If the measured response is cleanly different (no oscillation, different
  time constants), that is a *plant-model* finding to record, not a tuning
  change to make in the air.

**Record line:** `F3 <date> <craft> rise<Xs> ov<X%> settle<Xs> xings<n> vs sim
<Δ%> VALID/DIFF`

### Flight 4 — Navigation (WP + RTH) flight (only if the frame is nav-capable)

**Preconditions:** F1–F3 PASS; failsafe RTH confirmed on bench; WP realisation
under wind/battery considered (AGENTS TODO — do the ground feasibility check
for the day's wind: achievable groundspeed vs. distance + Loiter/RTH reserve
BEFORE the leg is sent).

**Procedure:** Load a short out-and-back (2 WPs, ≤200 m, well inside field).
Launch, wait for GPS, enable nav. Let it fly the leg; verify turn behaviour
and arrival; command RTH from reasonably far; verify home descent and landing
gate. Abort-fly manually at least once mid-nav (mode switch) and verify clean
handover.

**Capture:** rawlog (whole flight).

**Pass criteria:** WP reached within a sensible radius; RTH returns to home
and holds; manual takeover is immediate and clean; no ballooning or big
cross-track in light wind; battery residual big enough to have loitered 2 min
more (energy-reserve evidence).

**Record line:** `F4 <date> <craft> WPΔ<m> RTH<ok/not> takeover<ok>`

### Flight 5 — Dive/recovery + VRS protections (commissioned frames only)

**Preconditions:** F1–F4 PASS for the airframe; `DiveRecoverAlt` (tag 116),
`VRSROC` (tag 103), `MaxDescentRateDmpS`/`DescentDelayS` **commissioned and
tuned together** (AGENTS TODO — never individually). No spectators inside the
recovery box.

**Procedure:** From altitude establish a deliberate descent (power low, nose
down on FW / descending on MR); verify the VRS shed happens BEFORE the
pull-out threshold; on FW verify the dive-recovery pull-out at
`DiveRecoverAlt`. Recover, climb, repeat once.

**Capture:** rawlog (whole flight); a trace capture during the dive if you want
the rate history of the protect itself.

**Pass criteria:** Protection engages before any threshold breach; recovery is
predictable and never degraded a subsequent manual command; no nuisance
trigger in normal flight.

**Record line:** `F5 <date> <craft> VRS_t<Xs before> diveR<ok/no>`

---

## 3. Track B — guest aircraft ("all care and no responsibility")

Friends fly **their own** aircraft. We provide the protocol, they fly to it,
they send logs. Responsibility for the model rests with the pilot.

**What a guest record needs:**

- The same `.af` load-saved and sent, so we can reproduce param state.
- A rawlog: same GCS connect→disconnect. If the guest lacks the GCS, a USB
  Dump of the trace is preferred but not required — the rawlog is the
  baseline.
- Flight notes: airframe mass/battery, wind, what was flown (level/alt/mode),
  anything odd.

**Guest progression:** recommend they fly the equivalent of F1 and F2 only on
their own aircraft (validation + AltHold), and send logs + trace dumps. Do not
send guests into F3 rate-probe territory on their first visit — probing was
designed for our benchmark frames; a guest frame's step response is our
analysis job, not their flying job.

**Analysis we return:** a `critic` comparison of their response against the
closest generic frame (`Quad`, `Oct`, `Spoileron`, `SkySurfer_Bixler`, …), a
PASS/FAIL opinion, and (only on request and only write-on-disarm) a suggested
param change. We do not flash friends' aircraft remotely; any `.af` change is
theirs to accept.

---

## 4. Log & data custody

| Artifact | Produced by | Goes where |
|----------|-------------|-----------|
| `.rawlog` (always-on) | GCS, per session | `~/UAVX/YYYYMMDD_HHMMSS_<Airframe>.rawlog` (folder via File → Set Log Folder); copy to `logs/<date>/…` on the analysis machine |
| Trace dumps | GCS Dump → viewer / File → Open Trace Dump | save per capture as `trace_<date>_<craft>_<axis>.bin` alongside the rawlog |
| `.af` | GCS Params Save | `logs/<date>/…` with the rawlog |
| Sim prediction | `tests/test_pid_sim.py` | compare on analysis (not per-flight) |
| Guest logs | sent by friends | dropbox/`logs/<guest>/…`; keep original filename |

**Naming convention (strict):** `YYYYMMDD_HHMMSS_<Airframe>.rawlog` (GCS
default when the persisted airframe name is present). Guest files keep their
GCS-generated name; we never rename the bytes, only the folder.

**Analysis workflow after any flight:**
1. Replay the rawlog (Ctrl+R) — first sanity: flight re-renders exactly as
   live.
2. Dump/load the trace capture(s), eyeball the strip.
3. Run the `critic` metrics + compare vs that frame's sim prediction (this is
   where measured-vs-sim goes into the running report).
4. Record the flight line in §5 and carry any finding into
   `wiki/Session_Report_<topic>.md`.

---

## 5. Flight log table (fill per flight)

| F | date | craft | sn | wind | battery | duration | mode | result | finding/link |
|---|------|-------|----|------|---------|----------|------|--------|--------------|
| | | | | | | | | | |

---

## 6. Weather and tempo rules

- **Restart of campaign after a gap > 2 wk:** restart at F1 for the
  airframe, not where you left off.
- Wind gates per flight: F1/F2 ≤ 12 km/h; F3 (probes) ≤ 8 km/h and no thermal
  bursts; F4 ≤ 12 km/h with the WP-realisation check; F5 ok to 15 km/h but
  with altitude margin.
- A marginal day is a **bench day**: G4/G5, viewer checks, sim/critic
  re-runs. Do not spend a half-open day on an F5.

---

## 7. Open items that feed the plan (not blocking F1)

- Viewer + critic integration (measure airborne step against the sim on
  open) — high leverage, bench-buildable before the first trace flight (F3).
- Trace V2 (multiple axes, extended rows), critic C port, CRSF last-breaths —
  **deferred by design** until F3 produces a validated capture.
- WP-realisation feasibility check (AGENTS TODO) — required only before F4.
```