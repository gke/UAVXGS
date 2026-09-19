# Session Report — Nav PosIntLim: integral limit decoupled from MaxVelocity — Aug 28

Build: `fc_build.py` OK (FLYINGRCF4WINGMINI, bin 228772). GCS `py_compile` OK.
Sim: `src/tests/test_nav_bank_analysis.py` (new).

## The alias
`Nav.MaxVelocity` has carried the legacy param name `NavPosIntLimit` (tag 40)
since the original codebase — present verbatim in the reference `UAVXArm32F4`
(`FLOAT(&Nav.MaxVelocity, …, "legacy"), // NavPosIntLimit`). The FC never uses a
`NavPosIntLimit` symbol. The name overpromises: its only actual effect on the
integral is an *indirect* one — in `NavPI_P()` the position-I was back-wound so
that `|P+I| ≤ NavMaxVel` (nav.c Windup block), making the effective I saturation
≈ `NavMaxVel − |Ppos|`. Near a waypoint (`Ppos ≈ 0`) that allows I to preload up
to a full 6 m/s of demanded speed — stale integral carried out of the leg, a
classic cascade windup contribution to WP capture.

## Decision: separate animals
MaxVelocity (m/s cruise-speed ceiling: RTH/Escape legs, windup cap, ETA numerator)
and the position-integrator saturation are distinct mechanisms and distinct
fault modes. Adopted: a **dedicated `Nav.PosIntLim`** (tag 62, native m/s),
"significantly less than MaxVelocity".

Alternatives rejected:
- Keep only the shared cap (status quo): lets I grow to 5-6 m/s worth of demand;
  real but un-tunable windup near WPs. Rejected on the observed overshoot issue.
- A derived limit (`PosIntLim = fraction × MaxVelocity`): no knob needed but
  couples the two again and gives no wind-trim authority at high cruise speeds.
  Rejected — an explicit param is trivially tunable in the GCS and .af.

## Default value — sim-justified
Extended the nav cascade sim (bank-angle analysis) with an I-clamp
(`I = clamp(I + Ki·e·dT, ±Ilim)`, mirroring the FC: state-clamped then P+I capped
at NavMaxVel). Sweep at gains PosKp0.15/PosKi0.012/VelKp0.2/MaxVel6:
- Step 15 m, no wind: `Ilim=1.0` identical to unbounded (3.53 m overshoot;
  the bias never reaches 1.0 during the transient); `Ilim=0.5` improves capture
  (3.53 → 2.22 m overshoot, final 0.19 → 0.12).
- Station-keep, 2 m/s wind: `Ilim=0.5` leaves a −3.37 m standing wind offset
  (P must carry what I cannot); `Ilim=1.0` keeps it near zero (−0.07 m);

Verdict: **default 1.0 m/s** vs MaxVelocity 5 m/s — "significantly less" (~1/5),
negligible wind standoff, and it bounds the transferable preload to 20% of the
speed cap. 0.5 documented as the tight option if capture overshoot still bites.

## Implementation
- `params.c` tag 62: `FLOAT(&Nav.PosIntLim, eClassGainNav, 1.0, 0, 15, 1.0f,
  "m/s")` — repurposed the free `Unused63` float slot (adjacent to `Nav.PosKi`
  tag 60). ParamTableCRC bumps → defaults reload preserving cal.
- `auto.h` `NavStruct`: `real32 PosIntLim;` (RAM-only, no config layout change).
- `nav.c` `NavPI_P()`: after integration, `Nav.C[a].PosIntE =
  Limit1(Nav.C[a].PosIntE, Nav.PosIntLim)` — state-level clamp, same idiom as
  `Alt.P.IntE`/`Alt.P.IntLim`. The existing Windup back-calc stays as the
  velocity-cap safeguard (`DesVel ≤ NavMaxVel` still enforced).
- GCS mirror: `parameters.py` (defs/default 1.0/mult 1.0/class eClassGainNav/
  PID_GAIN_TAGS/EXPLICIT_LIMITS removal/scale 1.0), `protocol_enums.py`
  `NAV_POS_INT_LIM = 62` (replacing the vestigial `TILT_THROTTLE_FF`, which bound
  no FC param), `parameter_window.py` tuner row "I-Limit (m/s)" 1.0 + slider
  range (0.2, 2.0); the old "Tilt Thr (%)" tuner row removed. Tag-40 defs
  description corrected to "Nav max velocity (m/s)".

## Open item surfaced (not changed)
AGENTS claims `LEGACY_TAGS`/`PARAM_SCALES` were removed with no legacy mode, but
the code still carries both plus the `_legacy_mode` toggle (off by default) and
`_refresh_legacy_display()`. Flagged to TODO; the new param works identically in
either display path (scale 1.0).