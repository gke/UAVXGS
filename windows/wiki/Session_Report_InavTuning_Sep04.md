# iNav Default PID Tunings — Multi-rotor vs Aileron (Fixed-Wing)

Date: 2026-09-04
Source: `~/Documents/Flight/OtherCode/inav-master` (iNav master checkout)
Module: GCS/FC tuning reference — comparison of the iNav MC and FW bank banks

## Purpose
Greg asked to "tunnel into iNav" and extract the **default PID tunings** for
our small quads, given that the path through the configurator to the PID loops
is "turgid". We have the iNav source on the host at
`~/Documents/Flight/OtherCode/inav-master`. Asked for the aileron (fixed-wing)
model as well. This is a read/reference report — no code changed.

## The process — how the defaults turned out to be "hidden"

### 1. The rate gains in `pid.c` are macros, not literals
`src/main/flight/pid.c` `PG_RESET_TEMPLATE(pidProfile_t, ...)` initialises the
bank with `SETTING_MC_P_ROLL_DEFAULT`, `SETTING_MC_I_PITCH_DEFAULT`, etc.
A repo-wide grep for those macro names finds **only `pid.c`** — the values are
not `#define`d anywhere in tracked source.

### 2. They are generated at build time
The macros come from a **generated header** `settings_generated.h` (included by
`src/main/fc/settings.h:9`), produced by a Ruby generator
(`src/utils/settings.rb`) driven by `CMake` (`cmake/settings.cmake`). The
generated header is not in the checkout — you must run a build to materialise
it. This is why the configurator/CLI path feels impenetrable.

### 3. The *values* live in a YAML settings table
The generator reads `src/main/fc/settings.yaml`. The authoritative defaults are
in the `PG_PID_PROFILE` block there (each `- name: mc_*` / `fw_*` entry carries
`default_value:` and `field:` mapping). That is the single source of truth.

### 4. Per-target overrides — none for our small quads
Checked the `config.c` of our likely small-quad targets
(`SPEEDYBEEF405MINI`, `MATEKF411`, `MATEKF411SE`, `BETAFPVF411`): **no PID-bank
override**. They all inherit the base `settings.yaml` defaults. So the defaults
below apply verbatim to our small quads.

### 5. Realtime scaling — the "mess"
In `pid.c` the integer defaults are divided by fixed multipliers at runtime:
- `kP = P / 31.0`
- `kI = I / 4.0`
- `kD = D / 1905.0`
- `kCD (MC roll/pitch FF) = FF / 7270.0 / looptime`; `kFF = FF / 31.0`
- Level `P`: `/ 6.56`

**Crucially, there is NO motor-count or mixer-type scaling in the MC rate
path** — the mixer normalises the summed output, so the same defaults serve a
3" micro and a 7" quad. The integer → real `kP/kI` conversion (`/31`, `/4`) is
the bridge if we want to compare iNav against our `A[axis].R.Kp/Ki`.

## MC (multi-rotor) defaults — the "small quad" answer

**Rate loop** (`mc_p/i/d/cd_*`):

| axis | P | I | D | CD(FF) |
|---|---|---|---|---|
| ROLL  | 40 | 30 | 23 | 60 |
| PITCH | 40 | 30 | 23 | 60 |
| YAW   | 85 | 45 |  0 | 60 |

**Self-level / attitude** (`mc_*_level`): P=**20** (1/sec), I=**15** (LPF
cutoff, 0=off), D=**75** (Horizon transition).

**Nav (position/velocity) MC loop**:
- `nav_mc_pos_xy_p` = 65, `nav_mc_vel_xy_p/i/d/ff` = 40/15/100/40
- `nav_mc_pos_z_p` = 50, `nav_mc_vel_z_p/i/d` = 100/50/10
- `nav_mc_heading_p` = 60

**Protections**: `iterm_windup` 50%, `pid_iterm_limit_percent` 33%,
`rate_accel_limit_roll_pitch` 0 (off), `rate_accel_limit_yaw` 10000 °/s²,
`heading_hold_rate_limit` 90 °/s.

## FW (fixed-wing / aileron) defaults

**Rate loop** (`fw_p/i/d/ff_*`):

| axis | P | I | D | FF |
|---|---|---|---|---|
| ROLL  |  5 |  7 | 0 | 50 |
| PITCH |  5 |  7 | 0 | 50 |
| YAW   |  6 | 10 | 0 | 60 |

**Self-level** (`fw_*_level`): P=**20**, I=5, D=70 (Horizon).

**Nav FW**: `nav_fw_pos_z_p/i/d/ff` = 30/5/10/30, `nav_fw_pos_xy_p/i/d` =
20/10/10, `nav_fw_pos_hdg_p/i/d` = 40/20/10, `nav_fw_heading_p` = 30.

## Side-by-side: MC vs Aileron (roll rate terms, raw integer)

| gain | MC ROLL | FW ROLL (aileron) | notes |
|---|---|---|---|
| P | 40 | 5 | MC ~8× stiffer rate P |
| I | 30 | 7 | MC ~4× |
| D | 23 | 0 | MC takes rate D; FW uses none (relies on aero damping) |
| FF/CD | 60 | 50 (FF) | MC uses CD (setpoint-rate FF), FW uses kFF (setpoint FF) |

Same realtime multiplier set applies to both banks (PID branch), so the raw
ratios above are meaningful: a multi-rotor runs far higher rate P and a D-term
because it has no aerodynamic self-damping, whereas a fixed-wing gets natural
roll damping from its fins/wings and needs only a light P-plus-FF.

## Sources / references
- `src/main/fc/settings.yaml` — authoritative defaults (`PG_PID_PROFILE`).
- `src/main/flight/pid.c` — reset template + realtime scaling.
- `src/main/flight/pid.h:41-46` — `FP_PID_*_MULTIPLIER` constants.
- `cmake/settings.cmake`, `src/utils/settings.rb` — the generator plumbing.
- Per-target `config.c` (SPEEDYBEEF405MINI et al.) — confirm no overrides.

## Next steps (offer)
- If we want, map iNav's real `kP/kI/kD` (after the `/31`,`/4`,`/1905` scaling)
  onto our `A[axis].R.*` gain fields for a concrete per-quad starting point, and
  note divergences from our critic-validated `original/` airframes.
