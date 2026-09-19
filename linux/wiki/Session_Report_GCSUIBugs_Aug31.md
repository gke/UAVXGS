# Session Report — GCS UI Bug Fixes (legacy write path, bargraph colour, RC indicator blink, rate-Kd decimals)

**Date:** 2026-08-31
**Component:** GCS `main_window.py` + `parameter_window.py` (`uavx-python/src/ui/`)
**Build/verify:** `python3 -m py_compile ui/main_window.py ui/parameter_window.py` ✓

Three user-reported GCS UI bugs, fixed in one session. Two were pure GCS-side
defects; the third was a cross-tag state-clobbering bug whose root cause could
only be pinned by cross-correlating the rawlog against the two telemetry tags
involved.

---

## Bug 1 — Legacy param scaling broken/reversed on the write path

### Symptom
In the parameter window's **Legacy** mode, reading a param back and re-saving
it (or relying on the initial write path at all) produced the wrong FC value.
The displayed legacy number and the raw float actually written to the FC did
not agree with the `legacy = raw / PARAM_SCALES[idx]` bridge.

### Root cause
`main_window.send_params_typed()` (`main_window.py:1974`) computed the write
multiplier with:

```python
mult = PARAM_DISPLAY_MULT.get(i, 1.0)
```

That is correct for the unified/normal view but **ignores the Legacy mode
entirely**. Every param-window read/write path already routes display↔raw
conversion through `parameter_window._display_mult(idx)`
(`parameter_window.py:875`), which selects `raw * PARAM_DISPLAY_MULT[idx]` in
normal mode and `1.0 / PARAM_SCALES[idx]` in legacy mode. The one path that
bypassed it was the bulk send — so in legacy mode the GCS blit the *unified*
raw value to the FC, not the value the spinbox was showing.

### Fix
`main_window.py:1974`:

```python
mult = self.param_window._display_mult(i)
```

Now the wire write is derived through the *same* `_display_mult` the spinbox
uses, so the round-trip is exact in both modes. `_write_param_direct`
(`main_window.py:2038`, trace-combo / param-125 path) takes already-raw floats
and was never affected.

### Alternatives rejected
- **Duplicate a legacy-check inside `send_params_typed`:** would fork the
  conversion logic in two places; `_display_mult` already encapsulates the
  mode switch and is the single source of truth.
- **Push conversion into the spinbox signal handlers:** the write path is
  shared by bulk/verification flows; fixing at the point of scaling is the
  minimal correct change.

---

## Bug 2 — Motor/servo bargraph percentage flips orange → black

### Symptom
In the Motor/Servo display, the percentage fills read as orange near the
range edges but near-black (unreadable) mid-range, and text contrast on the
fills was poor.

### Root cause
`parameter_window._update_motor_display()` styled the mid-band
(`10 <= pct <= 80`) with `bg = "#555"`. On the dark GCS theme that renders
as near-black — indistinguishable from an inactive/off channel. The label
stylesheet also carried no explicit text colour, so the text could blend into
either fill.

### Fix
Mid-band fill → `#27ae60` (green); the label stylesheet now sets
`color: white` explicitly. Edge-of-range values (`pct < 10 or pct >
80`) keep the orange `#f39c12`. The fill now has three clearly distinct
visual states (off / mid / edge).

---

## Bug 3 — RC channel indicator blinks (red background *is* correct)

### Symptom
The RC channel spinboxes (Param ▸ RC, `rc_channel_spins` in
`parameter_window.py`) revert to red border/background at a steady cadence —
a fast blink at telemetry rate. The red *state itself* is correct (a mapped
slot `>= DiscoveredRCChannels` is genuinely unusable); only the blinking is
wrong.

### Root cause (cross-tag clobber)
Two facts from the rawlog (`src/log.txt`) made it non-obvious:

- **FLIGHT frames (tag 13) carry `discovered_channels=0`.**
- **RC frames (tag 22) carry `discovered_channels=1`.**

`main_window.py` `case 13:` replaces `self.flight_data` **wholesale** with the
fresh parsed object (`main_window.py:2266`). `FlightData.discovered_channels`
defaults to `0` (`packet_parser.py:187`), and only `case 22:`
(`main_window.py:2349`, parser at `packet_parser.py:644` is fed from the RC
frame) ever sets it to the real value. The tag-13 replacement therefore
clobbered the RC-frame value back to 0 on every FLIGHT frame (~10 Hz).

`update_rc_display()` (`parameter_window.py:4284`) evaluates
`bad = discovered > 0 and slot >= discovered` at 50 ms via `rc_timer` plus on
every data update. The `bad` state flapped `1 → 0 → 1 → …` as FLIGHT (0) and
RC (1) frames interleaved — i.e. the *restyle guard* (`bool(bad) != old`,
`parameter_window.py:4295`) fired every cycle, exactly the observed blink.
The guard was working; the *data feeding it* was being wiped on a sibling tag.

The codebase already had the fix pattern in the same block: tag-13 preserves
`pwm` and `rc_channels` across the wholesale replacement
(`main_window.py:2252-2270`) precisely because those are set on the RC tag
and would otherwise be lost moment-to-moment. `discovered_channels` was
simply never added to the preserved set.

### Fix
Extend the same preservation block to `discovered_channels`
(`main_window.py` `case 13:`): save `old_discovered` before the replacement,
restore it if the incoming frame's value is falsy. RC-derived state now
survives FLIGHT-frame replacement exactly like `pwm`/`rc_channels`.

### Alternatives rejected
- **Debounce the restyle in `update_rc_display` (require N sustained frames):**
  masks the defect; the underlying flight-data state would still flap, and any
  other consumer of `discovered_channels` would see garbage. Fix the data
  source, not the paint.
- **GCS-side "last known non-zero" tracking in the parameter window:** same
  objection — papering over clobbering instead of fixing it at the one place
  that replaces the object.
- **Stop replacing `self.flight_data` wholesale on tag 13:** larger touch —
  the replacement is load-bearing for the field set that *does* come from the
  FLIGHT frame; the pre-existing preserve-pattern is the intended design.

### Why the red state was never "wrong" on the bench
`discovered_channels=1` means the bench decoder currently discovers only
channel slot 0, so any mapped function on a higher slot is *genuinely*
non-functional this session. The red flag is doing its job; the blink was the
bug.

---

## Bug 4 — Rate Kd spinboxes show 3 decimals, real gains display as 0.000

### Symptom
The Rate-Kd parameter spins (Roll/Pitch/Yaw D-gain, tags 11/27/90) only
offered **3 decimal places**, so fine D-gain values (e.g. the fleet yaw rate
D 0.0003375, or the 0.001125 unified default) displayed as **0.000** — a
nonzero gain rendered as zero.

### Root cause
`parameters.py` `PID_GAIN_TAGS` — the set that forces ≥4 decimals on gain
spinboxes via `parameter_window._min_decimals` — contained only the **P, I,
and I-limit** terms (comment even said "18 PID terms"). The three **D** tags
(11 Roll, 27 Pitch, 90 Yaw) were never included. Without the ≥4 floor, the
spinbox decimals fall out of `step = max(0.001, span/50)`; for the yaw Kd
default range (0.0, 0.05) → step 0.001 → `dec = 3`. Result: any stored value
< 0.0005 is shown as 0.000 and fine editing is impossible.

### Fix
`parameters.py:498` — added tags `11`, `27`, `90` to `PID_GAIN_TAGS`
(18 → 21 terms) and corrected the comment. Every spin-build/refresh path
already routes through `_min_decimals` (`add_spin` 897, refresh paths 939,
3709), so one addition fixes all four builder sites (parameter groups, PID
grid, legacy refresh, bulk-read refresh).

### Alternatives rejected
- **Narrow only the yaw step floor / drop `step=0.001` floor:** unnecessary —
  typing is already exact (raw float32 write, decimals affect display/step
  only) and the ≥4 decimal guarantee is the existing, documented design
  intent for gain terms. Keep the change mirrored to the same rule gains use.
- **Per-airframe decimals override:** adds a new mechanism when the existing
  `_min_decimals` gate was simply missing the D tags.

---

## Verification
- Both edited files: `python3 -m py_compile ui/main_window.py ui/parameter_window.py` → OK.
- Bug 3 evidence trace: `src/log.txt` per-record `discovered_channels=0`
  (FLIGHT) / `=1` (RC) at interleave rate — matches the flap frequency
  end-to-end.
- Bug 4: `parameters.py` compile ✓ and membership assert
  (`11/27/90 in PID_GAIN_TAGS`) ✓ — roll/pitch/yaw Rate-Kd now resolve ≥4
  decimals in `add_spin` (+0 → `0.0003375` renders `0.0003`, never `0.000`).
- On-air/bench confirmation of the blink being gone is a next-bench item
  (see Follow-ups).

## Follow-ups
- Bench (F4V3, wired UART): connect and confirm the RC channel spinboxes hold
  one stable red/black state per slot instead of blinking. Also sanity-check
  Bug 1 by switching the param window to Legacy, reading a tagged param (e.g.
  NavPosKp), saving, and re-reading.
- Bench: check the Rate-Kd rows now show 4 decimals and the roll/pitch/yaw
  D-gains render non-zero when a non-zero airframe is loaded (e.g.
  original/Rok_Quad yaw D 0.0003375 → 0.0003).
- Note: `PARAM_DISPLAY_MULT`/`PARAM_TYPES` import in `main_window.py:29` is
  now unused after the Bug-1 fix but retained deliberately to keep the diff
  minimal (`PARAM_TYPES` was already unused there).