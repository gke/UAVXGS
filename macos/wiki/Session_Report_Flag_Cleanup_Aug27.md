# GCS Flag Cleanup & Link Stats — Session Report (27 Aug 2026)

## Summary

Cleaned up the GCS flag display system: removed redundant FC-sent boolean flags,
replaced them with GCS-computed pseudo flags derived from CRSF link stats (tag 69),
added three-state colouring (green/orange/red) for link quality indicators, and
updated the FC build script to build all commissioned targets.

## Changes

### 1. Flag Defs (main_window.py `FLAG_BIT_DEFS`)

| Bit | Old | New | Rationale |
|-----|-----|-----|-----------|
| 50 | `S-RC` (Have Serial RC) | **`Loss`** (RC Signal Lost) | Redundant — just says "serial Rx connected"; replaced with GCS-computed signal loss |
| 53 | `LQWarn` | **`LQ`** | Simplified label; now 3-state coloured (green/orange/red) |
| 58 | `LQCrit` | **`FS`** (Failsafe) | Removed redundant LQCrit (LQ already goes red < 50); replaced with failsafe pseudo flag |
| 75 | `RSSIWarn` | **`RSSI`** | Simplified label; now 3-state coloured |
| 77 | `NewCmds` | **`SNR`** | NewCmds never set by FC; replaced with SNR pseudo flag |

### 2. Pseudo Flags — Computed in GCS from Link Stats

FC AllFlags still sends the raw flag byte, but LQ/RSSI warning bits are now
**overwritten by the GCS** from CRSF link statistics (tag 69). Three new pseudo
flags added. Non-CRSF receivers grey out all five CRSF flags.

| Bit | Flag | Condition (boolean) | Source |
|-----|------|---------------------|--------|
| 50 | Loss | `uplink_lq < 25` | link_stats |
| 53 | LQ | always set when CRSF active | link_stats |
| 58 | FS | `uplink_lq < 50` | link_stats |
| 75 | RSSI | always set when CRSF active | link_stats |
| 77 | SNR | always set when CRSF active | link_stats |

### 3. Three-State Colouring

**Flag grid** (each CRSF flag coloured by actual value, not boolean on/off):

| Metric | Green (OK) | Orange (Warning) | Red (Critical) |
|--------|-----------|-----------------|---------------|
| LQ % | ≥ 80 | 50–79 | < 50 |
| RSSI dBm | ≥ -90 | -100 to -91 | < -100 |
| SNR dB | ≥ 6 | 0–5 | < 0 |

**Link stats panel** (value labels coloured):

| Metric | Green | Orange | Red |
|--------|-------|--------|-----|
| Losses count | 0 | 1–2 | ≥ 3 |
| Failsafes count | 0 | 1–2 | ≥ 3 |

### 4. Build Script (fc_build.py)

Now builds **all commissioned targets** by default (was single-target SPEEDYBEEF405WING):

```
UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI
```

Override with `BOARD=UAVXF4V3` for single target.

### 5. CRSF Grey-Out Logic

`crsf_active` now checks `self.link_stats is not None` (link stats packet received)
instead of reading flag bit 50 (which was S-RC, now repurposed).

## Rationale

- **LQ/RSSI/SNR thresholds** agreed from iNav/ELRS/Betaflight research:
  - ELRS: `osd_link_quality_alarm = 60`; Oscar Liang: turn back < 90%, never below 50%
  - RSSI: -100 dBm universal "turn back" (sensitivity + 10 dBm margin)
  - SNR: < 0 dB = signal below noise floor
  - Failsafe: 3+ per session indicates hardware/environment problem
- **FC confirms** non-CRSF protocols (SBus, Spektrum, CPPM) have their own signal
  detection — trackers never interfere with them (trackers only written in CRSF case).
  No artificial LQ-from-RSSI needed.
- **iNav** uses same approach: CRSF gets native LQ/RSSI/SNR; SBus gets pseudo-LQ from
  frame-lost flags; others rely on frame timeout. No cross-derivation.

## Build Verification

- FC build: all 5 targets compile clean (UAVXF4V3 through FLYINGRCF4WINGMINI)
- GCS: `py_compile` passes for main_window.py
