# Session Report — Test-Mission Config Bit Folded Into Emulation (2026-09-23)

## Summary of Change
Per Greg's directive ("setting the emulation config bit will also generate the test
missions. So the Test mission config bit can be marked unused as we usually do it"),
the FC now derives `F.TestMissionEnabled` from `F.Emulation`, and **Config1 bit 6
is freed** and marked unused in both the FC and the GCS.

| Role | Before | After |
|---|---|---|
| FC latch | `F.TestMissionEnabled = (pConfig1Bits & Unused1_6) != 0;` | `F.TestMissionEnabled = F.Emulation;` |
| Config1 bit 6 | consumed as the Test-WP-mission bit | **FREE** (`Unused1_6`) |
| DC-gain of the change | 4-WP test mission (the rate-Kp ident mission used by emulation) now generates whenever the Emulation bit (Config1 bit 3) is set | identical behaviour at the bench, one fewer bit to set |

## Why
- The test mission is a bench/emulation-only feature: it lives on
  `cDefaultHomeLat/cDefaultHomeLon` and exists purely to exercise the nav chain
  (mission.c:95-132, generated in `InitNavigation`, nav.c:388-390). A real flight
  never wants it, and - The test mission is a bench/emulation-only feature: it lives on
- Config1 bit 6 had already carried three unrelated lives (Dive → TraceMode →
  TestMission), each freeing it back afterwards. Folding the test mission into the
  Emulation bit (Config1 bit 3) returns the bit to the **Spare Config Bits** pool,
  where AGENTS.md already documented it.
- Removes a hidden coupling: previously a bench operator had to remember two bits
  (Emulation + WP Test); forgetting bit 6 silently produced an emu session without
  the test mission.

## What Changed
### FC — `UAVXArmQ`
- `src/params.c` `DoConfigBits()`: replaced the `Unused1_6` read with
  `F.TestMissionEnabled = F.Emulation;` (comment explains config1 bit 6 is free).
  Both `F.Emulation` (line 495) and the new derivation live in `DoConfigBits`, which
  is driven from `ConditionParameters()` (boot: LoadParameters→ConditionParameters,
  and tag-72 commit). `F.Emulation` is also latched early in `LoadParameters`
  (params.c ~846) before `ConditionParameters`, so every entry path sets the flag.
- `src/params.c` `Unused1_6` macro comment: extended to record (was Dive, then
  TraceMode, then TestMission — folded into EmulationEnableMask 2026-09-23).
- `src/nav.c` `InitNavigation`: comment updated (emulation bit ⇒ test mission; no
  separate test-mission bit).
- `src/mission.c` readers (`Altitude > cNavCeilingM && !F.TestMissionEnabled`
  FenceAlarm gate, `NAV_ENFORCE_ALTITUDE_CEILING` clamp) unchanged — they consume
  the derived flag as before, so emulation sessions still get the ceiling
  exemptions intended for the test mission.
- No layout change to `Flags` (`TestMissionEnabled` bit kept in place).

### GCS — `UAVXGS`
- `uavx-python/src/protocol_enums.py` `Config1Bits`: **renamed `eTestMission = 0x40`
  → `eUnused1_6 = 0x40`** (bit 6 FREE), with `eTestMission` kept as an **alias** so
  legacy `.af` files (which write `CONFIG1_BITS = ...|eTestMission|...`) still load
  without clobbering the bit (AGENTS "NO silent combo fallbacks" rule).
- `uavx-python/src/airframes/airframes.py` `_LEGACY_ENUM_TOKENS['Config1Bits']`:
  `'TEST_MISSION'` now bridges → `'eUnused1_6'` (was `'eTestMission'`).
- `uavx-python/src/ui/parameter_window.py` `_create_config_group`: Config1 row 6
  relabelled **"WP Test" → "Unused 1-6"**, checkbox disabled + tooltip ("Unused —
  free config bit ... Emulation now implies the 4-WP test mission"), mirroring the
  existing Config2 `Unused 2-2`/`Unused 2-7` placeholder pattern.
- `uavx-python/src/ui/main_window.py` config strip: **"WP Test" label removed**
  (Config1 bit 6 no longer displayed), `(1, 7, "Clamp")` added for symmetry with the
  full bit set. `config_flags` stayed index/bit-keyed (no positional hazard).
- `AGENTS.md` **Spare Config Bits** table: Config1 bit 6 entry updated to record the
  test-mission fold and the retained `eTestMission` GCS alias.

## Verification
- All 7 FC targets build clean (SPEEDYBEEF405WINGQ_r0.bin 236 324 B, unchanged size
  as expected — data-only, no layout change). `fc_build.py` per AGENTS.
- GCS `python3 -m py_compile` clean on all four touched python files.
- Token-resolution exercised headless: `eTestMission`, `eUnused1_6`, `TEST_MISSION`
  (legacy UPPER), and `eEmulationEnable` all resolve to the same values as before;
  `Config1Bits.DEFAULT` unchanged (22).

## Open / Notes
- Existing `.af` files that carry `eTestMission` keep the bit set in the stored raw
  value (harmless round-trip); the FC no longer reads it on the wire, and any
  re-save writes `eUnused1_6` instead.
- No FC re-verification flight needed for this logic change beyond the normal bench
  emulation check (bit 3 on ⇒ 4-WP test mission generated at `InitNavigation`).