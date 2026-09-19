# Session Report: Rok_Quad Alt-Rise, Config-Bit Reboot Policy, Rev-Props Audit (Sep 13)

## 1. Rok_Quad alt-rise — physics floor closed

### Problem
`original/Rok_Quad.af` was the last original-fleet failure under the AH critique: alt-hold 5 m rise **2.51 s vs the 2.5 s criterion** (`AH_CRITERIA_MR`). The prior analysis flagged it as a **T/W physics floor**: `ALT_THROTTLE_COMP_LIMIT` 0.15 capped the feedforward+comp authority below what the mass/prop combination could actually deliver.

### Evidence (Greg, provided real motor/prop data)
- AUW 1344 g, 520 mm wheelbase, 3S `PHYS_BATT_V=11.1`, 920 Kv, 9×4 props.
- **610 g thrust/motor** × 4 = 2440 g → T/W = 2440/1344 = **1.816 ≈ 1.82** (matches the previously assumed physics).

### Simulation results (already established, re-run to confirm)
| `ALT_THROTTLE_COMP_LIMIT` | 5 m rise | vs 2.5 s criterion |
|---|---|---|
| 0.15 (shipped) | 2.51 s | FAIL |
| **0.20 (adopted)** | **2.24 s** | **PASS** |
| 0.25 (proposed/generic Quad) | 2.06 s | PASS |

### Change
`airframes/original/Rok_Quad.af:116` `ALT_THROTTLE_COMP_LIMIT 0.15 → 0.20`. Chose **0.20 (not 0.25)** as the minimal lift that passes, so the original fleet reflects a *validated* rather than *overpumped* comp; `proposed/Rok_Quad.af` keeps 0.25. (The parameter range is 0.05–0.25; `generic/Quad.af` ships 0.25.)

### Verdict
- Full `original/` fleet re-run (9 frames): **0 issues**. **Rok_Quad alt rise was the ONLY remaining original-fleet FAIL — fleet now fully critic-validated.**
- Synced `original/Rok_Quad.af` byte-identical to all three kits.
- `backup_angleunits/Rok_Quad.af` left untouched (historical snapshot).

## 2. Config-bits are boot-scoped — Apply & Reboot offer

### Problem
Changing the **HaveGPS** config bit (`eUseGPS`, Config2 bit 3, tag 73) did **not** arm the Apply & Reboot offer, because `PARAM_BOOT_REQUIRED` only listed individual sensor/selector tags. The GPS **port** is configured at boot-time init only, so a GPS-bit toggle had no effect until some *other* boot-scoped edit (e.g. RxType) happened to trigger the offer — Greg's observed workaround.

### Decision (Greg 2026-09-13)
> "I think perhaps a forced reboot if any of the config bits are changed."

Any change to **Config1Bits (tag 15)** or **Config2Bits (tag 73)** now arms the one-shot Apply & Reboot offer, mirroring the existing boot-scoped tags. Rationale: config bits are latched by `DoConfigBits()` inside `ConditionParameters()` at boot; the safe, simple policy is "any config-bit edit → reboot to take effect" rather than a per-bit boot analysis.

### Implementation
- **GCS** `parameters.py` `PARAM_BOOT_REQUIRED` `{8,12,13,14,35,43,44,47,71,89}` → `{8,12,13,14,15,35,43,44,47,71,73,89}` (+ rationale comment). Path verified: `bit_changed → param_changed → _enqueue_live_write(15/73)` sets `_boot_reboot_offer` → drains → `_offer_apply_reboot_for_boot_param`.
- **FC** `params.c` `BootRequiredParameterTags[]` mirrored (entries 15 Config1Bits, 73 Config2Bits with boot-latch comments). `ParamBootRequired()` has no live callers — it is the source-of-truth mirror the GCS list tracks.

### Build/verification
- GCS `parameters.py` py_compile clean.
- FC **MATEKF411WING** and **SPEEDYBEEF405WING** both build clean (`MATEKF411WINGQ_r0.bin` 233 980 B, `SPEEDYBEEF405WINGQ_r0.bin` 234 436 B).
- `parameters.py` synced byte-identical to all three kits.
- AGENTS.md updated (12-tag list; boot-scoped-by-policy note supersedes the RxType workaround).

## 3. Rev-props config-bit audit — CLEAN

### Scope
`PropsInwardsMask` (Config2 bit 4, `0x10`): SET = props-in (usual convention), CLEAR = **props-out (UAVX preference)** → drives `MultiPropSense = +1/-1`.

### Audit result (2026-09-13)
- **61 `.af` files** scanned across original/proposed/generic/user/backup_angleunits: **0 set the bit** (all use POD `BatteryComp|FastStart` ± `GPS|NavBeep` = 0x03 / 0x0B / 0x4B).
- **FC default** `DEFAULT_CONFIG2 = UseBatteryCompMask|UseFastStartMask|UseGPSMask|UseNavBeepMask` = **0x4B** → bit 4 CLEAR.
- **GCS default** `_set_config_default(CONFIG2_BITS)` = `int(eUseBatteryComp|eUseFastStart|eUseGPS|eUseNavBeep)` = **0x4B** → bit 4 CLEAR.

**All three layers agree: props-out.** No change needed.

## Cross-reference
- AGENTS.md → "Original-airframes critique" entry (Rok_Quad resolved) and "Config bits 15/73 boot-scoped by policy" entry and "Rev-props config bit AUDITED CLEAN 2026-09-13".
- `wiki/docs/critique_report.md` v4.0 (original fleet table) + `wiki/Session_Report_CritiqueRetune_Aug29.md` (Rok_Quad 2.51 s entry now historical).
- Prior session: `wiki/Session_Report_LogFileDebris_Sep13.md` (git/GitHub log purge, earlier the same day).