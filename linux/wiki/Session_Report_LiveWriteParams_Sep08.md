# Session Report — Live-Write Parameter Model (no Read/Write buttons)

**Date:** 2026-09-08
**Status:** GCS implemented, py_compile clean; FC `ParamBootRequired` built (7/7 targets). Bench round-trip = user-side reflash test.

## What changed and where

**GCS (`uavx-python/src/`):**
- `ui/main_window.py`
  - `_write_params_live(items)` — the tag-17 live-write queue enqueue. Now **appends to an in-flight `_param_write_list`** instead of dropping, so the transport is lossless (the old drop hazard could lose a live tweak made during a bulk; the commit path depends on every pending value landing before tag-72 packs flash).
  - `_write_param_direct` delegates to `_write_params_live` (trace combo path unchanged).
  - `disconnect()` no longer touches a `ReadParamsButton` (removed); clears `_flash_write_pending` + `_write_in_progress`.
- `ui/parameter_window.py`
  - **Toolbar: no Read, no Write, no "Apply & Reboot" button.** Only Save Params / Load Airframe remain in the button row (plus FLASH name label).
  - `param_changed` (debounced ~350 ms `_live_timer`/`_live_pending`) live-writes every spinbox/combo change to the FC RAM image via tag-17; also records `_boot_reboot_offer` when a `PARAM_BOOT_REQUIRED` index is touched.
  - `_flush_live_writes` pushes the debounce set; after drain it arms the one-shot commit+reboot offer if a boot-scoped index was in that batch.
  - `_offer_apply_reboot_for_boot_param` — single prompt ("Apply & Reboot?") → `apply_and_reboot(confirm=False)`.
  - `apply_and_reboot(confirm=True)` — consolidated safety gate (`can_write_parameters` blocks in flight), optional confirm; snapshots the **whole** widget image (`_written_float`/`_written_params`, `_verify_mode=True`) so the post-reboot tag-71 readback verifies the flash load; flushes live writes; waits for the queue to drain; commits.
  - `_when_param_write_drained(callback)` — polls `main_window._param_write_list` empty (20 ms) up to 200 tries, then fires.
  - `_commit_to_flash` — clears dirty, `send_afname(...)` then `send_param_commit()` (tag-72), arms `main_window._pending_param_verification`.
  - Removed dead: `write_params()` (old tag-17 batch Write button), `_on_write_success`, `_on_read_success`, `_on_params_typed_sent`, `_on_write_progress`, `_on_apply_reboot_clicked`, the tag-17 `ack_handler.register_button` (it would have sat forever in `pending_requests` and flickered a button that no longer exists), the `WriteParamsButton` styling in the verify/commit/timeout paths, `_write_request_id`.
  - `reset_read_button()` is a documented no-op; `reset_write_button()` is state-only; both retained so the read/verify call sites stay unchanged.
  - Stale comment fixes (interlock comment, "Saved" stray, timeout block button refs).
- `parameters.py` — `PARAM_BOOT_REQUIRED = {8, 12, 13, 14, 35, 43, 44, 47, 71, 89}` (mirror of FC `BootRequiredParameterTags`).

**FC (`UAVXArmQ/src/`):**
- `params.c` — `ParamBootRequired(uint8 tag)` over the static `BootRequiredParameterTags[]` (8,12,13,14,35,43,44,47,71,89); `params.h` prototype. **No behaviour change** — pure GCS-facing authority so the GCS knows which edits need a reboot offer. Built earlier as part of this effort; verified clean.

## Rationale and logical discourse

**Why live-write (Greg's model, confirmed via question):** spinbox/combo edit → tag-17 → FC RAM `Config.ParamData[i].f` + `e->target` (control loops read RAM each tick, so the value applies immediately — even in flight). The FC persists to flash **only** via its own opportunistic `RefreshConfig()` (`ConfigChanged && State != eInFlight`; bench ~1 s, never in air) and on disarm. This removes the batch-Write-and-verify latency and the Read-button need (connect auto-populates the page via tag-71). Flash endurance is not a concern at one config-write per debounce batch.

**Remove the Read button:** the connect flow already pulls all 128 params (tag-71, main_window:1800 → param_window.read_params) and the post-reboot verification re-reads them; a manual Read was pure redundancy. `read_params` is retained as the transport for the auto/verify flows.

**Remove the Write button — Greg: "we do not need the write button at all".** Every param now writes live; a batch Write is meaningless. The only thing a button was still for was the *commit* (persist+reboot) for boot-scoped params — but Greg: "**There are no Apply & Reboot buttons.**" So the visible button concept was dropped entirely; the commit+reboot is a **one-shot auto-offer** that only appears after editing one of the boot-scoped tags. Rationale: normal edits persist via the opportunistic/disarm path with zero user action; only a boot-scoped edit *cannot* take effect until power-on, so only that case deserves an interruptive reboot prompt. Default = Yes (the change is otherwise invisible until reboot), abortable.

**Boot-scoped set (10 of 128):** RFSensorType(8), IMUFiltType(12), BBLogType(13), RxType(14), ESCType(35), AFType(43), TelemetryType(44, deprecated), GyroLPFSel(47), ASSensorType(71), AccLPFSel(89) — consumed only during boot-time init (RX protocol/ISR, IMU/attitude filter setup, ESC/PWM pin config, sensor init). Every other param reaches `e->target` live. `BBLogType` is a dead slot (blackbox removed in favour of Trace) but is kept in the list for completeness — harmless. List lives in **two mirrored places** (`params.c` `BootRequiredParameterTags` and GCS `PARAM_BOOT_REQUIRED`); AGENTS.md pins the keep-in-sync rule. Alternatives weighed: enumerate *live* apply params instead (fragile — `e->target` is assigned in fns.c, opaque without a full scan; new 118-vs-10: the small boot list is the stable, auditable subset).

**Safety gate consolidation:** with the button gone, the flight-safety check (`can_write_parameters`) moved **inside** `apply_and_reboot()` so the auto-offer cannot be a back door that commits mid-air. Rejected: leaving the gate only in the old button handler (offer path would bypass it).

**Verify after commit:** even though nothing is batch-sent anymore, `apply_and_reboot()` snapshots the whole current image and arms `_verify_mode`; the post-reboot first-flight-packet triggers `_request_write_verify()` (tag-71), and any mismatch raises the existing flashing WRITE alarm. This preserves Greg's "commit NACK / verify mismatch must not be wiped" invariant without resurrecting a button.

**Lossless transport:** changed `_write_params_live` drop→append. The old "a bulk already carries the value" rationale is gone (no UI bulk sends), and the commit's flush-then-drain depends on RAM containing the very latest value before tag-72 packs the block. Rejected: single-shot "wait for the lazy debounce" — the 350 ms timer plus the 5 ms serial gut are already the pacing; appending to the existing pump is strictly simpler and provably lossless.

## Build / verification status

- FC: `ParamBootRequired` added → `python3 scripts/fc_build.py` all 7 targets build clean (exit=0, 0 errors).
- GCS: `python3 -m py_compile src/ui/parameter_window.py src/ui/main_window.py src/parameters.py` → OK.
- Grep checks: no `ReadParamsButton`, `WriteParamsButton`, `write_params(`, `_on_write_success`, `_on_read_success`, `_on_params_typed_sent`, `_write_request_id`, `register_button` references remain in `src/`.
- **Not bench-verified here** (no board in the sandbox). **USER (Greg): next bench session** — connect, tweak a rate gain, confirm it applies without any button; flash persistence still occurs on disarm; edit AFType (index 43) and confirm the one-shot "Apply & Reboot?" offer appears, Yes reboots the FC, and the post-reboot readback verifies green (no WRITE alarm).
### Combo routing fix — auto-offer now fires for AF_TYPE (follow-up, post-bench report)

**User report (PreFlight):** editing AFType (idx 43) did not show the "Apply & Reboot?" offer.

**Root cause:** the advanced-panel selector combos (AF_TYPE included) connect to
`combo_changed` (parameter_window.py:3531 `currentIndexChanged → self.combo_changed`),
NOT `param_changed` (spins only). The live-write debounce + `_boot_reboot_offer` logic
had been added to `param_changed` alone, so every selector-type change was recorded as
dirty but **never live-written and never offered a reboot** — and all 10 boot-scoped
params (RFSensorType, IMUFiltType, BBLogType, RxType, ESCType, AFType, TelemetryType,
GyroLPFSel, ASSensorType, AccLPFSel) are selector combos, so the feature was fully inert
via that route. Setup-page combos also funnel into `combo_changed`
(`_setup_sync_to_advanced` calls it explicitly), so one fix covers both entry points.

**Fix:**
- New shared `_enqueue_live_write(idx)`: lazy-inits `_live_pending`/`_live_timer`,
  starts the 350 ms debounce, and arms `_boot_reboot_offer` for `PARAM_BOOT_REQUIRED`.
- `combo_changed` and `param_changed` both tail into `_enqueue_live_write` (single
  implementation, no duplicated block).
- `_pending_live_float` already resolves QComboBox via `float(currentData())` — U8
  selectors write their raw enum float, correct for the unified float storage.

**Dead safety gate revived:** GCS `can_read_parameters`/`can_write_parameters` were
called via `hasattr` guards at the read/commit sites but **did not exist on
MainWindow** — the in-flight commit interlock was silently a no-op for the whole
life of the live-write model (FC-side `RefreshConfig` gating was the only real
protection). Now implemented on `main_window` (returns `(safe, reason)` tuples):
connection + param_window `_flash_write_pending`/`_write_in_progress` for read;
plus in-flight (`FlightState.eInFlight`) and in-progress-commit for write. The
`read_params` call site now unpacks the tuple (a bare `if not <tuple>` never
blocked — non-empty tuples are truthy).

**Build/verification:** GCS py_compile clean (parameter_window, main_window).
No FC change. **USER: re-test — editing AFType (advanced or Setup page) while
connected in PreFlight should now: apply instantly via tag-17 (green sync), then
pop the one-shot "Apply & Reboot?" offer; Yes → commit+verify+reboot; post-reboot
readback confirms the new AFType with no WRITE alarm.**

### Load-defaults flow + readback feedback loop (second bench report)

**User reports:** "Load and write defaults for 'Shadow'?" dialog still said
"You will need to press Write to send them to the FC"; an unexpected "P90?"
question appeared during load/connect; RED screen "PARAM VERIFY FAIL 7 mismatches".

**Root causes (three independent defects):**

1. **Stale dialog + no-write load.** The Airframe Defaults dialog text referenced
   the pre-live-write model ("press Write") and `_load_airframe` genuinely did
   exactly that: it filled the UI (blockSignals) but never sent anything to the
   FC, while the status bar claimed "— writing to FC". Under the no-button
   live-write model a bulk load silently left FC RAM with the OLD airframe's
   values, so a subsequent commit persisted stale values and the post-reboot
   verify (correctly) flagged the real gap — hence "7 mismatches".

2. **`update_params_from_typed` readback loop (the P90 pop).** The tag-71
   readback / tag-17 echo path set widgets WITHOUT `blockSignals` — this is the
   Sep-07 defect class, but in the copy that became the PRIMARY read path after
   the Read/Write button removal (connect auto-populate). Every readback
   re-fired `combo_changed`/`param_changed`: popped the protected-param confirm
   dialog for the first differing selector (P90 = ASSensorType, idx 89), echoed
   the FC's own values back as phantom live-writes, and re-armed the Apply &
   Reboot offer. `_set_widgets_from_raw`/`_verify_parameters` were already
   correct; only this path was missed.

3. (Already fixed in the prior entry: selector combos routing through
   `combo_changed` without the live-write/offer — the actual P90 offer arm.)

**Fix:**
- `update_params_from_typed` normal-update now wraps every widget set in
  `blockSignals` and updates `_committed_values` for protected params (same
  discipline as `_verify_parameters`). Readback/echo is not a user edit —
  no dialogs, no phantom writes, no re-armed offer. Existing tail
  (`update_config_display`, `sync_setup_from_advanced`, dirty-clear) preserved.
- `_load_airframe`: when connected, after `_set_widgets_from_raw` + limits it
  live-writes the FULL loaded widget image to FC RAM via `_write_params_live`
  (the no-button model: Load IS the write), then arms `_boot_reboot_offer` for
  the first boot-scoped param in the set and reuses
  `_when_param_write_drained → _offer_apply_reboot_for_boot_param`. Offline load
  is UI-only with an honest status label. The dialog body now says values are
  written to the FC immediately and boot-only params will ask about reboot.

**Build/verification:** GCS py_compile clean (parameter_window, main_window,
parameters). No FC change. **USER: reload Shadow → the UI populates, FC RAM is
updated, ONE Apply & Reboot offer appears for the boot-scoped selectors; on
Yes the post-reboot verify should now pass green (no PARAM VERIFY FAIL).**

### Crash + flashing during load — load path hardened (third bench report)

**User report:** after the previous fix, Load Shadow.af still showed P90 and 10
verify mismatches (e.g. PITCH_RATE_KP Written=0.0022 vs Received=0.00225; four
params Written=0 vs tiny nonzero received — the FC still held OTHER values:
the loaded set was NOT reaching RAM/flash, so the commit persisted stale values
and post-reboot verify flagged them). Then a worse failure: the Physics window
started flashing, desktop icons flashed, and the GCS crashed.

**Root causes:**
1. **The load-push I added used the LIVE per-param queue for all 128 params**
   (`_write_params_live`) WITHOUT setting `param_window._write_in_progress`.
   Each tag-17 echo ACK therefore fully re-entered `update_params_from_typed`
   → `update_config_display()` + `sync_setup_from_advanced()` + status churn,
   128× in ~1 s, interleaved with the drain-poll and (on drain) a modal
   Apply & Reboot offer in a nested event loop — widget churn behind a modal,
   an event storm, then a Qt segfault on the bench.
2. **Persistent WRITE alarm:** after the earlier PARAM VERIFY FAIL the
   `AlarmFlashBox` WRITE source stays active and flashes red (by design) until
   the next successful commit ACK. Greg was loading against a still-flashing
   alarm box, compounding the perception ("physics window flashing").
3. **The tag-17 echo path was NOT rendering inert during writes** (only DIED
   after `update_config_display()`); the verify readback was gated but normal
   reads/echoes weren't.

**Fix (load path hardened and serialized):**
- `_load_airframe` no longer uses the live per-param queue. When connected it
  now sends the whole widget image via the existing **batch** `send_params_typed`
  with `_write_in_progress = True` for the duration (set False in on_complete).
  Echo ACKs therefore take the early-return in `update_params_from_typed` —
  the UI stays completely static during the push. The Apply & Reboot offer is
  shown exactly once, from the batch on_complete callback
  (`_offer_apply_reboot_for_boot_param(idx)` now accepts an explicit index).
  Offline load remains UI-only with an honest label.
- `update_params_from_typed`: `update_config_display()` moved BELOW the
  verify/write-in-progress guard so no display refresh runs during any write.
- (Carried from the previous entry: signals blocked in the normal readback path
  — this is what kills the P90 pop on connect.)

**Build/verification:** py_compile clean (parameter_window, main_window). No FC
change. **USER: relaunch the GCS FRESH** (the persistent flashing alarm from the
failed verify only clears on the next successful commit), then Load Shadow →
UI populates and stays static, values are batch-written to FC RAM, ONE Apply &
Reboot offer appears → Yes → commit+reboot → post-reboot verify should now read
back the loaded values (no PARAM VERIFY FAIL). If a residual flash still appears,
capture the terminal for the Python traceback so we can pin the segfault.
