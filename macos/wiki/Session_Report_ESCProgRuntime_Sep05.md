# Session Report — Runtime ESC-Prog (AM32/BLHeli) feed-through

Date: 2026-09-05 (Prof Greg & assistant)

## Summary

Implemented the **GCS ESC-button → runtime ESC-programming feed-through** over the
existing 4way/MSP stack in `escprog/serial_4way.c`. The old boot-time ESC-Prog
entry is **removed** — the FC always boots straight to Ready and only enters the
ESC-programming stage when the GCS explicitly asks. **IMPLEMENTED, UNTESTED ON
HARDWARE** (code complete, all 6 FC targets build clean, GCS py_compile clean).

## The flow (user-confirmed)

1. FC boots normally through every boot step to **Ready**. No ESC window at boot.
2. Pilot clicks the **ESC** toolbar button in the GCS → confirm dialog → the
   button **turns red (orange default → red "ESC •" on click)** as a visual
   warning/signpost that a disconnect consequence follows.
3. GCS sends `request(MISC, miscESCProg=13, 0)` (tag 17-style MISC request).
4. FC (disarmed only) ACKs success, then enters `DoESCProg(s)` — a **10 s
   connect window** with **countdown beeps** (one short tick per second),
   yellow LED on.
5. GCS, on the successful ACK, waits **2 s** then **disconnects the serial link**
   — the GCS app stays **open but disconnected**. (User ruling: leave GCS open,
   do NOT close it; also no abort-by-re-click — re-press does nothing since the
   button is disabled.)
6. The AM32/BLHeli App opens the same COM port directly and talks 4way/MSP.
7. App closes / interface-exit / silence → FC releases, re-enables normal
   telemetry, GCS reconnects with Connect.
8. If the window expires with no connection, FC beeps 3× (window-closed
   signal) and returns to telemetry.

## FC changes (`UAVXArmQ/src/`)

- **`escprog/serial_4way.c`**
  - Top-of-file comment rewritten: ESC-Prog is entry-at-runtime-only.
  - Removed the `ESC_PROG_ENABLED 0` boot-era gate (now undefined → `#if` is
    false; no residual references).
  - New defines: `ESC_PROG_BYTE_TIMEOUT_MS 2000` (per-byte, kept from the
    boot-trap watchdog), `ESC_PROG_SESSION_IDLE_MS 4000` (between-frames
    silence abort), `ESC_PROG_CONNECT_WINDOW_MS 10000` (GCS→App window).
  - `readByte()`: `WdtFeed()` added in the wait loop; comment corrected from
    "boot watchdog" to "session watchdog".
  - `esc4wayProcess()` outer loop: `WdtFeed()` at loop top; session-idle timer
    armed before the loop, re-armed on each `SerialAvailable(s)`, re-armed
    fresh after each fully-serviced frame; added
    `else if (mSTimeout(ESCProgTimeoutmS)) esc4wayAborted = true;` — fixes a
    latent hang where a silent link at a frame boundary (no mid-frame byte to
    trip `readByte()`'s timeout) would spin the session forever.
  - `DoMSPCmds()` do-loop: `WdtFeed()` added (non-blocking `GetMSPPacket`
    loop would otherwise starve the runtime IWDG for ~10 s).
  - `DoESCProg()` rewritten: `mSTimer(ESCProgTimeoutmS,
    ESC_PROG_CONNECT_WINDOW_MS)` (was 5000); wait loop feeds the watchdog and
    beeps one short tick (`DoBeep(2,2)`) per remaining second; on connection →
    `ESCProgActive=true; esc4wayInit/DoMSPCmds/esc4wayProcess/esc4wayRelease`;
    asserts `DoBeeps(2)` on session completion, `DoBeeps(3)` if the window
    expired unconnected.
  - `CheckESCProg()` **deleted** (was the boot-stage entry, `#if ESC_PROG_ENABLED`
    guarded).
- **`escprog/serial_4way.h`**: removed `CheckESCProg(void)` and the orphaned
  `DoEscProgramming(void)` declarations (no definition ever existed).
- **`main.h`** `FlightStates`: removed `eStepESCProg` (now unused).
- **`uavxarm-v3-gke.c`**: removed the `CheckESCProg()` boot call (was between
  `SPIClearSelects()` and `InitIMU()`).
- **`params.c` / `params.h`**: freed Config2 bit 2 (was `UseESCProgMask` /
  `UsingESCProg`) → `Unused2_2`, with a comment explaining ESC-Prog is now
  runtime-triggered and needs no config bit. Latch read and extern removed.
- **`telemetry/telem.c`**: `MiscComms` gained `miscESCProg` (13, after
  `miscCycleBBLog`). Dispatch case (disarmed gate): if `F.DrivesArmed` →
  `SendAckPacket(s, miscESCProg, false)`; else ACK true then `DoESCProg(s)`.
  (ACK is **sent before** blocking so the GCS sees success and releases the
  port; rationale: the FC main loop is blocked inside `DoESCProg` for the
  window, so an ACK after would only arrive on reconnect.)

## GCS changes (`uavx-python/src/`)

- **`protocol_enums.py`**: `MiscCommand.ESC_PROG = 13`. `Config2Bits` freed
  bit 2 `eUseESCProg` → `eUnused2_2` (with the same "runtime-triggered, no
  config bit" comment).
- **`airframes/airframes.py`**: legacy token map `USE_ESC_PROG` / `USE_BLHELI`
  now map to `eUnused2_2` (old `.af` files carrying that bit resolve to the
  freed slot, harmlessly unused).
- **`ui/main_window.py`**:
  - Toolbar `ESC` button (orange text default → bold red with "ESC •" on
    click; disabled while the request is in flight).
  - `enter_esc_programming()`: confirm dialog (warns of link release), refuses
    when not connected or in flight, sends `request(MISC, ESC_PROG, 0)`.
  - ACK handling in the tag-51 path: on success `_on_esc_prog_ack()` leaves
    the button red and schedules `disconnect()` after 2 s (`QTimer.singleShot`)
    so the serial port frees for the App; on refuse `_on_esc_prog_denied()`
    restores the button state and warns "disarm and retry".
  - `on_connected(True)` resets the ESC button to normal orange.

## Design decisions & alternatives

- **Runtime-only entry**: the boot stage was inherently a hang risk (a stuck
  programmer wedged boot before IMU/altitude/control init — the Sep-04 fix was
  only a per-byte watchdog mitigation). A runtime request flows through the
  normal `ProcessRxPacket` dispatch, is gated on disarmed, and a runaway session
  only affects ESC programming, never boot. Decisive (user): the pilot clicks
  the button, the FC provides a bounded window, and the consequence is fully
  recoverable. Alternative (keeping boot entry on a config bit) rejected: the
  bit did not remove the hang hazard, and the user lands Ready every boot so the
  config-bit path gives nothing over explicit request.
- **10 s window + countdown beeps** (user-mandated): the pilot must move the
  USB cable mentally "the link is down" — the beeps tell audibly how long the
  window has left; the GCS button/window state is a silent equivalent.
- **Wait, don't push**: `DoESCProg` blocks the main loop (window + session).
  Justified — it is exactly what a serial feed-through must do; the loop is
  watchdog-fed throughout, and all blocking paths have a hard timeout
  (byte/4 s idle/10 s window), so the FC always self-recovers.
- **Why ACK-before-block**: the GCS needs to know it is safe to release the
  port; the FC cannot respond once blocked in the window.
- **Freed the config bit**: ESC-Prog no longer needs a boot-time permission
  bit. The GCS gate is operational (disarmed), not config. Matches the
  "Spare Config Bits" bookkeeping already done for Config1 bit 6 / Config2 bit 7.

## Build / verify status

- FC: `scripts/fc_build.py` → all 6 targets **OK**
  (UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI,
  BLUEBERRYF405).
- GCS: `python3 -m py_compile ui/main_window.py protocol_enums.py
  airframes/airframes.py` → clean.
- **NOT yet hardware-tested**: reflash SPEEDYBEEF405WING, verify Ready-on-boot
  (no ESC window), then GCS ESC button → red → 2 s → disconnect → open AM32/
  BLHeli App → connect within 10 s (beeps) → read/write an ESC → close App →
  wait for release → GCS Connect re-establishes link. Also confirm the window
  expiry (no App) leaves the FC responsive after ~10 s.

## Next

Parameter-defaults cleanup across FC (`params.c` ParamTable defaults), GCS
(`properties.py` / `PARAM_DEFAULTS`), and `.af` files — user-directed.