# Session Report — First-Connect Signal Bug (GCS first-link hang)

**Date:** 2026-08-31
**Component:** GCS `TelemetryThread` (`uavx-python/src/ui/main_window.py`)
**Build/verify:** `python3 -m py_compile ui/main_window.py` ✓

This is the one-line bug that produced *three* apparently unrelated bench
symptoms on Greg's UAVXF4V3 test board (serial UART adapter to telemetry
USART). One root cause, one emitted signal.

---

## The bug

`TelemetryThread.run()` connected the serial port on first open and then
entered `_link_loop()`. But `connected(True)` was emitted **only** inside
`_link_loop`'s **reopen** branch (the `if self.serial is None or not
self.serial.is_open:` path). The *first successful* open never signaled the
GUI at all. So the first-connect sequence ran the whole lifetime without ever
reaching `on_connected(True)` → the GUI stayed in its initial "Connecting…"
state forever.

Why had it never shown up before? USB-CDC links *usually* hiccup once shortly
after open, the link drops, the reopen branch fires, `connected(True)` is
emitted, and the GUI transitions normally. A **hardwired UART adapter never
drops**, so the reopen path — the only path that emitted `True` — never runs.
First-connect hangs are therefore *more* likely on a proper wired serial link
than on USB — the exact opposite of intuition (the USB link was "accidentally"
masking the bug via its instability).

### Symptom chain (all one bug)

| Symptom | Mechanism |
|---|---|
| Connection light stays red "Connecting…" | `connected(True)` never emitted on first open |
| Param window `READ LOCKED` (orange) forever | `on_connected(True)` is what sends `PARAM_TAGGED_READ` (tag-255); never sent → no table → `_set_read_lock(True)` (`parameter_window.py:800`) never released |
| "Mag cal not responding / CPPM not decoded" | Corrected on the bench: with the table now downloading, ACC/KF/horizon all live; the apparent dead subsystems were downstream of the missing readback. RC bars moved immediately once re-connected with the fix |

The CC runner confirmed on the bench: after the fix, connect → green
"Connected", param readback flows, `READ UNLOCKED` (green), and the main-window
control bars respond to CPPM sticks — CPPM decode was healthy all along
(verified FC-side for TIM2-CH1/PA0 at `isr.c:160-171`, default eCPPMRx at
`params.c:156`, tag-22 sent unconditionally at `telem.c:1334`).

---

## The fix

`uavx-python/src/ui/main_window.py`, `TelemetryThread.run()`:

```python
self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
# First successful open: signal the GUI connected NOW, not only on
# a later reopen. A reopen (line 274) exists for dropped CDC links;
# a stable UART adapter never drops, so without this the GUI stays
# stuck at "Connecting..." and never triggers the on_connected(True)
# param readback / rawlog start / read-lock release.
self.connected.emit(True)
self._link_loop(serial)
```

Emit `connected(True)` on first open, immediately after `serial.Serial(...)`
succeeds. Reopen-branch emit stays for genuinely dropped links.

### Alternatives rejected

- **Emit on first *packet* instead:** adds coupling to telemetry cadence and
  would still leave the readback silent on a healthy-but-quiet link; opening
  the port is the correct and sufficient success criterion.
- **Move emit into `on_connected` internals / GUI timer poll:** would blur the
  thread/GUI boundary and risks double-transition. Single emit point per
  successful open is the minimal correct change.
- **Leave as-is, "USB links always hiccup":** relying on a fault in one
  transport to drive another's UI state is exactly the wrong invariant (see
  rationale above).

## Follow-ups

- None pending for this bug. The `.af` write (tag-72) — previously blocked by
  the read-lock — is now the next bench step for the F4V3 test board.