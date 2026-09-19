# Session Report — USB Link Resilience & Startup Fix — Aug30

## Symptom
1. The GCS **failed to launch** with `AttributeError: 'MainWindow' object has
   no attribute '_trace_combo_tooltip'` at `setup_ui()`.
2. The **USB connection drops out periodically**; the GCS drops to
   "Disconnected" and needs a manual Connect.

## 1. Startup crash — `ui/main_window.py` `setup_ui()`
`_set_trace_combo_state('neutral')` was called at the toolbar creation point
**before** `self._trace_combo_tooltip` had been assigned, so the neutral-state
call hit an uninitialised attribute. Fixed by moving the tooltip assignment
above the `_set_trace_combo_state('neutral')` call (lines ~637-643). GCS now
starts again.

## 2. Periodic USB dropout — `TelemetryThread` (`ui/main_window.py`)

### FC-side audit (no change required — verified clean)
- The USB IN staging ring can only strand the pipe when a foreground writer
  blind-writes past `APP_Rx_ptr_out` (wrap → pump sees empty → host EIO →
  ~30 s re-enumeration). Every live writer is guarded:
  - `SerialTxUSB()` (`UAVXArmQ/src/serial.c:72`) drains `TxQ[eUSBSerial]` into
    the ring byte-by-byte, breaking on `TM_USB_VCP_TxFree()==0`; runs every
    housekeeping loop (`uavxarm-v3-gke.c:92`).
  - `TxChar()` (`serial.c:234`) discards + flags `TxOverflow` if the TxQ ring
    is full — non-blocking by design.
  - `TM_USB_VCP_Putc/Puts` are the only unguarded writers; **zero callers**.
- Conclusion: the FC **cannot** overrun the ring through any reachable path.

### GCS-side root cause — the silent single point of failure
The old read loop did:
```python
except (OSError, Exception): break
```
- `(OSError, Exception)` ≡ `Exception`: it caught **anything**, including
  transient CDC `OSError`s, and silently killed the thread → `connected(False)`
  → manual reconnect required.
- The exception was **swallowed with no diagnostic**, so FC-reset vs
  host-CDC-glitch vs decode-bug could not be distinguished — ever.

### Changes (`TelemetryThread`)
- **`run()`** opens the port, then delegates to:
- **`_link_loop()`** — read/decode loop with an auto-reconnect path:
  - `OSError` (vanished/stalled device): log `"<telem> link error: ... ->
    reconnecting"`, `connected.emit(False)`, close the port, then loop-top
    reopens it with **bounded backoff** (1 s → 2 → 4 → capped 5 s, reset on
    success) until the device returns (USB re-enumeration ~30 s) or `stop()`.
  - Reopen failures are **expected** until the device is back — silently paced
    by the backoff (the drop was already announced by the `<telem>` print).
  - Any **non-OSError** exception is a decode/logic bug: logged, link reported
    down, thread stands down (no hot-reopen of a broken loop).
  - Decoder byte-parsing is byte-for-byte unchanged; fresh frame state
    (`buffer`/`packet`/`esc_flag`) on each successful reopen so a partial
    pre-drop frame cannot bleed into the new session.
  - `connected.emit(True)` on reopen re-drives the existing `on_connected`
    handshake (param reads, rawlog restart) with no GUI changes.
- **`_reconnect_pause()`** — sleeps the backoff in 50 ms slices so `stop()`
  returns promptly (≤ 5 s worst case).
- **`_close_port()`** factored out of `_close_serial()`: closes + nils the
  serial object *without* forcing `running=False`; `_close_serial()` keeps the
  full-shutdown semantics (`_is_closing` + `running=False`).
- **`connect_telemetry()`** now stops any still-running reconnect thread before
  starting a fresh one, so two threads can never pump the same link (race the
  new backoff path introduced).
- Initial-connect failure behaviour preserved: `serial.Serial` raising at first
  open propagates to `run()`'s outer handler → modal `on_error`, exactly as
  before — only reconnect failures are silent/retried.

## Rationale / discourse
- **Retry everything on OSError vs only "port gone"**: on Linux CDC a dropped
  device can surface as ENOENT, ENODEV, EIO, or a read interrupt; trying to
  distinguish them reliably is error-prone. Since a reconnect is cheap and
  gated by backoff, treat *all* OSError as "retryable" and let the 1–5 s
  backoff absorb genuinely-missing ports (they will keep failing until the
  stack re-enumerates — no busy-spin).
- **Non-OSError must NOT auto-reopen**: a decode/emit bug would otherwise loop
  forever printing the same traceback every 10 ms. User-visible log + stand
  down keeps it debuggable and bounded.
- **Silent reconnect-open failures, announced drop**: the `<telem>` line prints
  once at the drop with the exception repr; subsequent open attempts staying
  quiet avoids 30 s of backoff spam while the device is genuinely gone. The
  user still sees Connected → Disconnected → Connected on the button.
- **Backoff 1→2→4→5 s cap**: 5 s worst-case re-acquisition is comfortably
  inside the ~30 s re-enumeration window, and 50 ms-sliced sleeps keep
  `TelemetryThread.stop()` snappy for manual Disconnect mid-retry.
- **A fresh rawlog per link session is intended** (drop → `on_connected(False)`
  stops the old log; reopen → `on_connected(True)` starts a new one). A dropped
  link never feeds the closed logger, and the reconnect path itself emits no
  `raw_bytes`.

## Verification
- `py_compile` clean on `ui/main_window.py` (and `src/main.py`), AST parse OK.
- Decode loop body is byte-identical to the pre-change logic (only the
  exception handling and the reopen branch are new).
- GCS launch / on-air behaviour still requires a PyQt5 test by the user —
  in particular: launch (crash fix), pull/re-plug the USB while connected
  (expect `${telem}` line + auto reconnect ~≤5 s), and confirm rawlog files are
  one-per-session.

## TODO / follow-up
- Correlate the next drop with host `dmesg -w` (`xhci`/`usb 1-… ` errors =
  host/bus side; nothing + `<telem>` print = GCS-thread side) — the new prints
  make the discrimination possible for the first time.
- Consider a GUI status-bar notice ("Link dropped — reconnecting…") on
  `connected(False)` if the current red/green button flip is considered
  insufficient.