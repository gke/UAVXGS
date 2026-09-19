# Session Report — Raw Telemetry Log & Replay — Aug30

## What changed (GCS, `uavx-python/`)

### 1. Shared wire decoder — `core/frame_decoder.py` (new)
The ESC-stuffing byte state machine that was inlined in
`ui/main_window.py::TelemetryThread.run()` (the `buffer/packet/esc_flag`
loop) was extracted into a reusable `FrameDecoder` class. Live serial and
logfile replay now run **the identical decoder**, so replay is decoded
byte-for-byte the way a live connection is.

- `feed(data) -> (frames, text_lines)`
  - `frames` = complete unstuffed `[SOH .. checksum]` byte strings (EOT is
    the terminator, **not** a member of emitted bytes — matches the old loop).
  - `text_lines` = printable-ASCII runs outside packet framing (boot
    messages); `None` marks a discarded non-printable run (old behaviour).
- State survives between `feed()` calls: raw-input backlog, in-progress
  frame, parked ESC flag → frames split across serial reads reassemble.
- Verified byte-identical to the original inlined loop (self-test: 200
  synthetic frames with random escaped payload bytes + interleaved boot text,
  chopped into random 1..19-byte chunks → identical `(frames, text)` output
  from old and new loops, and restored frame content matches what was sent).

### 2. Always-on raw log — `logger/rawlog.py` (new)
Every byte arriving on the serial port is captured, all the time, whether or
not CSV/KML logging is enabled. Format is deliberately trivial:

    magic  8 bytes  b"UAVXRAW\x01"
    chunk  per serial read:
       marker byte 0xAA (resync anchor)
       u32 LE ms since log open (monotonic)
       u32 LE payload length
       payload = the exact raw serial read bytes

- File: `YYYYMMDD_HHMMSS.rawlog` in the log folder (starts at connect).
- **Airframe rename**: the FC's persisted airframe name only arrives over
  tag 63 *after* connect, so the file starts timestamp-only and is
  `os.rename`d to `…_<Airframe>.rawlog` when the name arrives (POSIX-safe;
  the open handle keeps writing to the same inode).
- `stop()` keeps `_path` (callers log the final filename).
- `forbidden` flag: set while replaying, so `write()` is vetoed — replay can
  never be re-captured (belt-and-braces; replay already bypasses `raw_bytes`).

### 3. Replay window — `ui/replay_window.py` (new)
Loads a `.rawlog`, decodes it once through `FrameDecoder` into a flat
`[(offset_us, frame_bytes)]` timeline, then re-drives the GCS by calling
`main_window.process_packet(frame)` per frame — the exact same downstream
path live telemetry uses. No serial port involved.

Transport (per user directive):
- **STOP** = play/stop toggle (click on / click off).
- **REW / FF** act **while depressed** (`QPushButton.setAutoRepeat`) — hold
  to scan back/forward.
- Speed slider 0.25x–8x (1x = real-time pacing from the recorded ms ticks).
- Timeline scrubber (drag to seek).
- File combo lists `.rawlog` files in the log folder; File ▸ Open Log…
  file picker too.

Seek semantics: rewinding/scrubbing sets `_last_idx` to the first frame at
or after the new offset and **re-sends** captured frames idempotently
(`process_packet` just re-writes the same display values), so looping back
over already-played data is safe and cheap.

### 4. Mode switch & safety (main window)
- File ▸ **Replay Log…** (Ctrl+R): drops the live FC connection first, then
  opens the replay window.
- **Connect while replaying → replay is immediately terminated**, GCS
  returns to the live link (`connect_telemetry` closes the replay window and
  clears the veto).
- **A replay never writes another log**: replay goes straight to
  `process_packet`, never through the serial thread's `raw_bytes` signal;
  the raw-log `forbidden` flag is the extra guard. `check_connection` is
  idle during replay (`connected` is False) so the "no FC data" timer cannot
  fire falsely even though `process_packet` updates `_last_rx_time`.

### 5. Toolbar changes
- **Trace-dump button relabeled**: the "BB" label was dropped (user: "we did not
  want the bb button removed just its 'BB' label"), so the momentary dump
  button now reads **"Dump"** — it still sends `MiscCommand.BB_DUMP` and
  reassembles tag-54 chunks to a save dialog (`dump_black_box` /
  `_finalize_bb_dump`). Once the auto-tune capture ring lands on the FC this
  dumps that instead.
- New **"Replay"** toolbar button (Ctrl+R) → `open_replay`: drops the FC
  connection, opens the replay window transport.
- Trace combo ("Info / Warnings / Errors / All") retained; **options to be
  reviewed into an enum as FC trace types are added** (deferred, TODO).
- KML generation is **no longer a goal of this work** — replaced by a future
  **3D flight-path viewer/reviewer** (map + 3D track replay from rawlog),
  deferred to the TODO list.

## Rationale / discourse
- **Shared decoder over replay-of-packets**: replaying pre-parsed data would
  fork the decode path forever; feeding raw bytes through the one state
  machine guarantees replay fidelity and keeps a single maintenance point.
- **Raw (not packet-only) log**: the stream carries printable diagnostics
  (boot, motor trace) *and* binary packets interleaved; capturing raw bytes
  with timing preserves everything and costs nothing at parse time.
- **1 ms monotonic offsets** (not wall-clock): immune to clock jumps, no
  timezone/offset bookkeeping, sufficient pacing accuracy for replay.
- **`process_packet` as the replay sink**: zero new display code — every
  existing widget (attitude, alt, GPS, flags, params) is driven as if live.
- **Rejecting re-log of replay**: the user's objection is structurally
  handled — replay never emits `raw_bytes`; the `forbidden` veto is defence
  in depth.

## Verification
- `py_compile` clean: `core/frame_decoder.py`, `logger/rawlog.py`,
  `ui/replay_window.py`, `ui/main_window.py`.
- Pure-logic self-tests (no PyQt in the Flatpak sandbox):
  - Decoder byte-identical to original inlined loop + content restored.
  - RawLog round-trip: 60 synthetic frames → file → decoded frames
    byte-exact, timeline offsets monotonic.
  - Airframe rename → `20260830_083059_Rok_Quad.rawlog`.
  - `forbidden` guard blocks `write()` while replaying.
- GCS UI itself cannot be launched in the sandbox (no PyQt5); on-air test by
  the user is required.

## TODO (deferred, pending user decisions)
- **3D flight-path viewer/reviewer** (map + 3D track replay) from `.rawlog`
  — replaces the KML-from-log idea.
- Trace combo options → enum as FC trace types are added.
- On-air check of replay pacing (timed-chunk granularity vs actual frame
  bursts) once live logs exist.

---

## Follow-up — toolbar order & CSV/KML strip (same session, Aug30)

### 6. Toolbar re-ordering (after transport was confirmed)
- **Replay** button moved to sit directly after **Flash** (was right of
  Dump). Toolbar line now reads: Connect | Params | Nav | Calib | Misc |
  Flash | Replay | ~~stretch~~ | Trace: [combo] | Dump | **status (last)**.
- The **connection status label now anchors the right end** of the toolbar
  line (user: "connection status should be last on that toolbar line").
- Task list for the GCS strip work is tracked in the session todo list.

### 7. KML / CSV flight-log machinery removed
The raw telemetry log is always on, so the opt-in KML and CSV flight-log
boxes and their file buttons were declared superfluous ("the kml and log
stuff need to go") and are now gone, along with the whole data path:

- **Toolbar**: `kml_check`, `kml_file_btn`, `csv_log_check`, `csv_file_btn`
  deleted.
- **State**: `self.logger` (CSV `Logger`), `self.gps_kml_logger`,
  `_logging_active`, `_kml_auto`, `kml_path`, `csv_path` removed from
  `__init__`.
- **Handlers**: `_sync_loggers`, `_finalize_loggers`, `_on_kml_toggled`,
  `_on_log_toggled`, `select_kml_file`, `select_csv_file`, `_new_kml_path`,
  `_update_log_location_indicator` deleted. The CSV/KML file buttons were
  also the only consumers of the folder-colour indicator, so it went with
  them.
- **Data path**: `logger.log_flight_data(parsed)` in the tag-13 handler and
  the two `gps_kml_logger.add_point(...)` blocks (tag 13/14) deleted.
  `_sync_loggers()` on take-off / `_finalize_loggers()` on land, disconnect,
  and connect-down removed. The tag-13 armed/disarm block now just tracks
  `_in_flight` (still feeds speech events).
- **Dead classes stripped** from `logger/logger.py`: `Logger` (CSV
  `start_log`/`log_flight_data`) and `GpsKmlLogger` (`.kml` track writer).
  `AnomalyLogger` and `ImuStatsLogger` remain — they are still driven by
  live packets. Module docstring updated to say the raw log supersedes them.

### Rationale / discourse
- **Why delete the classes, not just the UI**: the raw log (`rawlog.py`)
  already captures *everything* with timing and is replayable, so the ~50%-%
  sampling CSV flight log and the KML-from-parsed-points track had zero
  remaining consumers after the toolbar strip. Keeping them would only
  invite a future re-wire to dead data. The **always-on rawlog is the single
  source of record**; any future CSV/KML-style export should be derived from
  it (3D viewer, per AGENTS.md TODO), never written alongside it.
- **`Optional` import** in `logger/logger.py` dropped with the `Logger`
  class that was its only user.
- Log folder prompt label updated ("...for raw telemetry logs") — no longer
  mentions KML & flight logs.

## Verification (follow-up)
- `py_compile` clean: `ui/main_window.py`, `logger/logger.py`.
- GCS launch / toolbar layout still requires an on-air (PyQt5) check by the
  user.