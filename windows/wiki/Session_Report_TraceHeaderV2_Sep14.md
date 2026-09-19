# Session Report — Trace Dump Header v2: self-describing capture (pot + gains in the dump) (Sep14)

Date: 2026-09-14. Companion docs: AGENTS.md **Trace Capture** section (ring
layout table updated to v2), `trace.h`/`trace.c` header comments, GCS
`ui/trace_viewer.py` module docstring + `_parse_gain_block()`.

## 1. What changed

### Goal (Greg's directive)
Embed the Rate-Gain channel value (Ch10 pot `RateGainScale`) **and** the current
controller gain parameters into the trace dump header, so a capture is
self-describing: the dump says exactly what gains flew and what the rate loops
were scaled by, without needing a side-channel (tag-71 readback / `.af`) at dump
time.

### Decisions locked during the session (with the user)
- **Value stored = the decoded multiplier `RateGainScale`** (`rc.c`
  `powf(4.0f, 2.0f*RC[eTransitionRC]-1.0f)`, range 0.25..4.0, default 1.0,
  applied at `control.c:479/492/511`), **not** the raw pot position. The GCS
  decodes it straight to the multiplier that was active; the raw pwm would need
  a decode map on the GCS side for no benefit.
- **Fixed-point u16 ×1000** (`TRACE_RATE_GAIN_Q = 1000.0f`): real 0.25..4.0 →
  code 250..4000, clamp to 0..65535. **0 = "legacy / not recorded"** — v1 dumps
  wrote 0 in this slot, so the code is a clean v1/v2 discriminator AND a
  never-used pot value. "We have not obtained any mission files yet so I am
  dubious about needing a legacy marker" — the 0-code interpretation is the
  cheap version of a legacy marker, no separate field needed.
- **Snapshot at capture-time, not commit-time** (Greg: "Most secure is build the
  header at the end of the capture with all of the current plus the qualifying
  pot gain but that takes a little more space - probably minor"; "saving at that
  point overcomes the potential for in air changes to gains but that should be
  locked out by the gcs"). The header is written at capture-OPEN, capture-end and
  disarm-commit; all three must carry the **capture-open** values. If the pot is
  reset on the ground after landing (the documented field workflow: run the trace
  tests, then reset the pot to centre and bake the measured gain into the `.af`),
  a live-read-at-commit header would record a scale that never applied to the
  capture. So the snapshot lives in FC globals (`TraceRateGainCode`,
  `TraceGains[21]`) filled once by `TraceSnapshotGains()` at the
  `eTraceArming → eTraceCapturing` transition, and `TraceWriteHeader()` always
  serialises the snapshot.
- **Gain block layout**: v2 header grows 32 → 128 B. Base 32 B unchanged
  (recordSize still 16); the gain block sits at bytes 32..119 and is
  `gainCount u16 (=21) + TRACE_GAIN_VERSION u16 (=0) + 21 f32`. Per axis
  (Roll, Pitch, Yaw) the loop-read order: `R.Kp, R.Kd, R.Max, P.Kp (QKp), P.Ki,
  P.IntLim, P.Max`. Record region shifts to slot 128; `TRACE_DATA_BASE` becomes
  128. `gainCount` is the forward-compat guard: a future version extends/trims
  the block without an unconditional header-size growth, and a GCS that sees an
  unexpected count treats the block as absent rather than mis-parsed.
- **v1 dumps stay loadable** (parity with the `parse_trace` version branch):
  GCS branches on `version` — v1 header 32, records at 32, no gains;
  v2 header 128, records at 128, gains dict populated.

### FC (`UAVXArmQ`)
- `src/trace.h`: `TRACE_VERSION` 1→2; `TRACE_DATA_BASE`/`TRACE_HEADER_SIZE`
  32→128; new `TRACE_RATE_GAIN_Q/MIN/MAX`, `TRACE_GAIN_COUNT (=21)`,
  `TRACE_GAIN_VERSION (=0)`, `TRACE_GAIN_OFF_COUNT/VERSION/DATA` (32/34/36);
  `TraceSnapshotGains()` (static, trace.c); header comments updated for the v2
  layout + capture-time snapshot rationale. Stale "slot 32 / offset 14 / 32 B
  header" comments corrected.
- `src/trace.c`:
  - New globals `TraceRateGainCode` (u16) + `TraceGains[TRACE_GAIN_COUNT]`
    (real32).
  - New `TraceSnapshotGains()` — clamps `RateGainScale*1000` to
    `TRACE_RATE_GAIN_MAX/MIN`, records it as `TraceRateGainCode`, then reads
    `A[a].R.Kp/.Kd/.Max` + `A[a].P.Kp/.Ki/.IntLim/.Max` per axis into
    `TraceGains[]`. (The `P` struct field is the generic `PIStruct` — the `Q` in
    `QKp` is the GCS naming semantic only.)
  - `TraceWriteHeader()` now writes the base 32 B (magic, version=2 via
    `TRACE_VERSION`, type, rsize, flags=0, axisMask=0x07, period,
    `TraceRateGainCode`, snapStart, count, start/end ticks), then
    `TRACE_GAIN_COUNT` u16 + `TRACE_GAIN_VERSION` u16 + the 21 `TraceGains[]`
    floats, then zero-pads to `TRACE_HEADER_SIZE` (128). Dropped the old
    inline-read of `RateGainScale` (was a live read at each header write — the
    exact hazard the user flagged).
  - `TraceCapture()` `eTraceArming` settle-pass path calls `TraceSnapshotGains()`
    immediately before `TraceWriteHeader()` at capture-open.
  - `StartBBChunkedDump()`/`SendBBChunk()`/`InitBlackBox()` unchanged —
    `BBChunkHdr[TRACE_HEADER_SIZE]` (now 128) carries the whole header, records
    stream from `TraceSnapStart` (=`TRACE_DATA_BASE`=128). `TraceCommit()`
    writes `[0..8+recordCount*16) = 128+count*16`.
  - Ring budgets: 64K `(65536-128)/16` = **4089** records; F411 32K = **2040**.
    Captures: rate/IMU ~2.72 s; slow types ~27.2 s (64K), half on F411.

### GCS (`UAVXGS`)
- `src/ui/trace_viewer.py`:
  - Constants: `TRACE_VERSION = 2`, `_BASE_HDR_SIZE = 32`,
    `TRACE_HEADER_SIZE = 128`, `TRACE_GAIN_COUNT = 21`,
    `_GAIN_BLOCK = Struct('<HH21f')`, `_GAIN_FIELDS` (7 per axis),
    `AXIS_NAMES`; helper `_parse_gain_block(data)` → `{axis: {field: value}}`
    (None when count ≠ 21 or absent).
  - `parse_trace()` branches on version: v1 → header 32 / records at 32 /
    gains None / pot None; v2 → header 128 / records at 128 / gains dict.
    Returns `rate_gain_scale` (decoded via `rate_gain_scale(code)` =
    `code/1000` unless 0 → None) + `gains`.
  - Viewer info panel now shows `rate-gain pot scale xN (code N)` + per-axis
    gain rows (`R.kp R.kd R.max Q.kp Q.ki Ilim Q.max`); "not recorded" when
    code 0; "(no v2 gain block)" when a v2 dump lacks a valid block.
- `src/tests/generate_trace_samples.py`: emits v2 headers — base with
  `RATE_GAIN_CODE = 1000` (×1.0), the 21-float gain block (plausible MR-class
  numbers), pad to 128, records at 128; unpack sites + record offsets updated.
  Samples regenerated in `tests/trace_samples/`.
- `src/ui/main_window.py`: **dump-finalize watchdog bug found by bench flight
  (Greg: "captured a trace and disarmed and pushed dump — nothing!")**. The
  no-capture `eBBNone` header is now 128 B = exactly one full dump chunk, so
  the old end-of-dump heuristic (`finalize when a chunk is `len < 128`, tag-54
  handler) never fired → an empty dump produced *no viewer, no dialog,
  nothing* (v1's 32 B header was short, so it worked). Fixed with a 500 ms
  single-shot `_bb_dump_timer` (armed on every received BB chunk, fires
  `_finalize_bb_dump` on idle) + the all-zero no-capture detector now raises a
  visible `QMessageBox` "No Trace Capture" listing causes (trace type None /
  ch8 never armed / no disarm commit). py_compile clean.
- `AGENTS.md`: ring-layout section rewritten for v2 (base 32 B table with
  `rateGainCode` at 14-15; gain block 32..119; pad 120..127; records 128..);
  durations/counts, state-machine rows ("snapshot gains + pot, write
  TRACE_HEADER_SIZE header", "overwrite from slot TRACE_DATA_BASE"), disarm
  commit length, dump strlen (TRACE_HEADER_SIZE). All kit copies +
  `../gitUAVXGS` re-synced.

## 2. Options considered / rejected (rationale)
- **Raw pot position instead of decoded scale**: rejected — the GCS would need
  the decode formula (and the `ActiveCh` gating) replicated, and the header
  would need two values (pot + active-ness) to be unambiguous. The decoded
  multiplier is the value the controller actually applied; one u16.
- **Snapshot at commit/live-read instead of capture-time**: rejected per Greg
  — the ground pot-reset workflow would corrupt the committed flash copy. Hence
  globals snapshot at capture-open, serialised at every header write.
- **No legacy marker**: "dubious about needing a legacy marker" → the 0 code
  doubles as the empty/legacy sentinel. v1 files on disk parse via the version
  branch regardless.
- **Grow header inline vs count-guarded block**: count u16 chosen so a future
  gain-set extension changes only the count, and a mis-read can't silently
  shift the record region (GCS treats unexpected count as absent block).
- **21 f32 vs u16-ish fixed point for gains**: f32 matches the FC native
  storage (`A[a].R.Kp` is `real32`), zero conversion loss, 84 B for all 7 gains
  × 3 axes — "probably minor" space, as Greg judged.

## 3. Verification
- FC: `python3 UAVXArmQ/scripts/fc_build.py` — **all 7 targets build clean**
  (UAVXF4V3, UAVXF4V4, DEVEBOXF4, SPEEDYBEEF405WING, FLYINGRCF4WINGMINI,
  BLUEBERRYF405, MATEKF411WING).
- GCS: `py_compile` clean on `trace_viewer.py` + `generate_trace_samples.py`.
- Headless round-trip (PyQt stubbed): v2 pot codes 1000/250/4000 → scales
  1.0/0.25/4.0; v2 gain block decoded with f32-exact values; v1 blob (32 B,
  records at 32) parses with `gains=None, scale=None`; truncation, too-short,
  bad-magic, and bad-version all `ValueError` correctly.
- Samples regenerated; `HEX files`/kits/Git mirror byte-identical (md5 verified
  for `trace_viewer.py`, AGENTS.md across source + 3 kits + git mirror).

## 4. Outstanding / next
- **Greg: hand-fly the workshop validation** — load `SPEEDYBEEF405WINGQ_r0.bin`
  (or the target board of choice; flash artifact is the v2 firmware), set ch8
  trace go-ahead, run a rate-probe capture, disarm, dump; expect: header
  `version=2`, `rate_gain_code` = whatever the Ch10 pot was (250–4000), a full
  21-float gain block matching the flown `.af` (R.kp/angle QKp etc.), and the
  existing critic trailer still present.
- Decide whether `StartBBChunkDump()`'s `eBBNone` zero-header grows to
  `TRACE_HEADER_SIZE` — **resolved**: the `eBBNone` branch zero-fills the whole
  `BBChunkHdr[TRACE_HEADER_SIZE]` (now 128 B), and the GCS no-capture detection
  in `_finalize_bb_dump` was widened from `len==32` to any all-zero dump
  (`not any(data) and len >= 32`), so the empty-dump path reports cleanly for
  both v1 (32) and v2 (128) transports.
- Kit/push: `master_update.sh` (host Merlin) not run this session — source,
  kits and git mirror are in sync locally; the SVN commit + GitHub push + PDF
  are the user's on-host step.