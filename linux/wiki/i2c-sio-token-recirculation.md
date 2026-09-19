# Merits and Failings of the Recirculating `SIOTokenFree` I2C Arbitration

## What it is

`SIOTokenFree` (sio.c:23, a single global `boolean`) is a cooperative, non-queued
mutex for the shared I2C bus(es). It does not use the RTOS/SVC primitives, ISR
fences, or timeout locks of ordinary mutexes — instead it is a *recirculating
permission token*:

- **Claim**: a device driver transacts only when `(SIOTokenFree && <its timer
  due>)` is true, and immediately sets `SIOTokenFree = false` (e.g.
  `GetMagnetometer` hmc5xxx_mag.c:149, `GetBaro` ms56xx.c:234 / spl0601.c:140,
  now `UpdateAirspeed`/MS4525 as.c:131).
- **Release**: the main control loop sets `SIOTokenFree = true` once per frame
  (uavxarm-v3-gke.c:491) at the end of the control-cycle block, i.e. it
  *recirculates* the token every PID cycle regardless of who claimed it.
- **Partner guarantees**: whichever device's timer expired grabs the token; the
  others see it held and skip until the next frame. Init paths do a forced
  one-shot release (hmc5xxx_mag.c:198, altitude.c:76).

The token therefore serialises bus access to **at most one I2C transaction per
control frame** — it is a frame-rate gate, not a general-purpose lock.

## Merits

1. **Deterministic single-access-per-frame.** Because the token is only released
   once per PID cycle, two devices can never interleave mid-transaction on the
   shared bus — the failure mode that produced torn/mixed sensor bursts.
   This is the strongest property and directly serves the IMU data-integrity
   work: it bounds the blast radius of a wedged/peripheral race to one frame.

2. **Zero-cost, cooperative.** No RTOS syscalls, no disable-IRQ windows, no
   spin-lock tick stalls. Costs one boolean test + store per frame. It is
   auditable by inspection across ~3 call sites.

3. **Entropy-safe timers kept intact.** Each device keeps its own independent
   sampling timer (`mSTimeout`/`uSTimeout`); if its turn is stolen it simply
   waits for a later frame, so data stays fresh without any lock-owned fence.

4. **Composes with the recovery architecture.** Because acquisitions are short
   (one block read), a wedged bus can't be held across frames; the next release
   is ≤1 frame away and `InitI2C`/`UnstickI2C` recovery can always re-arm.

5. **Matches the cooperative, single-threaded control model.** The FC is one
   main loop + ISRs the loop itself drains, so a recirculating flag is a
   *sufficient* arbiter — it never needs to arbitrate between true concurrent
   threads.

## Failings

1. **Not ownership-earned: free release can violate mutual exclusion in edge
   cases.** The frame-end `SIOTokenFree = true` is unconditional. If a driver
   holds the token across the frame boundary (a slow device, an SDIO-style long
   read, or a future DMA-based sensor), the loop will set the token free *while
   the holder is still mid-transaction*, letting a second device start a burst.
   Today the acquisitions are short enough that this doesn't occur; it is an
   implicit, not enforced, assumption.

2. **No interruption/backpressure.** A device whose timer is due but straggles
   behind (e.g. a burst of bus errors keeps it from ever seeing the token free)
   will silently accumulate a stale gap — the token gives it no priority boost
   or fairness guarantee. Under sustained I2C errors the baro/mag can be starved
   for many frames while the IMU (bus-critical, outside the token) hogs the
   controller.

3. **Single global token, not per-bus.** All devices share one flag even though
   the STM32F4 has multiple I2C ports (`I2CState[MAX_I2C_PORTS]`). Two sensors
   on *different* buses are pointlessly serialised, which both wastes bandwidth
   and over-centralises the fault domain: one slow device stalls every bus.

4. **No ownership identity / no debugging hooks.** The flag never says *who*
   holds it, so a hung driver holding the token is invisible — the only
   symptom is other devices skipping frames. The newly-added tag-76 I2C
   counters help diagnose *transaction* errors but not *token starvation*.

5. **Recirculation defeats hand-off latency guarantees.** A device can only
   receive the token at frame cadence; for sub-frame-rate sensors this adds up
   to one full PID cycle of latency. Fine at 1 kHz, but it caps the achievable
   sample rate of any future high-rate sensor on the bus.

## Verdict and notes

- The token's core value is the **one-transaction-per-frame invariant**, which
  is exactly what the IMU/I2C integrity work needs. It is the right tool for
  this cooperative firmware.
- Its principal latent risk is the **unconditional recirculation** — if a future
  or existing driver is ever able to hold the token across the frame boundary,
  the exclusivity it promises silently disappears. That is the first thing to
  change (release must become owner-earned, per-bus) if a DMA/SPI/I2C-long
  sensor is added.
- The MS4525 airspeed change (as.c:129–137) follows the established
  claim-skip idiom and is safe precisely because its acquisition is a single
  short block read that never crosses a frame boundary.

*Prepared 2026-08-10. References: sio.c:23, hmc5xxx_mag.c:149/198,
ms56xx.c:234, spl0601.c:140, uavxarm-v3-gke.c:491, as.c:129, i2c.c.*
---

## Addendum 2026-08-20 — per-device SIO timing consolidated at the choke point

### Before
Each block driver measured (and EMAd-accumulated) its own transaction cost
inline: `I2CReadBlock`/`I2CWriteBlock` (i2c.c) and `SPIReadBlock`/`SPIWriteBlock`
(spi.c) each ran the same ~1/16 EMA + peak logic against `SIOTiming[]`, and
`SIOReadBlockataddr`'s I2C branch bypassed them (its own timings bypassed too).
The EMA also carried a `!FirstPass` gate whose rationale (exclude init/cal
delays) was contradicted by the fixed on-wire cost argument.

### After
- `sio.c` holds the **single** accumulator `SIOAccumTiming(sioDev, t0, ok)`,
  called by `SIOReadBlock`, `SIOWriteBlock`, and the I2C branch of
  `SIOReadBlockataddr`. Measured edge-to-edge at the SIO layer, so I2C and SPI
  devices are timed the same way.
- The four inline timing blocks (with the dead `!FirstPass` gate) were deleted;
  `i2c.c`/`spi.c` keep only the per-device `errors++` counts.
- `avgUs` is now a real32 1/2 running average (EMA "prime + ramp" and the
  signed-delta underflow handling are gone); `peakUs` unsigned stays as the
  long-tail frame-budget bound. The `ok` guard excludes only genuinely failed
  transactions (wedged bus busy-wait-timeout).
- i2c.h doc comment updated to match. Values stream unchanged in tag 76.
- Note: SPI writes at the slow clock (0.656 MHz vs 2.625 MHz read) can pin
  `peakUs` — accepted as a true worst case (a write is a real on-bus cost).

*Build checked (BUILD OK). Files: sio.c, i2c.c, spi.c, i2c.h, +sio.h.*
