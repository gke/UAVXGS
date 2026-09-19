# Gyro Slew Limiter — Origin of the Concept

Mini report, 2026-08-09 (gke).

## Subject

The gyro **slew limiter** used in `UAVXArmQ/src/sensors/mpu6xxx.c` (and long
before that in Betaflight as `gyroSlewLimiter()`, `src/main/sensors/gyro.c`)
originated with a proposal posted by **gke** to the Betaflight GitHub issue
tracker in 2017. Its purpose: when the airframe strikes an obstacle (gate,
branch) the impact produces extreme accel/gyro readings that are not
physically achievable by the rotorcraft's own dynamics; such a sample should
be trapped rather than fed to the controller.

## Original post (betaflight/betaflight #3959)

> I think you will find that while the code does not check for the specific
> case it should find *impossible changes to the gyro values* and thus acts
> as a form of slew filter...
>
> I offer this merely as a contribution, with the encouragement of others who
> were concerned enough to PM me.

Followed in reply by the "gate/branch hit" motivation and the "newton
constant" reasoning:

> As you suggested elsewhere, it should reject shocks like gate/branch hits,
> possibly in conjunction with the accelerometer. The constant need not be
> set at the gyro limits. A threshold like 3200 would accommodate a huge
> turn, separate from the sample rate. You can set the "newton" constant to
> anything you like. Better to set it high to capture only the truly bizarre
> values, as there are already parameters for max commanded rate.

## Core idea (raw delta bound)

The mechanism: if the raw sample jumps by more than a physically possible
bound for one sample interval, the value is not a genuine measurement.
In Betaflight's merged implementation this became `gyroSlewLimiter()`, which
holds the previous sample when `|raw - rawPrev| > 2^14`.

## Timeline (Betaflight)

1. **gke** posts the slew-filter concept in #3959.
2. Maintainers pick it up: *"I think a slew filter is a useful addition.
   I've used your example as a basis for creating a slew filter,
   PR #3983."* (martinbudden)
3. After testing on a yaw-spin rig, z-axis application and per-axis guard
   merged via PR #4058 — *"Closing, fixed by PR #4058."*
4. Later regressions: the limiter only fires when the gyro runs in 32 kHz
   mode (wider-bandwidth DLPF); in 8 kHz mode the on-chip LPF smooths the
   overflow transition and the raw delta no longer exceeds `2^14`
   (see #4736). This motivated the alternate `gyro_overflow_detect`
   (rate-over-threshold) path that remains today.

## Where it is today in UAVXArmQ

Implementation in place: `IMU_RATE_SLEW_RAW` defined in
`sensors/mpu6xxx.h`, applied in `ReadIMUAccAndRate()` in
`sensors/mpu6xxx.c`, clamped rather than held (the estimator keeps tracking),
with a per-cycle reject counter shipped to the GCS (exec-time packet, tag 67).

## Was it a good idea? An assessment (2026-08-09)

Yes — the concept holds up. The later Betaflight "regressions" were a mix of
real bugs and of the project repurposing the idea for a problem it was never
built to catch.

**Real regressions (implementation, not concept):**
- #4407: the slew limiter silently wasn't compiled in for the ICM20608G
  because the `#ifdef USE_GYRO_SLEW_LIMITER` guard list omitted
  `USE_GYRO_SPI_MPU6500` (the ICM20608G is driven via the MPU6500 driver).
  A pure configuration bug, trivially fixed.
- #4736: in 8 kHz mode the chip's DLPF smooths the gyro-overflow transition
  across ~34 samples, so the per-sample delta never exceeds 2^14. Legit
  insight — but it exposes the limit of a *one-sample* delta bound against a
  slow, smoothed excursion, which is a different failure mode.

**Where the idea was misread:**
- The "newton constant" was specified **high**, specifically to reject only
  the physically impossible single-sample spikes (gate/branch-hit shocks),
  and to work in conjunction with the accelerometer. That is a *bounded
  delta, one-sample, physical-impossibility* filter — the correct design for
  impact rejection.
- Maintainer critiques ("fragile, only looks one sample apart, may hold an
  intermediate value") were all framed against the *overflow/sign-reversal*
  case. Fair there — but the concept never claimed to be an overflow detector;
  the project applied it to that problem (originally Z-axis only) while
  keeping the name. The "fragility" was a consequence of that misapplication,
  not of the original idea.
- The eventual overflow solution (`gyro_overflow_detect`,
  rate-over-threshold + PID zero) is a good, complementary mechanism — not a
  refutation. For a genuine impact spike it is wrong/slow; the slew clamp
  acts instantly.

**Why the UAVXArmQ use is closer to the original intent:**
- Runs on raw, unfiltered 1 kHz reads, so no DLPF-smoothed inversion exists
  to mask an impossible delta.
- The read is free-running-gated and the clamp holds/approaches (not hard
  reject), so the controller keeps tracking the truth.
- A gate/branch impact is a genuine single-sample impossibility – exactly the
  trigger described in 2017.

Verdict: for impact/shock rejection the idea was sound at release; the
regressions that followed pointed at overflow handling, not at the
concept itself.

*Note: quoted text above is reconstructed from the #3959 thread as retrieved
online; verify exact wording against the issue page before re-citing verbatim.*

## Citation note

Primary citations:
- gke (2017): "Gyro overflow causing high-speed yaw spin after crash."
  betaflight/betaflight issue #3959, GitHub.
- Merged implementation: betaflight PR #4058 (gyro slew limiter).
- UAVXArmQ adaptation dated 2026-08-09 in `src/sensors/mpu6xxx.c`.

The post is attributed to the pseudonym "gke" in the public thread; confirm
authorship/attribution against the raw issue discussion before reuse.