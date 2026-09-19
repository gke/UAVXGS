#!/usr/bin/env python3
"""
Nav bank-angle (attitude) shaping analysis — UAVXArmQ.

Question: which nav knob actually shapes tracking? Hypothesis to test — the bank
angle (VelKp -> DesiredNavCorr, capped by Nav.MaxBankAngle) is the limiter, not
Nav.MaxVelocity: with bank-limited acceleration (~g*tan(bank)), the demanded
velocity cap (6 m/s) rarely binds on a position-step / station-keep, while the
bank loop sets the reachable correction force budget.

Modelled inner loop -> attitude: bank_cmd from VelKp*VelE, clamped to
MaxBankAngle, 1st-order lag (ATT_BANK_LAG). accel = g*tan(bank).

Also models the GPS-accuracy gain conditioning from NavPI_P() (nav.c:259-267):
  GpsScale = 1.0 for hAcc <= cGpsHaccGood(2 m),
             linear fade to cNavGpsFloor(0.25) at cGpsMinHacc(5 m),
  applied to PosKp, PosKi, VelKp and NavMaxVel.

Gain units follow test_nav_sim.py convention (EcksTuned .af displayed values):
PosKp=0.15, PosKi=0.012, VelKp=0.2, MaxVel=6, bank 30 deg.

Usage:
  python3 src/tests/test_nav_bank_analysis.py
"""

import math

GRAVITY = 9.80665
DEG = math.pi / 180.0

NAV_POS_KP = 0.15
NAV_POS_KI = 0.012
NAV_VEL_KP = 0.20
NAV_MAX_VEL = 6.0
MAX_BANK = 30.0 * DEG
ATT_BANK_LAG = 0.04

GPS_HZ = 5
GPS_TAU = 0.15
VEL_LPF_HZ = 1.0

C_GPS_HACC_GOOD = 2.0
C_GPS_MIN_HACC = 5.0
C_NAV_GPS_FLOOR = 0.25


def lpf1(prev, new, k):
    return prev + (new - prev) * k


def lpf1_coeff(cut_hz, dt):
    return dt / (1.0 / (2.0 * math.pi * cut_hz) + dt)


def gps_scale(hacc):
    if hacc <= C_GPS_HACC_GOOD:
        return 1.0
    s = 1.0 - (hacc - C_GPS_HACC_GOOD) / (C_GPS_MIN_HACC - C_GPS_HACC_GOOD)
    return max(C_NAV_GPS_FLOOR, min(1.0, s))


ILIM_UNBOUNDED = float("inf")


def sim(pos_error0, duration=60.0, plant_dt=0.001, wind_start=10.0,
        wind_vel=0.0, bank_limit_deg=30.0, vel_kp=None, pos_kp=None,
        pos_ki=None, max_vel=None, hacc=1.5, ilim=ILIM_UNBOUNDED):
    vk = NAV_VEL_KP if vel_kp is None else vel_kp
    pk = NAV_POS_KP if pos_kp is None else pos_kp
    ki = NAV_POS_KI if pos_ki is None else pos_ki
    mv = NAV_MAX_VEL if max_vel is None else max_vel
    bank_lim = bank_limit_deg * DEG
    accel_budget = GRAVITY * math.tan(bank_lim)

    gps_dt = 1.0 / GPS_HZ
    vel_k = lpf1_coeff(VEL_LPF_HZ, gps_dt)
    gs = gps_scale(hacc)

    pos = pos_error0
    vel = 0.0
    bank = 0.0
    bank_meas = 0.0
    I = 0.0
    vel_meas = 0.0
    bank_cmd = 0.0

    delay_steps = max(1, int(round(GPS_TAU / plant_dt)))
    pos_hist = [0.0] * (delay_steps + 1)
    hist_idx = 0

    t = 0.0
    next_gps = gps_dt

    peak_bank = 0.0
    peak_vel = 0.0
    overshoot = 0.0
    last_past = 0.0
    crossed = False
    final_pos = pos

    n = int(duration / plant_dt)
    for i in range(n):
        wind = wind_vel if t >= wind_start else 0.0

        if t >= next_gps - 1e-9:
            pos_meas = pos_hist[hist_idx]
            vel_meas = lpf1(vel_meas, vel, vel_k)

            pos_e = 0.0 - pos_meas
            des_vel = pk * gs * pos_e + I
            if des_vel > mv * gs:
                des_vel = mv * gs
            elif des_vel < -mv * gs:
                des_vel = -mv * gs
            else:
                I = max(-ilim, min(ilim, I + ki * gs * pos_e * gps_dt))
            vel_e = des_vel - vel_meas
            corr = vk * gs * vel_e
            if corr > bank_lim:
                corr = bank_lim
            elif corr < -bank_lim:
                corr = -bank_lim
            bank_cmd = corr
            next_gps += gps_dt

        bank += (bank_cmd - bank) * min(1.0, plant_dt / ATT_BANK_LAG)
        bank_meas = max(bank_meas, abs(bank))
        accel = GRAVITY * math.tan(bank)
        vel += accel * plant_dt
        vel -= wind * plant_dt
        pos += vel * plant_dt

        pos_hist[hist_idx] = pos
        hist_idx = (hist_idx + 1) % len(pos_hist)

        peak_vel = max(peak_vel, abs(vel))
        if crossed:
            overshoot = min(overshoot, pos)
        elif pos < -1e-6:
            crossed = True
        if crossed and pos >= 0.5:
            pass
        final_pos = pos
        t += plant_dt

    return {
        "final_pos": final_pos,
        "overshoot_m": -overshoot,
        "peak_vel": peak_vel,
        "peak_bank_deg": math.degrees(bank_meas),
        "peak_accel_g": bank_meas and (GRAVITY * math.tan(bank_meas)) / GRAVITY,
        "accel_budget_g": accel_budget / GRAVITY,
        "vel_limit_bound": peak_vel >= mv * gs - 1e-6,
    }


def fmt(r):
    return (f"over={r['overshoot_m']:5.2f}m peakV={r['peak_vel']:5.2f} "
            f"peakBank={r['peak_bank_deg']:5.1f}deg "
            f"accel={r['peak_accel_g']:5.2f}g "
            f"flyLimit={r['accel_budget_g']:5.2f}g "
            f"VlimHit={r['vel_limit_bound']} final={r['final_pos']:+6.2f}m")


def main():
    print("Nav bank-angle shaping — gains PosKp=0.15 PosKi=0.012 VelKp=0.2 "
          "MaxVel=6")
    print("GPS 5Hz lag150ms; attitude lag 40ms\n")

    print("A) MaxBankAngle sweep (bank -> accel budget g*tan(bank)):")
    for b in (15, 20, 25, 30, 35):
        r = sim(15.0, bank_limit_deg=float(b))
        print(f"  bank {b:2d} deg  {fmt(r)}")

    print("\nB) VelKp sweep at 30 deg bank (gain margin vs overshoot):")
    for vk in (0.1, 0.2, 0.3, 0.4):
        r = sim(15.0, vel_kp=vk)
        print(f"  VelKp {vk:<4}  {fmt(r)}")

    print("\nC) Stronger wind, station-keep (5 m/s from t=10s, bank=30):")
    for w in (2.0, 5.0, 8.0):
        r = sim(0.0, wind_vel=w)
        print(f"  wind {w:4.1f} m/s  {fmt(r)}")

    print("\nD) GPS-accuracy conditioning (hAcc, GpsScale) — wind 2 m/s, "
          "still step 15 m:")
    for hacc in (1.5, 3.0, 4.0, 4.9):
        r = sim(15.0, wind_vel=2.0, hacc=hacc)
        print(f"  hAcc {hacc:4.1f}m scale={gps_scale(hacc):.2f}  {fmt(r)}")

    print("\nE) Position-integral clamp (Nav.PosIntLim) sweep, MaxVel=6, "
          "bank 30 deg:")
    for lbl, ilim in [("unbounded", ILIM_UNBOUNDED), ("2.0 m/s", 2.0),
                      ("1.0 m/s", 1.0), ("0.5 m/s", 0.5)]:
        r = sim(15.0, ilim=ilim)
        print(f"  Ilim {lbl:<11}  {fmt(r)}")

    print("\nF) Same clamp sweep under 2 m/s crosswind (station-keep):")
    for lbl, ilim in [("unbounded", ILIM_UNBOUNDED), ("2.0 m/s", 2.0),
                      ("1.0 m/s", 1.0), ("0.5 m/s", 0.5)]:
        r = sim(0.0, wind_vel=2.0, ilim=ilim)
        print(f"  Ilim {lbl:<11}  {fmt(r)}")


if __name__ == "__main__":
    main()