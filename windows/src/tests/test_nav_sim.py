#!/usr/bin/env python3
"""
Nav position/velocity cascade simulation — UAVXArmQ.

Models the outer-loop navigation controller as it runs in the FC, to check
stability with a slow / lagged GPS (5 Hz typical, 10 Hz rare):

  Outer loop (runs at GPS rate, NavdT = GPS period):
    PosE   = DesPos - Pos_meas
    DesVel = Limit1(PosKp*PosE + I, MaxVel)      I += PosKi*PosE*NavdT
    VelE   = DesVel - Vel_meas
    Corr   = VelKp*VelE                          (bank-angle command, rad)

  Inner loop (attitude, modelled as a fast 1st-order bank tracking):
    bank -> accel = g*tan(bank) -> vel -> pos

  GPS:
    Pos_meas = true pos delayed by transport lag tau
    Vel_meas = LPF(Vel_meas, true vel, GPSVelLPFK)   (~1 Hz, our new filter)

Goal: confirm the current gains (PosKp=0.15, PosKi=0.012, VelKp=0.2,
MaxVel=6) hold position without oscillation/limit-cycle at 5 Hz and 10 Hz.

Usage:
  python3 src/tests/test_nav_sim.py
"""

import math

GRAVITY = 9.80665
DEG = math.pi / 180.0

# ── Current Nav gains (fc values from ParamTable) ──
NAV_POS_KP = 0.15
NAV_POS_KI = 0.012
NAV_VEL_KP = 0.20
NAV_MAX_VEL = 6.0          # m/s (EcksTuned: NAV_POS_INT_LIMIT=6)
MAX_BANK = 30.0 * DEG      # A[a].P.Max
ATT_BANK_LAG = 0.04        # s — fast inner-loop bank tracking (conservative)

# ── GPS ──
GPS_HZ = 5                 # primary case
GPS_TAU = 0.15             # s — transport lag of position measurement
VEL_LPF_HZ = 1.0           # our new velocity filter cutoff


def lpf1(prev, new, k):
    return prev + (new - prev) * k


def lpf1_coeff(cut_hz, dt):
    return dt / (1.0 / (2.0 * math.pi * cut_hz) + dt)


def sim(gps_hz, gps_tau, vel_lpf_hz, pos_error0, duration=60.0, plant_dt=0.001,
        wind_start=10.0, wind_vel=0.0):
    gps_dt = 1.0 / gps_hz
    vel_k = lpf1_coeff(vel_lpf_hz, gps_dt)

    pos = pos_error0
    vel = 0.0
    bank = 0.0
    I = 0.0
    vel_meas = 0.0
    bank_cmd = 0.0

    delay_steps = max(1, int(round(gps_tau / plant_dt)))
    pos_hist = [0.0] * (delay_steps + 1)
    hist_idx = 0

    t = 0.0
    next_gps = gps_dt

    peak_pos = abs(pos)
    peak_vel = 0.0
    crossings = 0
    prev_err = pos
    crossed = False
    final_pos = pos

    n = int(duration / plant_dt)
    for i in range(n):
        wind = wind_vel if t >= wind_start else 0.0

        if t >= next_gps - 1e-9:
            pos_meas = pos_hist[hist_idx]
            vel_meas = lpf1(vel_meas, vel, vel_k)

            pos_e = 0.0 - pos_meas
            des_vel = NAV_POS_KP * pos_e + I
            if des_vel > NAV_MAX_VEL:
                des_vel = NAV_MAX_VEL
            elif des_vel < -NAV_MAX_VEL:
                des_vel = -NAV_MAX_VEL
            else:
                I += NAV_POS_KI * pos_e * gps_dt
            vel_e = des_vel - vel_meas
            corr = NAV_VEL_KP * vel_e
            if corr > MAX_BANK:
                corr = MAX_BANK
            elif corr < -MAX_BANK:
                corr = -MAX_BANK
            bank_cmd = corr
            next_gps += gps_dt

        bank += (bank_cmd - bank) * min(1.0, plant_dt / ATT_BANK_LAG)
        accel = GRAVITY * math.tan(bank)
        vel += accel * plant_dt
        vel -= wind * plant_dt          # wind pushes aircraft (disturbance on velocity)
        pos += vel * plant_dt

        pos_hist[hist_idx] = pos
        hist_idx = (hist_idx + 1) % len(pos_hist)

        a = abs(pos)
        peak_pos = max(peak_pos, a)
        peak_vel = max(peak_vel, abs(vel))
        if t > 2.0:
            if prev_err * pos < 0 and not crossed:
                crossings += 1
                crossed = True
            elif prev_err * pos >= 0:
                crossed = False
        prev_err = pos
        final_pos = pos

        t += plant_dt

    return {
        "gps_hz": gps_hz,
        "final_pos": final_pos,
        "peak_pos": peak_pos,
        "peak_vel": peak_vel,
        "crossings": crossings,
    }


def run_case(label, **kw):
    r = sim(**kw)
    osc = r["crossings"]
    bounded = r["peak_pos"] < 60
    flag = "STABLE" if bounded else "DIVERGE"
    o = "OK" if osc <= 3 else "OSC"
    print(f"  {label:<28} final={r['final_pos']:+7.2f}m peak={r['peak_pos']:6.2f}m "
          f"peakV={r['peak_vel']:5.2f} osc={osc:2d}  {flag} {o}")


def main():
    global NAV_POS_KP, NAV_VEL_KP
    print("Nav cascade stability — current gains "
          f"(PosKp={NAV_POS_KP}, PosKi={NAV_POS_KI}, VelKp={NAV_VEL_KP}, MaxVel={NAV_MAX_VEL})")
    print(f"Attitude lag={ATT_BANK_LAG*1000:.0f}ms, GPS vel LPF={VEL_LPF_HZ}Hz\n")

    print("A) Position-step recovery (start 15 m off, no wind):")
    for hz, tau in [(5, 0.15), (5, 0.30), (10, 0.15), (10, 0.30)]:
        run_case(f"{hz}Hz lag{int(tau*1000)}ms", gps_hz=hz, gps_tau=tau,
                 vel_lpf_hz=VEL_LPF_HZ, pos_error0=15.0)

    print("\nB) Constant wind disturbance (2 m/s from t=10s, start on station):")
    for hz, tau in [(5, 0.15), (10, 0.15)]:
        run_case(f"{hz}Hz lag{int(tau*1000)}ms wind2", gps_hz=hz, gps_tau=tau,
                 vel_lpf_hz=VEL_LPF_HZ, pos_error0=0.0, wind_vel=2.0)

    print("\nC) Higher-gain stress (VelKp x2, PosKp x2) — margin to oscillation:")
    save_pk, save_vk = NAV_POS_KP, NAV_VEL_KP
    NAV_POS_KP, NAV_VEL_KP = 0.30, 0.40
    for hz, tau in [(5, 0.15), (10, 0.15)]:
        run_case(f"{hz}Hz lag{int(tau*1000)}ms", gps_hz=hz, gps_tau=tau,
                 vel_lpf_hz=VEL_LPF_HZ, pos_error0=15.0)
    NAV_POS_KP, NAV_VEL_KP = save_pk, save_vk


if __name__ == "__main__":
    main()
