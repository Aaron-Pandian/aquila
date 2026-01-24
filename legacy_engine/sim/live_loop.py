#!/usr/bin/env python3
import argparse
import csv
import os
import subprocess
import sys
import math
from typing import Dict, Any, Tuple

import numpy as np

from sim.dynamics_6dof import SixDofAircraft, AircraftParams
import sim.sensors as sensors
from sim.trim import solve_longitudinal_trim

# -------------------------
# State layout (13D quaternion model)
# x = [pn(3), vb(3), q(4), omega_b(3)]
# -------------------------
IDX_PN0 = 0
IDX_VB0 = 3
IDX_Q0  = 6
IDX_OMG0 = 10


SENSOR_HEADER = [
    "t_s",
    "n_m", "e_m", "d_m",
    "psi_rad",
    "qw", "qx", "qy", "qz",
    "v_mps",
    "imu_ax_mps2", "imu_ay_mps2", "imu_az_mps2",
    "imu_gx_radps", "imu_gy_radps", "imu_gz_radps",
    "gps_x_m", "gps_y_m", "gps_z_m",
    "gps_vx_mps", "gps_vy_mps", "gps_vz_mps",
    "baro_alt_m",
    "baro_pressure_pa",
]

# Closed-loop log: include truth + commands so plot_open_vs_closed works
CLOSEDLOOP_HEADER = [
    "t_s", "n_m", "e_m", "d_m",
    "psi_rad", "theta_rad",
    "v_mps",
    "q_radps",
    "cmd_aileron", "cmd_elevator", "cmd_rudder", "cmd_throttle",
    "elevator_applied",
    "qw","qx","qy","qz",
    "pn","pe","pd",
    "vbx","vby","vbz",
    "p_radps","r_radps",
    "q_norm",
    "alpha_rad","beta_rad",
    "throttle_applied",
]


def quat_to_yaw(q: np.ndarray) -> float:
    """Yaw from quaternion [qw,qx,qy,qz]"""
    qw, qx, qy, qz = q
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(math.atan2(siny_cosp, cosy_cosp))


def quat_to_pitch(q: np.ndarray) -> float:
    """Pitch (theta) from quaternion [qw,qx,qy,qz]."""
    qw, qx, qy, qz = q
    # qy = -qy # theta positive = nose-up, consistent quaternion convention
    sinp = 2.0 * (qw*qy - qz*qx)
    sinp = max(-1.0, min(1.0, sinp))
    return float(math.asin(sinp))


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], dtype=float)


def alpha_beta_from_vb(vb: np.ndarray) -> tuple[float, float]:
    vbx, vby, vbz = vb
    V = math.sqrt(vbx*vbx + vby*vby + vbz*vbz)
    if V < 1e-6:
        return 0.0, 0.0

    # body z positive DOWN convention:
    alpha = math.atan2(-vbz, vbx)  # nose-up alpha positive
    beta = math.asin(max(-1.0, min(1.0, vby / V)))
    return float(alpha), float(beta)


def _normalize_quat_in_state(x: np.ndarray) -> np.ndarray:
    x2 = x.copy()
    q = x2[IDX_Q0:IDX_Q0+4]
    qn = np.linalg.norm(q)
    if (not np.isfinite(qn)) or qn < 1e-12:
        x2[IDX_Q0:IDX_Q0+4] = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    else:
        x2[IDX_Q0:IDX_Q0+4] = q / qn
    return x2


def rk4_step(model: SixDofAircraft, dt: float, x: np.ndarray, u: np.ndarray) -> np.ndarray:
    x = _normalize_quat_in_state(x)

    k1 = model.derivatives(dt, x, u)

    x2 = _normalize_quat_in_state(x + 0.5 * dt * k1)
    k2 = model.derivatives(dt, x2, u)

    x3 = _normalize_quat_in_state(x + 0.5 * dt * k2)
    k3 = model.derivatives(dt, x3, u)

    x4 = _normalize_quat_in_state(x + dt * k3)
    k4 = model.derivatives(dt, x4, u)

    x_next = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    x_next = _normalize_quat_in_state(x_next)
    return x_next


def launch_fsw(fsw_path: str, elevator_trim: float) -> subprocess.Popen:
    return subprocess.Popen(
        [fsw_path, "--stream",],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=sys.stderr,  # do NOT mix diagnostics into stdout
        text=True,
        bufsize=1,
    )


def make_initial_state(alt_m: float, speed_mps: float) -> np.ndarray:
    """
    NED: D positive down. For +altitude, D should be negative.
    """
    x = np.zeros(13, dtype=float)
    x[0] = 0.0
    x[1] = 0.0
    x[2] = -alt_m  # D
    x[3] = speed_mps
    x[4] = 0.0
    x[5] = 0.0
    x[6:10] = np.array([1.0, 0.0, 0.0, 0.0])  # level quaternion
    x[10:13] = 0.0
    return x


def pick(d: Dict[str, Any], *keys, default=0.0) -> float:
    for k in keys:
        if k in d and d[k] is not None and d[k] != "":
            return float(d[k])
    return float(default)


def truth_to_sensor_row(
    t_s: float,
    x: np.ndarray,
    x_prev: np.ndarray,
    dt_s: float,
    rng: np.random.Generator,
    imu_bias: Dict[str, np.ndarray],
    imu_params: sensors.ImuParams,
    gps_params: sensors.GpsParams,
    baro_params: sensors.BaroParams,
) -> Tuple[Dict[str, float], Dict[str, np.ndarray]]:

    pn = x[0:3]
    vb = x[3:6]
    q = x[6:10]
    # normalize defensively before logging
    qn = float(np.linalg.norm(q))
    if qn < 1e-9:
        q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    else:
        q = q / qn
    omega_b = x[10:13]

    psi = quat_to_yaw(q)
    v_mps = float(np.linalg.norm(vb))
    alt_m = float(-pn[2])  # +up

    imu_meas, imu_bias = sensors.measure_imu_from_state(
        x=x,
        x_prev=x_prev,
        dt=dt_s,
        imu_bias=imu_bias,
        imu_params=imu_params,
        rng=rng,
    )

    gps_meas = sensors.measure_gps_from_state(
        x=x,
        gps_params=gps_params,
        rng=rng,
    )

    baro_meas = sensors.measure_baro_from_state(
        x=x,
        baro_params=baro_params,
        rng=rng,
    )

    row = {
        "t_s": float(t_s),
        "n_m": float(pn[0]),
        "e_m": float(pn[1]),
        "d_m": float(pn[2]),
        "psi_rad": float(psi),

        "qw": float(q[0]),
        "qx": float(q[1]),
        "qy": float(q[2]),
        "qz": float(q[3]),

        "v_mps": float(v_mps),

        "imu_ax_mps2": float(imu_meas["imu_ax"]),
        "imu_ay_mps2": float(imu_meas["imu_ay"]),
        "imu_az_mps2": float(imu_meas["imu_az"]),

        "imu_gx_radps": float(imu_meas["imu_gx"]),
        "imu_gy_radps": float(imu_meas["imu_gy"]),
        "imu_gz_radps": float(imu_meas["imu_gz"]),

        "gps_x_m": float(gps_meas["gps_n"]),
        "gps_y_m": float(gps_meas["gps_e"]),
        "gps_z_m": float(gps_meas["gps_d"]),
        "gps_vx_mps": float(gps_meas["gps_vn"]),
        "gps_vy_mps": float(gps_meas["gps_ve"]),
        "gps_vz_mps": float(gps_meas["gps_vd"]),

        "baro_alt_m": float(baro_meas["baro_alt"]),
        "baro_pressure_pa": float(baro_meas["baro_p"]),
    }

    return row, imu_bias


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fsw", required=True, help="Path to built FSW executable (e.g., ./build/fsw/aquila_fsw_main)")
    ap.add_argument("--dt", type=float, default=0.05)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--alt0", type=float, default=100.0)
    ap.add_argument("--speed0", type=float, default=15.0) # In controller.cpp and modes.hpp
    ap.add_argument("--out_sensors", default="logs/sim_sensors.csv")
    ap.add_argument("--out_closedloop", default="logs/sim_closedloop.csv")
    ap.add_argument("--elev_step", type=float, default=0.0, help="If nonzero, apply an elevator step (plant input) in radians.")
    ap.add_argument("--elev_step_t0", type=float, default=2.0)
    ap.add_argument("--elev_step_dt", type=float, default=1.0)
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out_sensors), exist_ok=True)
    os.makedirs(os.path.dirname(args.out_closedloop), exist_ok=True)
    
    params = AircraftParams(
        mass_kg=8.0,
        Ixx=1.2, Iyy=2.2, Izz=3.0,   
        S=0.8, b=2.0, c_bar=0.4,
        rho=1.225,
        T_max=50.0,
    )
    aircraft = SixDofAircraft(params)
    rng = np.random.default_rng(0)
    x = make_initial_state(args.alt0, args.speed0)

    # Trim the aircraft to the desired initial state
    theta_trim, elevator_trim, throttle_trim = solve_longitudinal_trim(
        aircraft, x,
        target_altitude=args.alt0,
        target_speed=args.speed0,
        throttle0=0.2,
    )

    # Apply trimmed pitch to initial quaternion
    # Kept solve_longitudinal_trim() only to initialize the starting attitude/velocity, so we start near reasonable AoA.
    x[IDX_Q0:IDX_Q0+4] = np.array([
        np.cos(theta_trim / 2.0),   # qw
        0.0,                        # qx
        -np.sin(theta_trim / 2.0),  # qy, theta positive = nose-up
        0.0                         # qz
    ], dtype=float)

    # Set initial flight path level in NED (no climb/descent):
    V = args.speed0
    v_ned_level = np.array([V, 0.0, 0.0], dtype=float)  
    Cnb = SixDofAircraft.quat_to_dcm(x[IDX_Q0:IDX_Q0+4]) # body -> NED
    Cbn = Cnb.T # NED -> body
    x[IDX_VB0:IDX_VB0+3] = Cbn @ v_ned_level 

    # --- Trim Sanity Check ---
    u_trim = np.array([0.0, elevator_trim, 0.0, throttle_trim], dtype=float)
    xdot = aircraft.derivatives(args.dt, x, u_trim)

    vb_dot = xdot[IDX_VB0:IDX_VB0+3]
    omega_dot = xdot[IDX_OMG0:IDX_OMG0+3]
    pn_dot = xdot[0:3]                           

    alpha0, beta0 = alpha_beta_from_vb(x[IDX_VB0:IDX_VB0+3])
    theta0 = math.asin(max(-1.0, min(1.0, -Cnb[2,0])))

    print("\n--- Trim sanity check ---")
    print(f"theta_trim (solver): {theta_trim:+.6f} rad")
    print(f"theta0  (from q):    {theta0:+.6f} rad")
    print(f"alpha0 (from vb):    {alpha0:+.6f} rad")
    print(f"elev_trim:           {elevator_trim:+.6f} rad")
    print(f"thr_trim:            {throttle_trim:+.6f}")
    print(f"vb_dot:              [{vb_dot[0]:+.4f}, {vb_dot[1]:+.4f}, {vb_dot[2]:+.4f}] m/s^2")
    print(f"omega_dot:           [{omega_dot[0]:+.4f}, {omega_dot[1]:+.4f}, {omega_dot[2]:+.4f}] rad/s^2")
    print(f"pn_dot: [{pn_dot[0]:+.4f}, {pn_dot[1]:+.4f}, {pn_dot[2]:+.4f}] m/s")

    imu_params = sensors.ImuParams()
    gps_params = sensors.GpsParams()
    baro_params = sensors.BaroParams()

    imu_bias = {"accel": np.zeros(3), "gyro": np.zeros(3)}
    x_prev = x.copy()

    proc = launch_fsw(args.fsw, elevator_trim)
    assert proc.stdin and proc.stdout

    # Send sensor header (FSW expects this)
    proc.stdin.write(",".join(SENSOR_HEADER) + "\n")
    proc.stdin.flush()

    # Read FSW output header once (stream output)
    _ = proc.stdout.readline()

    with open(args.out_sensors, "w", newline="") as f_s, open(args.out_closedloop, "w", newline="") as f_c:
        w_s = csv.DictWriter(f_s, fieldnames=SENSOR_HEADER)
        w_s.writeheader()

        w_c = csv.DictWriter(f_c, fieldnames=CLOSEDLOOP_HEADER)
        w_c.writeheader()

        t_s = 0.0
        n_steps = int(args.duration / args.dt)

        for _k in range(n_steps):
            # Build sensor row
            sens_row, imu_bias = truth_to_sensor_row(
                t_s=t_s,
                x=x,
                x_prev=x_prev,
                dt_s=args.dt,
                rng=rng,
                imu_bias=imu_bias,
                imu_params=imu_params,
                gps_params=gps_params,
                baro_params=baro_params,
            )
            x_prev = x.copy()
            w_s.writerow(sens_row)

            # Feed to FSW
            proc.stdin.write(",".join(str(sens_row[h]) for h in SENSOR_HEADER) + "\n")
            proc.stdin.flush()

            # Read command from FSW
            line = proc.stdout.readline()
            if not line:
                raise RuntimeError("FSW exited or produced no output.")

            parts = line.strip().split(",")
            if len(parts) < 5:
                raise RuntimeError(f"Bad FSW output line: {line!r}")

            _, cmd_a, cmd_e, cmd_r, cmd_t = parts[:5]
            # Simple for current tests on longitudinal dynamics
            cmd_a = float(cmd_a)
            cmd_e = float(cmd_e) # already radians from FSW
            cmd_r = float(cmd_r)
            cmd_t = float(cmd_t)

            # Apply mapping to plant (PLANT expects normalized surface cmds in [-1,1])
            ELEV_MAX = np.deg2rad(aircraft.p.de_max_deg)
            AIL_MAX  = np.deg2rad(aircraft.p.da_max_deg)
            RUD_MAX  = np.deg2rad(aircraft.p.dr_max_deg)

            #--- Elevator rate limiting params ---
            cmd_e = float(np.clip(cmd_e, -ELEV_MAX, ELEV_MAX))

            u = np.array([
                0.0,                                         # aileron disabled
                cmd_e,                                   # elevator in radians
                0.0,                                         # rudder disabled
                float(np.clip(cmd_t, 0.0, 1.0)),             # throttle 0..1
            ], dtype=float)
            
            # Optional elevator step test applied to the PLANT input
            if args.elev_step != 0.0 and (args.elev_step_t0 <= t_s < args.elev_step_t0 + args.elev_step_dt):
                u[1] = float(np.clip(args.elev_step, -1.0, 1.0))

            # Propagate state
            x = rk4_step(aircraft, args.dt, x, u)

            # Closed-loop truth  
            pn = x[0:3]
            vb = x[3:6]
            q = x[6:10]
            omega_b = x[IDX_OMG0:IDX_OMG0+3]

            # Further values for debugging
            psi = quat_to_yaw(q)
            v_mps = float(np.linalg.norm(vb))
            theta = quat_to_pitch(q)
            elevator_applied = float(u[1])   

            p_radps = float(omega_b[0])
            q_radps = float(omega_b[1])
            q_norm = float(np.linalg.norm(q))
            r_radps = float(omega_b[2])

            alpha, beta = alpha_beta_from_vb(vb)

            w_c.writerow({
                "t_s": t_s,
                "n_m": float(pn[0]),
                "e_m": float(pn[1]),
                "d_m": float(pn[2]),
                "psi_rad": float(psi),
                "theta_rad": float(theta),
                "v_mps": float(v_mps),
                "q_radps": float(q_radps),
                "cmd_aileron": cmd_a,
                "cmd_elevator": cmd_e,
                "cmd_rudder": cmd_r,
                "cmd_throttle": cmd_t,
                "elevator_applied": float(elevator_applied),
                "qw": float(q[0]), "qx": float(q[1]), "qy": float(q[2]), "qz": float(q[3]),
                "pn": float(pn[0]), "pe": float(pn[1]), "pd": float(pn[2]),
                "vbx": float(vb[0]), "vby": float(vb[1]), "vbz": float(vb[2]),
                "p_radps": float(p_radps),
                "r_radps": float(r_radps),
                "q_norm": float(q_norm),
                "alpha_rad": float(alpha),
                "beta_rad": float(beta),
                "throttle_applied": float(u[3]),
            })

            t_s += args.dt

    # --- Clean shutdown: send EOF so FSW finishes its loop and closes fsw_output.csv cleanly ---
    try:
        if proc.stdin:
            proc.stdin.flush()
            proc.stdin.close()  
    except Exception:
        pass

    # Give FSW a moment to exit on its own
    try:
        proc.wait(timeout=2.0)
    except Exception:
        # If it didn't exit, fall back to terminate/kill
        try:
            proc.terminate()
            proc.wait(timeout=2.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


if __name__ == "__main__":
    main()