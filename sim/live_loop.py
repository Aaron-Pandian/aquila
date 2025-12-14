#!/usr/bin/env python3
import argparse
import csv
import os
import subprocess
import sys
import math
from typing import Dict, Any

import numpy as np

from sim.dynamics_6dof import SixDofAircraft, AircraftParams
import sim.sensors as sensors


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
    "t_s",
    "n_m", "e_m", "d_m",
    "psi_rad",
    "v_mps",
    "cmd_aileron", "cmd_elevator", "cmd_rudder", "cmd_throttle",
]


def quat_to_yaw(q: np.ndarray) -> float:
    """Yaw from quaternion [qw,qx,qy,qz]"""
    qw, qx, qy, qz = q
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(math.atan2(siny_cosp, cosy_cosp))


def rk4_step(model: SixDofAircraft, dt: float, x: np.ndarray, u: np.ndarray) -> np.ndarray:
    k1 = model.derivatives(dt, x, u)
    k2 = model.derivatives(dt, x + 0.5 * dt * k1, u)
    k3 = model.derivatives(dt, x + 0.5 * dt * k2, u)
    k4 = model.derivatives(dt, x + dt * k3, u)
    x_next = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    # Normalize quaternion
    q = x_next[IDX_Q0:IDX_Q0+4]
    qn = np.linalg.norm(q)
    if qn < 1e-9:
        x_next[IDX_Q0:IDX_Q0+4] = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        x_next[IDX_Q0:IDX_Q0+4] = q / qn
    return x_next


def launch_fsw(fsw_path: str) -> subprocess.Popen:
    return subprocess.Popen(
        [fsw_path, "--stream"],
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


def truth_to_sensor_row(t_s: float, x: np.ndarray, dt_s: float, rng: np.random.Generator) -> Dict[str, float]:
    pn = x[0:3]
    vb = x[3:6]
    q = x[6:10]
    omega_b = x[10:13]

    psi = quat_to_yaw(q)
    v_mps = float(np.linalg.norm(vb))
    alt_m = float(-pn[2])  # +up

    imu = {}
    gps = {}
    baro = {}

    # Try your sensors module if functions exist
    if hasattr(sensors, "measure_imu_from_state"):
        try:
            imu = sensors.measure_imu_from_state(dt_s=dt_s, x=x, rng=rng) or {}
        except Exception:
            imu = {}
    if hasattr(sensors, "measure_gps_from_state"):
        try:
            gps = sensors.measure_gps_from_state(dt_s=dt_s, x=x, rng=rng) or {}
        except Exception:
            gps = {}
    if hasattr(sensors, "measure_baro_from_state"):
        try:
            baro = sensors.measure_baro_from_state(x=x, rng=rng) or {}
        except Exception:
            baro = {}

    return {
        "t_s": float(t_s),
        "n_m": float(pn[0]),
        "e_m": float(pn[1]),
        "d_m": float(pn[2]),
        "psi_rad": float(psi),
        "v_mps": float(v_mps),

        "imu_ax_mps2": pick(imu, "imu_ax_mps2", "imu_ax", "ax_mps2", default=0.0),
        "imu_ay_mps2": pick(imu, "imu_ay_mps2", "imu_ay", "ay_mps2", default=0.0),
        "imu_az_mps2": pick(imu, "imu_az_mps2", "imu_az", "az_mps2", default=0.0),

        "imu_gx_radps": pick(imu, "imu_gx_radps", "imu_gx", "gx_radps", default=float(omega_b[0])),
        "imu_gy_radps": pick(imu, "imu_gy_radps", "imu_gy", "gy_radps", default=float(omega_b[1])),
        "imu_gz_radps": pick(imu, "imu_gz_radps", "imu_gz", "gz_radps", default=float(omega_b[2])),

        "gps_x_m": pick(gps, "gps_x_m", "gps_n_m", "x_m", default=float(pn[0])),
        "gps_y_m": pick(gps, "gps_y_m", "gps_e_m", "y_m", default=float(pn[1])),
        "gps_z_m": pick(gps, "gps_z_m", "gps_d_m", "z_m", default=float(pn[2])),
        "gps_vx_mps": pick(gps, "gps_vx_mps", "vx_mps", default=0.0),
        "gps_vy_mps": pick(gps, "gps_vy_mps", "vy_mps", default=0.0),
        "gps_vz_mps": pick(gps, "gps_vz_mps", "vz_mps", default=0.0),

        "baro_alt_m": pick(baro, "baro_alt_m", "alt_m", default=alt_m),
        "baro_pressure_pa": pick(baro, "baro_pressure_pa", "pressure_pa", default=101325.0),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fsw", required=True, help="Path to built FSW executable (e.g., ./build/fsw/aquila_fsw_main)")
    ap.add_argument("--dt", type=float, default=0.05)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--alt0", type=float, default=100.0)
    ap.add_argument("--speed0", type=float, default=15.0)
    ap.add_argument("--out_sensors", default="logs/sim_sensors.csv")
    ap.add_argument("--out_closedloop", default="logs/sim_closedloop.csv")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out_sensors), exist_ok=True)
    os.makedirs(os.path.dirname(args.out_closedloop), exist_ok=True)

    params = AircraftParams(
        mass_kg=8.0,
        Ixx=0.25, Iyy=0.30, Izz=0.45,
        S=0.8, b=2.0, c_bar=0.4,
        rho=1.225,
        T_max=50.0,
    )
    aircraft = SixDofAircraft(params)
    rng = np.random.default_rng(0)
    x = make_initial_state(args.alt0, args.speed0)

    proc = launch_fsw(args.fsw)
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
            sens_row = truth_to_sensor_row(t_s, x, args.dt, rng)
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
            cmd_a = float(cmd_a); cmd_e = float(cmd_e); cmd_r = float(cmd_r); cmd_t = float(cmd_t)

            # Apply mapping to plant (keep consistent with your earlier offline replay)
            u = np.array([
                np.clip(cmd_a, -1.0, 1.0),
                np.clip(-cmd_e, -1.0, 1.0),  
                np.clip(cmd_r, -1.0, 1.0),
                np.clip(cmd_t, 0.0, 1.0),
            ], dtype=float)

            x = rk4_step(aircraft, args.dt, x, u)

            # Closed-loop truth log + commands (this is what plot_open_vs_closed expects)
            pn = x[0:3]
            vb = x[3:6]
            q = x[6:10]
            psi = quat_to_yaw(q)
            v_mps = float(np.linalg.norm(vb))

            w_c.writerow({
                "t_s": t_s,
                "n_m": float(pn[0]),
                "e_m": float(pn[1]),
                "d_m": float(pn[2]),
                "psi_rad": float(psi),
                "v_mps": float(v_mps),
                "cmd_aileron": cmd_a,
                "cmd_elevator": cmd_e,
                "cmd_rudder": cmd_r,
                "cmd_throttle": cmd_t,
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