"""
Closed-loop simulation driven by FSW commands.

This module replays actuator commands from logs/fsw_output.csv into the
6-DOF rigid-body dynamics, generating a new sensor log:

    logs/sim_closedloop.csv

This is the "offline closed-loop" variant:
- Python sim: plant + sensors
- C++ FSW: estimator + controller
- This module: plant driven by FSW commands
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np

from sim.dynamics_6dof import SixDofAircraft, AircraftParams
from .sensors import (measure_imu_from_state, measure_gps_from_state, measure_baro_from_state, ImuParams, GpsParams, BaroParams)

# Resolve repo root and log paths
_REPO_ROOT = Path(__file__).resolve().parents[1]
LOGS_DIR = _REPO_ROOT / "logs"
FSW_LOG_PATH = LOGS_DIR / "fsw_output.csv"
CLOSED_LOOP_LOG_PATH = LOGS_DIR / "sim_closedloop.csv"


@dataclass
class FswCommandRow:
    t_s: float
    cmd_aileron: float
    cmd_elevator: float
    cmd_rudder: float
    cmd_throttle: float


def _ensure_logs_dir() -> None:
    LOGS_DIR.mkdir(parents=True, exist_ok=True)


def _load_fsw_commands(path: Path) -> List[FswCommandRow]:
    """
    Load FSW output log and extract time + actuator commands.

    Assumes fsw_output.csv has the header written by CsvLogger with
    fields including: t, cmd_aileron, cmd_elevator, cmd_rudder, cmd_throttle.
    """
    if not path.exists():
        raise FileNotFoundError(f"FSW log file not found: {path}")

    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows: List[Dict[str, str]] = list(reader)

    if not rows:
        raise ValueError(f"FSW log file is empty: {path}")

    cmds: List[FswCommandRow] = []
    for r in rows:
        cmds.append(
            FswCommandRow(
                t_s=float(r["t"]),
                cmd_aileron=float(r["cmd_aileron"]),
                cmd_elevator=float(r["cmd_elevator"]),
                cmd_rudder=float(r["cmd_rudder"]),
                cmd_throttle=float(r["cmd_throttle"]),
            )
        )
    return cmds


def _write_header(writer: csv.writer) -> None:
    """
    Write a header compatible with sim_sensors.csv so that existing
    plotting and analysis tools can be reused.
    """
    writer.writerow(
        [
            "t_s",
            "n_m",
            "e_m",
            "d_m",
            "psi_rad",
            "v_mps",
            "imu_ax_mps2",
            "imu_ay_mps2",
            "imu_az_mps2",
            "imu_gx_radps",
            "imu_gy_radps",
            "imu_gz_radps",
            "gps_x_m",
            "gps_y_m",
            "gps_z_m",
            "gps_vx_mps",
            "gps_vy_mps",
            "gps_vz_mps",
            "baro_alt_m",
            "baro_pressure_pa",
        ]
    )


def _initial_state_6dof() -> np.ndarray:
    """
    Same initial state as scenarios.py:
    N,E,D = (0,0,-100), forward flight 15 m/s, level attitude, zero rates.
    """
    p_ned = np.array([0.0, 0.0, -100.0])
    vb = np.array([15.0, 0.0, 0.0])
    q = np.array([1.0, 0.0, 0.0, 0.0])
    omega_b = np.zeros(3)
    return np.concatenate([p_ned, vb, q, omega_b])


def rk4_step(model, t, x, u, dt):
    k1 = model.derivatives(t, x, u)
    k2 = model.derivatives(t + 0.5*dt, x + 0.5*dt*k1, u)
    k3 = model.derivatives(t + 0.5*dt, x + 0.5*dt*k2, u)
    k4 = model.derivatives(t + dt, x + dt*k3, u)
    return x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)


def run_closed_loop_from_fsw() -> None:
    """
    Replay FSW commands into the 6-DOF dynamics to generate a closed-loop trajectory.

    This uses:
    - logs/fsw_output.csv as the command source
    - SixDofAircraft.derivatives() for state propagation
    - measure_imu_from_state / measure_gps_from_state / measure_baro_from_state
      for sensor generation

    Output:
    - logs/sim_closedloop.csv
    """
    _ensure_logs_dir()

    cmds = _load_fsw_commands(FSW_LOG_PATH)
    if not cmds:
        raise RuntimeError("No FSW commands loaded for closed-loop sim.")

    # Aircraft model (same as scenarios.py)
    params = AircraftParams(
        mass_kg=8.0,
        Ixx=0.25,
        Iyy=0.30,
        Izz=0.45,
        S=0.8,
        b=2.0,
        c_bar=0.4,
        rho=1.225,
        T_max=50.0,
    )
    model = SixDofAircraft(params)

    # Initial state
    state = _initial_state_6dof()
    state_prev = state.copy()

    # Sensor models
    rng = np.random.default_rng(seed=1)
    imu_bias = {"accel": np.zeros(3), "gyro": np.zeros(3)}
    imu_params = ImuParams()
    gps_params = GpsParams()
    baro_params = BaroParams()

    with CLOSED_LOOP_LOG_PATH.open("w", newline="") as f:
        writer = csv.writer(f)
        _write_header(writer)

        prev_t = None

        for row in cmds:
            if prev_t is None:
                # ensure matches scenarios.py
                dt_s = 0.05
            else:
                dt_s = max(1e-3, row.t_s - prev_t)
            prev_t = row.t_s
            t = row.t_s

            # Use FSW commands directly as control inputs [da, de, dr, dt]
            controls = np.array(
                [
                    row.cmd_aileron,
                    -row.cmd_elevator,
                    row.cmd_rudder,
                    row.cmd_throttle,
                ],
                dtype=float,
            )

            # Integrate 6-DOF dynamics
            state = rk4_step(model, t, state, controls, dt_s)
            
            # Renormalize quaternion
            state[6:10] /= np.linalg.norm(state[6:10])

            # Sensors from state
            imu_meas, imu_bias = measure_imu_from_state(state, state_prev, dt_s, imu_bias, imu_params, rng)
            gps_meas = measure_gps_from_state(state, gps_params, rng)
            baro_meas = measure_baro_from_state(state, baro_params, rng)

            state_prev = state.copy()

            # Position
            N, E, D = state[0:3]

            # Velocity in NED
            v_n = gps_meas["gps_vn"]
            v_e = gps_meas["gps_ve"]
            v_mps = float(np.hypot(v_n, v_e))

            # Heading: use GPS track as approximation
            psi_rad = float(np.arctan2(v_e, v_n))

            writer.writerow(
                [
                    f"{t:.3f}",
                    f"{N:.3f}",
                    f"{E:.3f}",
                    f"{D:.3f}",
                    f"{psi_rad:.6f}",
                    f"{v_mps:.3f}",
                    f"{imu_meas['imu_ax']:.6f}",
                    f"{imu_meas['imu_ay']:.6f}",
                    f"{imu_meas['imu_az']:.6f}",
                    f"{imu_meas['imu_gx']:.6f}",
                    f"{imu_meas['imu_gy']:.6f}",
                    f"{imu_meas['imu_gz']:.6f}",
                    f"{gps_meas['gps_n']:.3f}",
                    f"{gps_meas['gps_e']:.3f}",
                    f"{gps_meas['gps_d']:.3f}",
                    f"{gps_meas['gps_vn']:.3f}",
                    f"{gps_meas['gps_ve']:.3f}",
                    f"{gps_meas['gps_vd']:.3f}",
                    f"{baro_meas['baro_alt']:.3f}",
                    f"{baro_meas['baro_p']:.3f}",
                ]
            )

    print(f"Wrote closed-loop sensor log to {CLOSED_LOOP_LOG_PATH}")


def main() -> None:
    run_closed_loop_from_fsw()


if __name__ == "__main__":
    main()