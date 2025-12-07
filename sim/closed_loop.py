"""
Closed-loop simulation driven by FSW commands.

This module replays actuator commands from logs/fsw_output.csv into the
point-mass dynamics, generating a new sensor log:

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

from .dynamics import PointMassState, step_dynamics
from .sensors import simulate_imu, simulate_gps, simulate_baro


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


def _map_commands_to_controls(cmd: FswCommandRow) -> Dict[str, float]:
    """
    Map FSW actuator commands to the simplified point-mass controls.

    - heading_rate_cmd [rad/s] is driven by aileron.
    - accel_cmd [m/s^2] is driven by throttle relative to a nominal value.

    These mappings are intentionally simple and centralized here so they
    can be refined later (e.g., with more realistic control laws or a
    proper lateral/longitudinal dynamics model).
    """
    # Max heading rate (rad/s) for full aileron deflection.
    max_heading_rate_radps = float(np.deg2rad(10.0))  # ≈ 0.175 rad/s
    heading_rate_cmd = max_heading_rate_radps * cmd.cmd_aileron

    # Map throttle deltas to longitudinal acceleration
    throttle_nominal = 0.6
    k_throttle = 3.0  # [m/s^2] per 1.0 throttle delta
    accel_cmd = k_throttle * (cmd.cmd_throttle - throttle_nominal)

    # Clamp to keep behavior near the open-loop scenario
    accel_cmd = float(np.clip(accel_cmd, -1.5, 1.5))

    return {
        "heading_rate_cmd": heading_rate_cmd,
        "accel_cmd": accel_cmd,
        "elevator_cmd": cmd.cmd_elevator,
    }


def _write_header(writer: csv.writer) -> None:
    """
    Write a header compatible with sim_sensors.csv so that existing
    plotting and analysis tools can be reused.
    """
    writer.writerow(
        [
            "t_s",
            "x_m",
            "y_m",
            "z_m",
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
        ]
    )


def run_closed_loop_from_fsw() -> None:
    """
    Replay FSW commands into the dynamics to generate a closed-loop trajectory.

    This uses:
    - logs/fsw_output.csv as the command source
    - step_dynamics() for state propagation
    - simulate_imu/gps/baro() for sensor generation

    Output:
    - logs/sim_closedloop.csv
    """
    _ensure_logs_dir()

    cmds = _load_fsw_commands(FSW_LOG_PATH)
    if not cmds:
        raise RuntimeError("No FSW commands loaded for closed-loop sim.")

    state = PointMassState()

    with CLOSED_LOOP_LOG_PATH.open("w", newline="") as f:
        writer = csv.writer(f)
        _write_header(writer)

        prev_t = None
        t = 0.0

        for row in cmds:
            if prev_t is None:
                dt_s = 0.0
            else:
                dt_s = row.t_s - prev_t
                if dt_s < 0.0:
                    dt_s = 0.0
            prev_t = row.t_s
            t = row.t_s

            controls = _map_commands_to_controls(row)
            state = step_dynamics(state, controls, dt_s)

            imu_meas = simulate_imu(state, dt_s)
            gps_meas = simulate_gps(state)
            baro_meas = simulate_baro(state)

            # Approximate heading and speed for logging (consistent with scenarios.py)
            vx = state.v_mps * np.cos(state.psi_rad)
            vy = state.v_mps * np.sin(state.psi_rad)
            v_mps = state.v_mps

            writer.writerow(
                [
                    f"{t:.3f}",
                    f"{state.x_m:.3f}",
                    f"{state.y_m:.3f}",
                    f"{state.z_m:.3f}",
                    f"{state.psi_rad:.6f}",
                    f"{v_mps:.3f}",
                    f"{imu_meas.accel_mps2[0]:.6f}",
                    f"{imu_meas.accel_mps2[1]:.6f}",
                    f"{imu_meas.accel_mps2[2]:.6f}",
                    f"{imu_meas.gyro_radps[0]:.6f}",
                    f"{imu_meas.gyro_radps[1]:.6f}",
                    f"{imu_meas.gyro_radps[2]:.6f}",
                    f"{gps_meas.position_ned_m[0]:.3f}",
                    f"{gps_meas.position_ned_m[1]:.3f}",
                    f"{gps_meas.position_ned_m[2]:.3f}",
                    f"{gps_meas.velocity_ned_mps[0]:.3f}",
                    f"{gps_meas.velocity_ned_mps[1]:.3f}",
                    f"{gps_meas.velocity_ned_mps[2]:.3f}",
                    f"{baro_meas.altitude_m:.3f}",
                ]
            )

    print(f"Wrote closed-loop sensor log to {CLOSED_LOOP_LOG_PATH}")


def main() -> None:
    run_closed_loop_from_fsw()


if __name__ == "__main__":
    main()