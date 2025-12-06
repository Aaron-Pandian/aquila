"""
Scenario runner for Aquila.

This module defines a simple waypoint-following scenario using the
point-mass dynamics and sensor models. It writes a CSV file containing
true state and noisy measurements.
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import List

import numpy as np

from .dynamics import PointMassState, step_dynamics
from .sensors import simulate_imu, simulate_gps, simulate_baro

_REPO_ROOT = Path(__file__).resolve().parents[1]
LOGS_DIR = _REPO_ROOT / "logs"
SENSORS_LOG_PATH = LOGS_DIR / "sim_sensors.csv"


@dataclass
class Waypoint:
    x_m: float
    y_m: float


def _ensure_logs_dir() -> None:
    LOGS_DIR.mkdir(parents=True, exist_ok=True)


def _heading_to_waypoint(state: PointMassState, wp: Waypoint) -> float:
    dx = wp.x_m - state.x_m
    dy = wp.y_m - state.y_m
    return math.atan2(dy, dx)


def _wrap_angle(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def _simple_heading_controller(state: PointMassState, waypoint: Waypoint, max_heading_rate_radps: float = math.radians(15.0), k_heading: float = 1.0,) -> float:
    """
    Proportional heading controller to steer towards a waypoint.
    """
    psi_des = _heading_to_waypoint(state, waypoint)
    err = _wrap_angle(psi_des - state.psi_rad)
    heading_rate_cmd = k_heading * err
    return max(-max_heading_rate_radps, min(max_heading_rate_radps, heading_rate_cmd))


def _simple_speed_controller(state: PointMassState, v_des_mps: float = 15.0, k_speed: float = 1.0, max_accel_mps2: float = 2.0,) -> float:
    """
    Proportional speed controller.
    """
    err = v_des_mps - state.v_mps
    accel_cmd = k_speed * err
    return max(-max_accel_mps2, min(max_accel_mps2, accel_cmd))


def run_scenario() -> None:
    """
    Run a simple waypoint-following scenario and log:
    - time
    - true state (x, y, z, psi, v)
    - IMU, GPS, baro measurements

    The output is written to logs/sim_sensors.csv.
    """
    _ensure_logs_dir()

    # Simulation parameters
    dt_s = 0.05
    t_final_s = 60.0
    n_steps = int(t_final_s / dt_s)

    # Define a simple square waypoint pattern
    waypoints: List[Waypoint] = [
        Waypoint(0.0, 0.0),
        Waypoint(200.0, 0.0),
        Waypoint(200.0, 200.0),
        Waypoint(0.0, 200.0),
    ]
    wp_index = 0
    wp_reached_thresh_m = 10.0

    state = PointMassState()

    with SENSORS_LOG_PATH.open(mode="w", newline="") as f:
        writer = csv.writer(f)
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

        # Main simulation loop
        t = 0.0
        for _ in range(n_steps):
            # Select current waypoint
            wp = waypoints[wp_index]

            # Check if waypoint reached
            dist_to_wp = math.hypot(state.x_m - wp.x_m, state.y_m - wp.y_m)
            if dist_to_wp < wp_reached_thresh_m:
                wp_index = (wp_index + 1) % len(waypoints)
                wp = waypoints[wp_index]

            # Compute control commands
            heading_rate_cmd = _simple_heading_controller(state, wp)
            accel_cmd = _simple_speed_controller(state)

            # Step dynamics
            controls = {
                "heading_rate_cmd": heading_rate_cmd,
                "accel_cmd": accel_cmd,
            }
            state = step_dynamics(state, controls, dt_s)

            # Simulate sensors
            imu_meas = simulate_imu(state, dt_s)
            gps_meas = simulate_gps(state)
            baro_meas = simulate_baro(state)

            # Log everything
            writer.writerow(
                [
                    f"{t:.3f}",
                    f"{state.x_m:.3f}",
                    f"{state.y_m:.3f}",
                    f"{state.z_m:.3f}",
                    f"{state.psi_rad:.6f}",
                    f"{state.v_mps:.3f}",
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

            t += dt_s

    print(f"Wrote sensor log to {SENSORS_LOG_PATH}")


def main() -> None:
    run_scenario()

if __name__ == "__main__":
    main()