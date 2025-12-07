"""
Aquila dynamics module (3-DOF placeholder).

This implements a very simple point-mass model in the horizontal plane:
- State: x, y [m], heading psi [rad], speed V [m/s]
- Controls: heading_rate_cmd [rad/s], accel_cmd [m/s^2]

This will be upgraded to a full 6-DOF model later, but the interface will
remain compatible with the rest of the system.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
import math


@dataclass
class PointMassState:
    """Simple 3D kinematic state for initial prototyping.

    NED convention:
    - x: North, y: East, z: Down (altitude = -z)
    """

    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = -100.0  # start at 100 m altitude

    psi_rad: float = 0.0        # heading (yaw)
    gamma_rad: float = 0.0      # flight-path angle (rad, >0 = climb)
    v_mps: float = 15.0         # speed magnitude

    # Inertial acceleration in NED (for IMU modeling)
    ax_n_mps2: float = 0.0
    ay_n_mps2: float = 0.0
    az_n_mps2: float = 0.0

    # Yaw rate (for IMU gyro z)
    yaw_rate_radps: float = 0.0


def _vel_components(v_mps: float, gamma_rad: float, psi_rad: float):
    """Helper: convert speed + angles to NED velocity components."""
    vn = v_mps * math.cos(gamma_rad) * math.cos(psi_rad)
    ve = v_mps * math.cos(gamma_rad) * math.sin(psi_rad)
    vd = -v_mps * math.sin(gamma_rad)  # NED z (down positive)
    return vn, ve, vd


def step_dynamics(state: PointMassState, controls: Dict[str, float], dt_s: float,) -> PointMassState:
    """
    Propagate the simple 3D kinematic point-mass forward by dt_s.

    Controls:
    - "heading_rate_cmd": desired heading rate [rad/s]
    - "accel_cmd": desired tangential acceleration [m/s^2]
    - "elevator_cmd": dimensionless elevator command in [-1, 1]
      (optional; defaults to 0.0 if not provided)
    """
    if dt_s <= 0.0:
        # No time step -> just return a copy with zero accel / yaw_rate.
        return PointMassState(
            x_m=state.x_m,
            y_m=state.y_m,
            z_m=state.z_m,
            psi_rad=state.psi_rad,
            gamma_rad=state.gamma_rad,
            v_mps=state.v_mps,
            ax_n_mps2=0.0,
            ay_n_mps2=0.0,
            az_n_mps2=0.0,
            yaw_rate_radps=0.0,
        )

    heading_rate_cmd = float(controls.get("heading_rate_cmd", 0.0))
    accel_cmd = float(controls.get("accel_cmd", 0.0))
    elevator_cmd = float(controls.get("elevator_cmd", 0.0))

    # Simple mapping: elevator command -> flight-path angle rate
    k_gamma = 0.15  # rad/s per 1.0 elevator command
    gamma_dot = k_gamma * elevator_cmd
    gamma_max = math.radians(20.0)  # limit climb / descent angle

    psi_prev = state.psi_rad
    gamma_prev = state.gamma_rad
    v_prev = state.v_mps

    # Integrate yaw, gamma, and speed
    psi_next = psi_prev + heading_rate_cmd * dt_s
    psi_next = ((psi_next + math.pi) % (2.0 * math.pi)) - math.pi  # wrap [-pi, pi]

    gamma_next = gamma_prev + gamma_dot * dt_s
    gamma_next = max(-gamma_max, min(gamma_max, gamma_next))

    v_next = max(0.0, v_prev + accel_cmd * dt_s)

    # Velocities before and after step (for inertial acceleration)
    vn_prev, ve_prev, vd_prev = _vel_components(v_prev, gamma_prev, psi_prev)
    vn_next, ve_next, vd_next = _vel_components(v_next, gamma_next, psi_next)

    ax_n = (vn_next - vn_prev) / dt_s
    ay_n = (ve_next - ve_prev) / dt_s
    az_n = (vd_next - vd_prev) / dt_s

    # Integrate position using new velocity
    x_next = state.x_m + vn_next * dt_s
    y_next = state.y_m + ve_next * dt_s
    z_next = state.z_m + vd_next * dt_s

    yaw_rate = heading_rate_cmd

    return PointMassState(
        x_m=x_next,
        y_m=y_next,
        z_m=z_next,
        psi_rad=psi_next,
        gamma_rad=gamma_next,
        v_mps=v_next,
        ax_n_mps2=ax_n,
        ay_n_mps2=ay_n,
        az_n_mps2=az_n,
        yaw_rate_radps=yaw_rate,
    )