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
    """Simple 3-DOF state for initial prototyping + IMU metadata.

    This is still a kinematic point-mass model in the horizontal plane,
    but we store the inertial acceleration in NED and yaw rate so that
    the IMU model can be more realistic.

    All NED quantities follow the usual convention:
    - x: North, y: East, z: Down
    """
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = -100.0  # NED: negative altitude

    psi_rad: float = 0.0        # heading (yaw)
    v_mps: float = 15.0         # speed (scalar, horizontal)

    # Inertial acceleration in NED (for IMU accelerometer)
    ax_n_mps2: float = 0.0
    ay_n_mps2: float = 0.0
    az_n_mps2: float = 0.0

    # Yaw rate (for IMU gyro z)
    yaw_rate_radps: float = 0.0


def step_dynamics(state: PointMassState, controls: Dict[str, float], dt_s: float,) -> PointMassState:
    """
    Propagate the simple point-mass dynamics forward by dt_s.

    Parameters
    ----------
    state : PointMassState
        Current state.
    controls : dict
        Control commands. Expected keys:
        - "heading_rate_cmd": desired heading rate [rad/s]
        - "accel_cmd": desired longitudinal acceleration [m/s^2]
    dt_s : float
        Time step [s].

    Returns
    -------
    PointMassState
        Next state after dt_s, including inertial acceleration metadata.
    """
    heading_rate_cmd = float(controls.get("heading_rate_cmd", 0.0))
    accel_cmd = float(controls.get("accel_cmd", 0.0))

    # Yaw rate is directly the commanded heading rate in this simple model.
    yaw_rate = heading_rate_cmd

    # Update heading
    psi_next = state.psi_rad + yaw_rate * dt_s
    psi_next = ((psi_next + math.pi) % (2.0 * math.pi)) - math.pi  # wrap [-pi, pi]

    # Update speed
    v_next = max(0.0, state.v_mps + accel_cmd * dt_s)

    # Inertial acceleration in NED (horizontal only for now)
    ax_n = accel_cmd * math.cos(psi_next)
    ay_n = accel_cmd * math.sin(psi_next)
    az_n = 0.0

    # Update position in NED (x: North, y: East, z: Down)
    x_next = state.x_m + v_next * dt_s * math.cos(psi_next)
    y_next = state.y_m + v_next * dt_s * math.sin(psi_next)
    z_next = state.z_m  # still constant; vertical dynamics will come later

    return PointMassState(
        x_m=x_next,
        y_m=y_next,
        z_m=z_next,
        psi_rad=psi_next,
        v_mps=v_next,
        ax_n_mps2=ax_n,
        ay_n_mps2=ay_n,
        az_n_mps2=az_n,
        yaw_rate_radps=yaw_rate,
    )