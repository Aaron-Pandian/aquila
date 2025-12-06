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


@dataclass
class PointMassState:
    """Simple 3-DOF state for initial prototyping."""
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = -100.0  # North-East-Down (altitude is negative)
    psi_rad: float = 0.0  # yaw angle
    v_mps: float = 15.0   # speed


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
        Next state after dt_s.
    """
    heading_rate_cmd = float(controls.get("heading_rate_cmd", 0.0)) # Default value 0.0 for now
    accel_cmd = float(controls.get("accel_cmd", 0.0))

    # Update heading (simple integrator)
    psi_next = state.psi_rad + heading_rate_cmd * dt_s

    # Normalize heading to [-pi, pi] for numerical sanity
    if psi_next > 3.141592653589793:
        psi_next -= 2.0 * 3.141592653589793
    elif psi_next < -3.141592653589793:
        psi_next += 2.0 * 3.141592653589793

    # Update speed (no constraints for now; can add min/max later)
    v_next = max(0.0, state.v_mps + accel_cmd * dt_s)

    # Update position in NED (x: North, y: East)
    x_next = state.x_m + v_next * dt_s * float(__import__("math").cos(psi_next))
    y_next = state.y_m + v_next * dt_s * float(__import__("math").sin(psi_next))

    # Altitude remains constant in this simple model
    z_next = state.z_m

    return PointMassState(
        x_m=x_next,
        y_m=y_next,
        z_m=z_next,
        psi_rad=psi_next,
        v_mps=v_next,
    )