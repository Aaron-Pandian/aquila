"""
Scenario runner for Aquila.

This module defines a simple waypoint-following scenario using the
6-DOF dynamics and sensor models. It writes a CSV file containing
true state and noisy measurements PLUS extra truth/debug columns.
"""

from __future__ import annotations

import csv
import math
import numpy as np
from dataclasses import dataclass
from pathlib import Path
from typing import List, Dict, Tuple

from sim.dynamics_6dof import SixDofAircraft, AircraftParams
from .sensors import (
    measure_imu_from_state,
    measure_gps_from_state,
    measure_baro_from_state,
    BaroParams,
    GpsParams,
    ImuParams,
)

_REPO_ROOT = Path(__file__).resolve().parents[1]
LOGS_DIR = _REPO_ROOT / "logs"
SENSORS_LOG_PATH = LOGS_DIR / "sim_sensors.csv"

params = AircraftParams(
    mass_kg=8.0,
    Ixx=0.25, Iyy=0.30, Izz=0.45,
    S=0.8, b=2.0, c_bar=0.4,
    rho=1.225,
    T_max=50.0
)

model = SixDofAircraft(params)


@dataclass
class Waypoint:
    x_m: float
    y_m: float


def _ensure_logs_dir() -> None:
    LOGS_DIR.mkdir(parents=True, exist_ok=True)


def _quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """Copy of the mapping in dynamics_6dof."""
    qw, qx, qy, qz = q
    qw2, qx2, qy2, qz2 = qw*qw, qx*qx, qy*qy, qz*qz

    C = np.empty((3, 3))
    C[0, 0] = 1 - 2*(qy2 + qz2)
    C[0, 1] = 2*(qx*qy + qw*qz)
    C[0, 2] = 2*(qx*qz - qw*qy)

    C[1, 0] = 2*(qx*qy - qw*qz)
    C[1, 1] = 1 - 2*(qx2 + qz2)
    C[1, 2] = 2*(qy*qz + qw*qx)

    C[2, 0] = 2*(qx*qz + qw*qy)
    C[2, 1] = 2*(qy*qz - qw*qx)
    C[2, 2] = 1 - 2*(qx2 + qy2)
    return C


def _state_to_nav_view(state: np.ndarray) -> dict:
    """
    Extract navigation-relevant quantities from 6-DOF state.
    state = [N,E,D, u,v,w, qw,qx,qy,qz, p,q,r]
    Returns dict with N, E, alt, V, phi, theta, psi.
    """
    N, E, D = state[0:3]
    vb = state[3:6]
    q = state[6:10] / np.linalg.norm(state[6:10])

    # Body -> NED
    C_nb = _quat_to_dcm(q).T
    v_ned = C_nb @ vb
    v_n, v_e, v_d = v_ned

    V = math.hypot(v_n, v_e)  # horizontal groundspeed
    alt = -D                  # altitude above reference plane

    # Euler angles from quaternion (NED convention, z-y-x)
    qw, qx, qy, qz = q

    sin_theta = -2.0 * (qx*qz - qw*qy)
    sin_theta = max(-1.0, min(1.0, sin_theta))
    theta = math.asin(sin_theta)

    phi = math.atan2(2.0 * (qw*qx + qy*qz),
                     1.0 - 2.0 * (qx*qx + qy*qy))

    psi = math.atan2(2.0 * (qw*qz + qx*qy),
                     1.0 - 2.0 * (qy*qy + qz*qz))

    return {
        "N": N,
        "E": E,
        "alt": alt,
        "V": V,
        "phi": phi,
        "theta": theta,
        "psi": psi,
    }


def _heading_to_waypoint_from_nav(nav: dict, wp: Waypoint) -> float:
    dx = wp.x_m - nav["N"]
    dy = wp.y_m - nav["E"]
    return math.atan2(dy, dx)


def _wrap_angle(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


# ---- Autopilot ----

def _aileron_from_heading(
    nav: dict,
    wp: Waypoint,
    max_bank_deg: float = 20.0,
    k_heading: float = 1.0,
    k_bank: float = 0.8,
) -> float:
    psi_des = _heading_to_waypoint_from_nav(nav, wp)
    psi_err = _wrap_angle(psi_des - nav["psi"])

    phi_cmd = k_heading * psi_err
    phi_max = math.radians(max_bank_deg)
    phi_cmd = max(-phi_max, min(phi_max, phi_cmd))

    phi_err = phi_cmd - nav["phi"]
    delta_a = k_bank * phi_err
    return max(-1.0, min(1.0, delta_a))


def _elevator_from_altitude(
    nav: dict,
    alt_target_m: float = 100.0,
    k_alt: float = 0.01,
    k_pitch: float = 0.6,
    max_pitch_deg: float = 10.0,
) -> float:
    alt_err = alt_target_m - nav["alt"]
    theta_cmd = k_alt * alt_err
    theta_max = math.radians(max_pitch_deg)
    theta_cmd = max(-theta_max, min(theta_max, theta_cmd))

    theta_err = theta_cmd - nav["theta"]
    delta_e = k_pitch * theta_err
    return max(-1.0, min(1.0, delta_e))


def _throttle_from_speed(
    nav: dict,
    v_target_mps: float = 15.0,
    throttle_trim: float = 0.6,
    k_speed: float = 0.01,
) -> float:
    err = v_target_mps - nav["V"]
    delta_t = throttle_trim + k_speed * err
    return max(0.0, min(1.0, delta_t))


def rk4_step(model_: SixDofAircraft, t: float, x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    k1 = model_.derivatives(t, x, u)
    k2 = model_.derivatives(t + 0.5*dt, x + 0.5*dt*k1, u)
    k3 = model_.derivatives(t + 0.5*dt, x + 0.5*dt*k2, u)
    k4 = model_.derivatives(t + dt, x + dt*k3, u)
    return x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)


def rate_limit(u_prev: float, u_cmd: float, dt: float, rate_per_s: float) -> float:
    du_max = rate_per_s * dt
    return u_prev + np.clip(u_cmd - u_prev, -du_max, du_max)


def quat_from_pitch(theta: float) -> np.ndarray:
    return np.array([np.cos(theta/2), 0.0, np.sin(theta/2), 0.0], dtype=float)


def compute_trim_longitudinal(V: float = 15.0) -> Tuple[float, float, float]:
    """
    Solve for (alpha_rad, de_rad, throttle) such that:
      1) Lift balances weight
      2) Pitch moment is zero
    Assumes q_hat=0 (steady) and uses your linear CL/Cm model.
    """
    g = 9.80665
    qbar = 0.5 * params.rho * V * V
    W = params.mass_kg * g

    CL_req = W / (qbar * params.S)

    # Solve:
    # [CL_alpha  CL_de] [alpha] = [CL_req - CL0]
    # [Cm_alpha  Cm_de] [de   ]   [-Cm0]
    A = np.array([
        [params.CL_alpha, params.CL_de],
        [params.Cm_alpha, params.Cm_de],
    ], dtype=float)

    b = np.array([
        CL_req - params.CL0,
        -params.Cm0,
    ], dtype=float)

    alpha, de = np.linalg.solve(A, b)

    # Drag + throttle (using trimmed CL)
    CL = params.CL0 + params.CL_alpha * alpha + params.CL_de * de
    CD = params.CD0 + params.CD_k * CL * CL
    D = qbar * params.S * CD
    throttle = float(np.clip(D / params.T_max, 0.0, 1.0))

    return float(alpha), float(de), throttle


def initial_state() -> Tuple[np.ndarray, float, float]:
    
    V = 15.0
    alpha, de_trim_rad, throttle = compute_trim_longitudinal(V)

    p_ned = np.array([0.0, 0.0, -100.0])

    # Body velocity consistent with alpha (z-down body):
    vb = np.array([V*np.cos(alpha), 0.0, -V*np.sin(alpha)])

    # Set attitude so body is pitched up by theta=alpha (gamma≈0 assumption)
    theta = alpha
    q = quat_from_pitch(theta)

    omega_b = np.zeros(3)

    # IMPORTANT: store elevator trim somewhere accessible if you want to apply it as bias later
    # For now, just return it so run_scenario can initialize u_act[1]
    state = np.concatenate([p_ned, vb, q, omega_b])
    return state, throttle, de_trim_rad


def _truth_aero_debug(state: np.ndarray, controls_norm: np.ndarray) -> Dict[str, float]:
    """
    Compute "truth" aero quantities for logging: V, alpha, beta, qbar, coeffs, forces, etc.
    controls_norm are normalized [-1,1] surfaces and [0,1] throttle, matching what you pass to dynamics.
    """
    vb = state[3:6]
    omega_b = state[10:13]
    u_b, v_b, w_b = vb
    p_rate, q_rate, r_rate = omega_b

    V = float(np.linalg.norm(vb) + 1e-6)
    alpha = float(np.arctan2(-w_b, u_b)) # updated to match downward w, corresponds to positive alpha
    beta = float(np.arcsin(np.clip(v_b / V, -1.0, 1.0)))
    qbar = float(0.5 * params.rho * V * V)

    # Convert normalized surfaces -> radians (must match dynamics_6dof.py)
    da = float(np.clip(controls_norm[0], -1.0, 1.0)) * np.deg2rad(params.da_max_deg)
    de = float(np.clip(controls_norm[1], -1.0, 1.0)) * np.deg2rad(params.de_max_deg)
    dr = float(np.clip(controls_norm[2], -1.0, 1.0)) * np.deg2rad(params.dr_max_deg)
    dt = float(np.clip(controls_norm[3], 0.0, 1.0))

    S, b, c = params.S, params.b, params.c_bar

    # Non-dimensional rates
    p_hat = p_rate * b / (2.0 * V)
    q_hat = q_rate * c / (2.0 * V)
    r_hat = r_rate * b / (2.0 * V)

    # Coefficients (same as aero_forces_moments)
    CL = float(params.CL0 + params.CL_alpha * alpha + params.CL_q * q_hat + params.CL_de * de)
    CD = float(params.CD0 + params.CD_k * CL * CL)
    CY = float(params.CY_beta * beta + params.CY_dr * dr)

    Cl = float(params.Cl_beta * beta + params.Cl_p * p_hat + params.Cl_r * r_hat + params.Cl_da * da)
    Cm = float(params.Cm0 + params.Cm_alpha * alpha + params.Cm_q * q_hat + params.Cm_de * de)
    Cn = float(params.Cn_beta * beta + params.Cn_p * p_hat + params.Cn_r * r_hat + params.Cn_dr * dr)

    # Forces in wind axes (magnitudes)
    L = float(qbar * S * CL)
    D = float(qbar * S * CD)
    Y = float(qbar * S * CY)

    # True aero forces in body (reuse your model implementation for consistency)
    F_aero_b, _M_aero_b = model.aero_forces_moments(vb, omega_b, da, de, dr)

    # Total Z (body) force sanity-check (aero + thrust + gravity)
    q_att = state[6:10] / np.linalg.norm(state[6:10])
    C_bn = model.quat_to_dcm(q_att)  # NED -> body
    g_n = np.array([0.0, 0.0, model.g])
    F_g_b = params.mass_kg * (C_bn @ g_n)
    F_thrust_b = np.array([params.T_max * dt, 0.0, 0.0])
    F_total_b = F_aero_b + F_thrust_b + F_g_b

    W = float(params.mass_kg * model.g)
    L_over_W = float(L / (W + 1e-9))

    return {
        "V_true": V,
        "alpha_rad": alpha,
        "beta_rad": beta,
        "qbar_pa": qbar,
        "da_rad": da,
        "de_rad": de,
        "dr_rad": dr,
        "CL": CL,
        "CD": CD,
        "CY": CY,
        "Cl": Cl,
        "Cm": Cm,
        "Cn": Cn,
        "L_N": L,
        "D_N": D,
        "Y_N": Y,
        "W_N": W,
        "L_over_W": L_over_W,
        "F_aero_x_N": float(F_aero_b[0]),
        "F_aero_y_N": float(F_aero_b[1]),
        "F_aero_z_N": float(F_aero_b[2]),
        "F_total_z_N": float(F_total_b[2]),
        "p_true_radps": float(p_rate),
        "q_true_radps": float(q_rate),
        "r_true_radps": float(r_rate),
    }


def run_scenario() -> None:
    _ensure_logs_dir()

    dt_s = 0.05
    t_final_s = 60.0
    n_steps = int(t_final_s / dt_s)

    waypoints: List[Waypoint] = [
        Waypoint(0.0, 0.0),
        Waypoint(200.0, 0.0),
        Waypoint(200.0, 200.0),
        Waypoint(0.0, 200.0),
    ]
    wp_index = 0
    wp_reached_thresh_m = 10.0

    state, throttle_trim, de_trim_rad = initial_state()

    # Convert de_rad to normalized elevator command [-1,1]
    de_trim_norm = float(np.clip(de_trim_rad / np.deg2rad(params.de_max_deg), -1.0, 1.0))

    # Actuator state (rate-limited)
    u_act = np.array([0.0, de_trim_norm, 0.0, throttle_trim], dtype=float)

    # Initialize sensors
    rng = np.random.default_rng(seed=0)
    imu_bias = {"accel": np.zeros(3), "gyro": np.zeros(3)}
    imu_params = ImuParams()
    gps_params = GpsParams()
    baro_params = BaroParams()

    state_prev = state.copy()

    with SENSORS_LOG_PATH.open(mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                # --- existing pipeline columns ---
                "t_s",
                "n_m",
                "e_m",
                "d_m",
                "alt_m",                 # explicit altitude = -D (positive up)
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

                # --- truth attitude (debug) ---
                "phi_true_rad",
                "theta_true_rad",
                "psi_true_rad",

                # --- truth airdata/aero (debug) ---
                "V_true",
                "alpha_rad",
                "beta_rad",
                "qbar_pa",

                # --- true body rates (debug) ---
                "p_true_radps",
                "q_true_radps",
                "r_true_radps",

                # --- actual surface deflections (debug) ---
                "da_rad",
                "de_rad",
                "dr_rad",

                # --- coefficients (debug) ---
                "CL",
                "CD",
                "CY",
                "Cl",
                "Cm",
                "Cn",

                # --- forces (debug) ---
                "L_N",
                "D_N",
                "Y_N",
                "W_N",
                "L_over_W",
                "F_aero_x_N",
                "F_aero_y_N",
                "F_aero_z_N",
                "F_total_z_N",
            ]
        )

        t = 0.0
        for _ in range(n_steps):
            nav = _state_to_nav_view(state)

            # waypoint switching
            wp = waypoints[wp_index]
            dist_to_wp = math.hypot(nav["N"] - wp.x_m, nav["E"] - wp.y_m)
            if dist_to_wp < wp_reached_thresh_m:
                wp_index = (wp_index + 1) % len(waypoints)
                wp = waypoints[wp_index]

            p_rate, q_rate, r_rate = state[10:13]

            # --- commands (normalized) ---
            delta_a = _aileron_from_heading(nav, wp)
            delta_e = _elevator_from_altitude(nav, alt_target_m=100.0)
            delta_t = _throttle_from_speed(nav, v_target_mps=15.0, throttle_trim=throttle_trim)

            # rate damping
            delta_a = max(-1.0, min(1.0, delta_a + (-0.08 * p_rate)))
            delta_e = max(-1.0, min(1.0, delta_e + (-0.12 * q_rate)))
            delta_r = max(-1.0, min(1.0, (-0.05 * r_rate)))

            # rate limit each channel
            u_cmd = np.array([delta_a, de_trim_norm + delta_e, delta_r, delta_t], dtype=float)
            u_act[0] = rate_limit(u_act[0], u_cmd[0], dt_s, rate_per_s=2.0)   # aileron
            u_act[1] = rate_limit(u_act[1], u_cmd[1], dt_s, rate_per_s=2.0)   # elevator 
            u_act[2] = rate_limit(u_act[2], u_cmd[2], dt_s, rate_per_s=2.0)   # rudder
            u_act[3] = rate_limit(u_act[3], u_cmd[3], dt_s, rate_per_s=1.0)   # throttle

            controls = u_act.copy()

            # propagate
            state = rk4_step(model, t, state, controls, dt_s)
            state[6:10] /= np.linalg.norm(state[6:10])

            if not np.all(np.isfinite(state)):
                print(f"[sim.scenarios] Non-finite state at t={t:.3f} s, stopping sim.")
                break

            # sensors
            imu_meas, imu_bias = measure_imu_from_state(state, state_prev, dt_s, imu_bias, imu_params, rng)
            gps_meas = measure_gps_from_state(state, gps_params, rng)
            baro_meas = measure_baro_from_state(state, baro_params, rng)
            state_prev = state.copy()

            def _all_finite(d: Dict[str, float]) -> bool:
                return all(np.isfinite(v) for v in d.values())

            if not (_all_finite(imu_meas) and _all_finite(gps_meas) and _all_finite(baro_meas)):
                print(f"[sim.scenarios] Non-finite sensor data at t={t:.3f} s, stopping sim.")
                break

            # truth/debug bundle
            dbg = _truth_aero_debug(state, controls)

            # write
            N, E, D = state[0:3]
            alt_m = -D

            writer.writerow(
                [
                    f"{t:.3f}",
                    f"{N:.3f}",
                    f"{E:.3f}",
                    f"{D:.3f}",
                    f"{alt_m:.3f}",
                    f"{nav['psi']:.6f}",
                    f"{nav['V']:.3f}",

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

                    # truth attitude
                    f"{nav['phi']:.6f}",
                    f"{nav['theta']:.6f}",
                    f"{nav['psi']:.6f}",

                    # truth airdata/aero
                    f"{dbg['V_true']:.3f}",
                    f"{dbg['alpha_rad']:.6f}",
                    f"{dbg['beta_rad']:.6f}",
                    f"{dbg['qbar_pa']:.3f}",

                    # body rates
                    f"{dbg['p_true_radps']:.6f}",
                    f"{dbg['q_true_radps']:.6f}",
                    f"{dbg['r_true_radps']:.6f}",

                    # surface deflections (rad)
                    f"{dbg['da_rad']:.6f}",
                    f"{dbg['de_rad']:.6f}",
                    f"{dbg['dr_rad']:.6f}",

                    # coefficients
                    f"{dbg['CL']:.6f}",
                    f"{dbg['CD']:.6f}",
                    f"{dbg['CY']:.6f}",
                    f"{dbg['Cl']:.6f}",
                    f"{dbg['Cm']:.6f}",
                    f"{dbg['Cn']:.6f}",

                    # forces
                    f"{dbg['L_N']:.3f}",
                    f"{dbg['D_N']:.3f}",
                    f"{dbg['Y_N']:.3f}",
                    f"{dbg['W_N']:.3f}",
                    f"{dbg['L_over_W']:.6f}",
                    f"{dbg['F_aero_x_N']:.3f}",
                    f"{dbg['F_aero_y_N']:.3f}",
                    f"{dbg['F_aero_z_N']:.3f}",
                    f"{dbg['F_total_z_N']:.3f}",
                ]
            )

            t += dt_s

    print(f"Wrote sensor log to {SENSORS_LOG_PATH}")


def main() -> None:
    run_scenario()


if __name__ == "__main__":
    main()