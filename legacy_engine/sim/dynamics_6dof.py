# sim/dynamics_6dof.py

from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class AircraftParams:
    mass_kg: float
    Ixx: float
    Iyy: float
    Izz: float
    S: float        # wing area [m^2]
    b: float        # span [m]
    c_bar: float    # mean aerodynamic chord [m]
    rho: float      # air density [kg/m^3]
    T_max: float    # max thrust [N]

    # Aerodynamic coefficients
    CL0: float = 0.3
    CL_alpha: float = 4.5   # [1/rad]
    CL_q: float = 3.0
    CL_de: float = 0.8

    CD0: float = 0.03
    CD_k: float = 0.08      # parabolic drag factor

    CY_beta: float = -0.98
    CY_dr: float = 0.17

    Cl_beta: float = -0.12
    Cl_p: float = -0.4
    Cl_r: float = 0.14
    Cl_da: float = 0.08

    Cm0: float = 0.02
    Cm_alpha: float = -1.0
    Cm_q: float = -8.0
    Cm_de: float = -1.1

    Cn_beta: float = 0.25
    Cn_p: float = -0.022
    Cn_r: float = -0.35
    Cn_dr: float = -0.06

    da_max_deg: float = 20.0
    de_max_deg: float = 20.0
    dr_max_deg: float = 25.0

class SixDofAircraft:
    """
    Simple 6-DOF fixed-wing rigid-body model in NED/body frames.

    State x = [p_ned(3), v_b(3), q(4), omega_b(3)]
    """

    def __init__(self, params: AircraftParams, g: float = 9.80665):
        self.p = params
        self.g = g

        # Inertia matrix and its inverse (diagonal approx)
        self.I = np.diag([params.Ixx, params.Iyy, params.Izz])
        self.I_inv = np.diag([1.0 / params.Ixx, 1.0 / params.Iyy, 1.0 / params.Izz])

    # ---------- Public API ----------

    def derivatives(self, dt_s: float, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Compute state derivatives xdot at time t.

        x: (13,) state vector
        u: (4,) control [aileron, elevator, rudder, throttle]
        """
        # Unpack state
        pn = x[0:3]            # N, E, D
        vb = x[3:6]            # body velocities [u, v, w]
        q = x[6:10]            # quaternion [qw, qx, qy, qz]
        omega_b = x[10:13]     # body rates [p, q, r]

        # Normalize quaternion for numerical safety
        q_norm = np.linalg.norm(q)
        if q_norm < 1e-6:
            q = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            q = q / q_norm

        # Controls
        delta_a, delta_e, delta_r, delta_t = u
        delta_t = float(np.clip(delta_t, 0.0, 1.0))
        # Convert to radians
        da_max = np.deg2rad(self.p.da_max_deg)
        de_max = np.deg2rad(self.p.de_max_deg)
        dr_max = np.deg2rad(self.p.dr_max_deg)
        # Treat inputs as radians (matches live_loop + your logged saturation at 0.349 rad)
        da = float(np.clip(delta_a, -da_max, da_max))
        de = float(np.clip(delta_e, -de_max, de_max))
        dr = float(np.clip(delta_r, -dr_max, dr_max))


        # Rotation matrices
        C_nb = self.quat_to_dcm(q) # body -> NED
        
        # --- Sanity Checks ---
        # For identity quaternion, C_nb should be identity if it truly maps body->NED
        if np.allclose(q, np.array([1.0, 0.0, 0.0, 0.0]), atol=1e-6):
            if not np.allclose(C_nb, np.eye(3), atol=1e-6):
                raise RuntimeError(f"quat_to_dcm is not identity for identity quaternion:\n{C_nb}")
            
        # Orthogonality check (always true for a proper DCM)
        M = C_nb @ C_nb.T
        if (not np.all(np.isfinite(C_nb))) or (not np.all(np.isfinite(M))):
            raise RuntimeError(f"C_nb contains NaN/Inf. q={q}")
        if not np.allclose(M, np.eye(3), atol=1e-5):
            raise RuntimeError(f"C_nb not orthonormal. q={q}\nC_nb=\n{C_nb}\nC_nb*C_nb^T=\n{M}")
        
        C_bn = C_nb.T # NED -> body

        # Position kinematics
        v_ned = C_nb @ vb
        p_dot = v_ned

        # Aerodynamic forces and moments (body frame)
        F_aero_b, M_aero_b = self.aero_forces_moments(vb, omega_b, da, de, dr)

        # Prevent numeric blow-up while debugging (tune later / remove later)
        M_limit = 200.0  # N*m per-axis cap (debug safety)
        M_aero_b = np.clip(M_aero_b, -M_limit, M_limit)

        # Thrust (body x-axis)
        F_thrust_b = np.array([self.p.T_max * delta_t, 0.0, 0.0])

        # Gravity in body frame
        g_n = np.array([0.0, 0.0, self.g])
        F_g_b = self.p.mass_kg * (C_bn @ g_n)

        # Total force in body frame
        F_total_b = F_aero_b + F_thrust_b + F_g_b

        # --- Clamp forces and moments before computing accelerations ---
        # g_limit = 5.0  # +/- 5 g per axis
        # F_limit = g_limit * self.p.mass_kg * self.g
        # F_total_b = np.clip(F_total_b, -F_limit, F_limit)

        # M_limit = 50.0  # N·m limit on each axis
        # M_b = np.clip(M_aero_b, -M_limit, M_limit)

        # Translational dynamics (body frame)
        v_dot_b = F_total_b / self.p.mass_kg - np.cross(omega_b, vb)

        # Rotational dynamics
        omega_dot_b = self.I_inv @ (M_aero_b - np.cross(omega_b, self.I @ omega_b))

        # Attitude kinematics
        q_dot = self.quat_dot(q, omega_b)

        # Pack derivatives
        xdot = np.zeros_like(x)
        xdot[0:3] = p_dot
        xdot[3:6] = v_dot_b
        xdot[6:10] = q_dot
        xdot[10:13] = omega_dot_b

        return xdot
    

    # ---------- Internal helpers ----------

    @staticmethod
    def quat_to_dcm(q: np.ndarray) -> np.ndarray:
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

    @staticmethod
    def quat_dot(q: np.ndarray, omega_b: np.ndarray) -> np.ndarray:
        qw, qx, qy, qz = q
        p, q_rate, r = omega_b

        Omega = np.array([
            [0.0,    -p,     -q_rate, -r],
            [p,       0.0,    r,      -q_rate],
            [q_rate, -r,      0.0,     p],
            [r,       q_rate, -p,      0.0],
        ])
        return 0.5 * (Omega @ q)

    def aero_forces_moments(self,
                            vb: np.ndarray,
                            omega_b: np.ndarray,
                            da: float,
                            de: float,
                            dr: float) -> Tuple[np.ndarray, np.ndarray]:
        
        u, v, w = vb
        p, q_rate, r = omega_b

        # Raw airspeed
        V_raw = float(np.linalg.norm(vb) + 1e-6)

        # Keep the model in a reasonable envelope while debugging
        V = float(np.clip(V_raw, 0.1, 120.0))

        # With body z positive DOWN, standard AoA (nose-up positive) uses -w
        alpha_raw = float(np.arctan2(-w, u))
        beta_raw  = float(np.arcsin(np.clip(v / V_raw, -1.0, 1.0)))

        # --- SIMPLE STALL / SANITY LIMITS ---
        alpha_eff = float(np.clip(alpha_raw, -0.35, 0.35))  # +/- 20 deg
        beta_eff  = float(np.clip(beta_raw,  -0.35, 0.35))

        qbar = 0.5 * self.p.rho * V * V
        S, b, c = self.p.S, self.p.b, self.p.c_bar

        # Non-dimensional rates (use V (clamped) to avoid insane hats)
        p_hat = float(p * b / (2.0 * V))
        q_hat = float(q_rate * c / (2.0 * V))
        r_hat = float(r * b / (2.0 * V))

        # Lift / Drag / Side force coefficients (use alpha_eff/beta_eff)
        CL = (self.p.CL0 +
            self.p.CL_alpha * alpha_eff +
            self.p.CL_q * q_hat +
            self.p.CL_de * de)

        CD = self.p.CD0 + self.p.CD_k * CL * CL

        CY = self.p.CY_beta * beta_eff + self.p.CY_dr * dr

        # Moments coefficients (use alpha_eff/beta_eff)
        Cl = (self.p.Cl_beta * beta_eff +
            self.p.Cl_p * p_hat +
            self.p.Cl_r * r_hat +
            self.p.Cl_da * da)

        Cm = (self.p.Cm0 +
            self.p.Cm_alpha * alpha_eff +
            self.p.Cm_q * q_hat +
            self.p.Cm_de * de)

        Cn = (self.p.Cn_beta * beta_eff +
            self.p.Cn_p * p_hat +
            self.p.Cn_r * r_hat +
            self.p.Cn_dr * dr)

        # Forces in wind axes
        L = qbar * S * CL
        D = qbar * S * CD
        Y = qbar * S * CY

        # --- Wind axes built directly from velocity direction (robust & convention-safe) ---
        V = np.linalg.norm(vb) + 1e-9
        xw_b = vb / V # wind x-axis (along velocity, in body coords)

        ez_b = np.array([0.0, 0.0, 1.0]) # body +z (DOWN in your convention)
        zproj = ez_b - xw_b * np.dot(xw_b, ez_b)

        # If velocity is near-parallel to ez_b, fall back to another reference axis
        nz = np.linalg.norm(zproj)
        if nz < 1e-6:
            ey_b = np.array([0.0, 1.0, 0.0])
            zproj = ey_b - xw_b * np.dot(xw_b, ey_b)
            nz = np.linalg.norm(zproj)
            if nz < 1e-6:
                zproj = np.array([0.0, 0.0, 1.0])  # last-resort

        zw_b = zproj / (nz + 1e-12)
        yw_b = np.cross(zw_b, xw_b) # right-handed

        # Columns are wind axes expressed in body coords
        C_bw = np.column_stack((xw_b, yw_b, zw_b))

        # Forces in wind axes: drag opposite xw, lift opposite zw (since +zw is "down-ish")
        F_w = np.array([-D, Y, -L], dtype=float)
        F_b = C_bw @ F_w
        if np.dot(F_b, vb) > 1e-6:
            print("WARNING: aero forces adding energy! dot(F,v) =", np.dot(F_b, vb))

        # Moments in body axes
        L_roll = qbar * S * b * Cl
        M_pitch = qbar * S * c * Cm
        N_yaw = qbar * S * b * Cn
        M_b = np.array([L_roll, M_pitch, N_yaw])

        return F_b, M_b