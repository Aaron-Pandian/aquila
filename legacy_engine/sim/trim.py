# sim/trim.py
import numpy as np

# -------------------------
# State layout (13D quaternion model)
# x = [pn(3), vb(3), q(4), omega_b(3)]
# -------------------------
IDX_PN0 = 0
IDX_VB0 = 3
IDX_Q0  = 6
IDX_OMG0 = 10

def solve_longitudinal_trim(
    aircraft,
    x0,
    target_altitude,
    *,
    # Choose ONE of these “modes”:
    target_speed=None,          # Option A: hold speed, solve throttle
    throttle_fixed=None,        # Option B: hold throttle, solve speed
    # Initial guesses
    theta0=0.05,
    elevator0=0.0,
    throttle0=0.2,
    speed0=15.0,
    # Bounds
    elevator_bounds=(-0.8, 0.8),
    theta_bounds=(-0.3, 0.3),
    throttle_bounds=(0.0, 1.0),
    speed_bounds=(5.0, 40.0),
    # Solver params
    tol=1e-6,
    max_iter=60,
):
    """
    Longitudinal trim solving (u_dot, w_dot, q_dot) = (0,0,0).

    Mode A (recommended for “hold speed”): provide target_speed, leave throttle_fixed=None
      -> solves for theta, elevator, throttle.

    Mode B (recommended for “hold throttle” debugging): provide throttle_fixed, leave target_speed=None
      -> solves for theta, elevator, speed.

    Returns:
      If Mode A: (theta_trim, elevator_trim, throttle_trim)
      If Mode B: (theta_trim, elevator_trim, speed_trim)
    """
    if (target_speed is None) == (throttle_fixed is None):
        raise ValueError("Provide exactly one of target_speed or throttle_fixed.")

    def build_state(theta, V):
        x = x0.copy()

        # Position: set altitude in NED (pd = -altitude)
        x[IDX_PN0 + 2] = -float(target_altitude)

        # Body velocity consistent with NED velocity [V,0,0] and pitch theta (phi=psi=0)
        x[IDX_VB0 + 0] = V * np.cos(theta)   # u
        x[IDX_VB0 + 1] = 0.0                 # v
        x[IDX_VB0 + 2] = V * np.sin(theta)   # w (z-down)

        # Attitude quaternion (pitch about +Y)
        x[IDX_Q0:IDX_Q0+4] = np.array([
            np.cos(theta / 2.0),
            0.0,
            -np.sin(theta / 2.0), # pitch about +Y, theta positive = nose-up matches in live loop
            0.0,
        ])

        # IMPORTANT: trim assumes zero body rates
        x[IDX_OMG0:IDX_OMG0+3] = 0.0
        return x

    def residual(z):
        theta = float(z[0])
        elevator = float(z[1])

        if throttle_fixed is None:
            # Mode A: solve throttle, hold speed
            V = float(target_speed)
            throttle = float(z[2])
        else:
            # Mode B: solve speed, hold throttle
            V = float(z[2])
            throttle = float(throttle_fixed)

        x = build_state(theta, V)
        u = np.array([0.0, elevator, 0.0, throttle])

        dx = aircraft.derivatives(0.0, x, u)

        u_dot = dx[IDX_VB0 + 0]
        w_dot = dx[IDX_VB0 + 2]
        q_dot = dx[IDX_OMG0 + 1]  # pitch rate derivative

        return np.array([u_dot, w_dot, q_dot], dtype=float)

    # Unknown vector
    if throttle_fixed is None:
        # Mode A: [theta, elevator, throttle]
        z = np.array([theta0, elevator0, throttle0], dtype=float)
        lb = np.array([theta_bounds[0], elevator_bounds[0], throttle_bounds[0]], dtype=float)
        ub = np.array([theta_bounds[1], elevator_bounds[1], throttle_bounds[1]], dtype=float)
    else:
        # Mode B: [theta, elevator, speed]
        z = np.array([theta0, elevator0, speed0], dtype=float)
        lb = np.array([theta_bounds[0], elevator_bounds[0], speed_bounds[0]], dtype=float)
        ub = np.array([theta_bounds[1], elevator_bounds[1], speed_bounds[1]], dtype=float)

    def clip(zv):
        return np.minimum(np.maximum(zv, lb), ub)

    z = clip(z)

    # Damped Newton iterations
    eps = 1e-4
    for _ in range(max_iter):
        r = residual(z)
        nr = np.linalg.norm(r)
        if nr < tol:
            break

        # Finite-difference Jacobian (central difference)
        J = np.zeros((3, 3), dtype=float)
        for i in range(3):
            dz = np.zeros(3, dtype=float)
            dz[i] = eps
            rp = residual(clip(z + dz))
            rm = residual(clip(z - dz))
            J[:, i] = (rp - rm) / (2.0 * eps)

        # Least-squares step (more robust than solve() if near-singular)
        step, *_ = np.linalg.lstsq(J, r, rcond=None)

        # Backtracking line search
        alpha = 1.0
        improved = False
        for _ls in range(12):
            z_try = clip(z - alpha * step)
            if np.linalg.norm(residual(z_try)) < nr:
                z = z_try
                improved = True
                break
            alpha *= 0.5

        if not improved:
            break

    theta_trim = float(z[0])
    elevator_trim = float(z[1])
    third = float(z[2])

    return theta_trim, elevator_trim, third