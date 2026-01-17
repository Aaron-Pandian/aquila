# sim/sensors.py

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np


@dataclass
class ImuParams:
    accel_noise_std: float = 0.05     # [m/s^2] 1-sigma
    gyro_noise_std: float = 0.001     # [rad/s] 1-sigma
    accel_bias_std: float = 0.05      
    gyro_bias_std: float = 0.0001
    g: float = 9.80665


@dataclass
class GpsParams:
    pos_noise_std: float = 1.5        # [m]
    vel_noise_std: float = 0.2        # [m/s]
    update_rate_hz: float = 5.0       # 5 Hz GPS


@dataclass
class BaroParams:
    alt_noise_std: float = 0.5           # [m]
    base_pressure_pa: float = 101325.0   # [Pa]
    rho: float = 1.225                   # [kg/m^2]
    g: float = 9.80665                   # [m/s^2]


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


# ---------- IMU ----------

def measure_imu_from_state(
    x: np.ndarray,
    x_prev: np.ndarray,
    dt: float,
    imu_bias: Dict[str, np.ndarray],
    imu_params: ImuParams,
    rng: np.random.Generator,
) -> Tuple[Dict[str, float], Dict[str, np.ndarray]]:
    """
    Generate IMU accel/gyro measurement from 6-DOF states at two time steps.

    x, x_prev: state vectors at t and t-dt
    imu_bias: dict with 'accel' and 'gyro' bias 3-vectors (will be random-walk updated)
    Returns:
        meas: dict with ax, ay, az, gx, gy, gz
        imu_bias: updated biases
    """

    # Unpack state
    vb = x[3:6]
    q = x[6:10] / np.linalg.norm(x[6:10])
    omega_b = x[10:13]

    vb_prev = x_prev[3:6]
    omega_prev = x_prev[10:13]

    # Approximate body acceleration via finite difference
    a_b = (vb - vb_prev) / max(dt, 1e-6)

    # Subtract gravity to get specific force
    C_nb = quat_to_dcm(q)
    C_bn = C_nb.T
    g_n = np.array([0.0, 0.0, imu_params.g])
    g_b = C_bn @ g_n
    f_b = a_b - g_b   # "specific force" the IMU would measure

    # Simple gyro: just body rates
    omega_meas_true = omega_b

    # Random-walk update for biases
    accel_bias = imu_bias.get("accel", np.zeros(3))
    gyro_bias = imu_bias.get("gyro", np.zeros(3))

    accel_bias = accel_bias + imu_params.accel_bias_std * rng.normal(size=3) * np.sqrt(dt)
    gyro_bias = gyro_bias + imu_params.gyro_bias_std * rng.normal(size=3) * np.sqrt(dt)

    # Add noise + bias
    accel_meas = (f_b + accel_bias + imu_params.accel_noise_std * rng.normal(size=3))
    gyro_meas = (omega_meas_true + gyro_bias + imu_params.gyro_noise_std * rng.normal(size=3))

    meas = {
        "imu_ax": accel_meas[0],
        "imu_ay": accel_meas[1],
        "imu_az": accel_meas[2],
        "imu_gx": gyro_meas[0],
        "imu_gy": gyro_meas[1],
        "imu_gz": gyro_meas[2],
    }

    return meas, {"accel": accel_bias, "gyro": gyro_bias}


# ---------- GPS ----------

def measure_gps_from_state(
    x: np.ndarray,
    gps_params: GpsParams,
    rng: np.random.Generator,
) -> Dict[str, float]:
    """
    GPS-like measurement from state:
    - Position in NED (with noise)
    - Velocity in NED (with noise)
    """

    p_ned = x[0:3]
    vb = x[3:6]
    q = x[6:10] / np.linalg.norm(x[6:10])

    C_bn = quat_to_dcm(q)
    C_nb = C_bn.T
    v_ned = C_nb @ vb

    pos_meas = p_ned + gps_params.pos_noise_std * rng.normal(size=3)
    vel_meas = v_ned + gps_params.vel_noise_std * rng.normal(size=3)

    meas = {
        "gps_n": pos_meas[0],
        "gps_e": pos_meas[1],
        "gps_d": pos_meas[2],
        "gps_vn": vel_meas[0],
        "gps_ve": vel_meas[1],
        "gps_vd": vel_meas[2],
    }
    return meas


# ---------- Barometer ----------

def measure_baro_from_state(
    x: np.ndarray,
    baro_params: BaroParams,
    rng: np.random.Generator,
) -> Dict[str, float]:
    """
    Simple barometer: returns altitude estimate (or pressure) from p_ned[2].
    We treat D (down) as positive, so altitude = -D.
    """

    p_ned = x[0:3]
    D = p_ned[2]
    alt_true = -D  # altitude above reference plane

    alt_meas = alt_true + baro_params.alt_noise_std * rng.normal()

    # Optionally compute pressure from hydrostatic relation: p = p0 - rho*g*z
    # (z upwards is positive altitude)
    pressure = baro_params.base_pressure_pa - baro_params.rho * baro_params.g * alt_true

    meas = {
        "baro_alt": alt_meas,
        "baro_p": pressure,
    }
    return meas