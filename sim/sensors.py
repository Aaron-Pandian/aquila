"""
Aquila sensor models (initial placeholders with basic noise).

This module provides simple IMU, GPS, and baro measurement models
based on the point-mass state. Noise levels are intentionally simple
and will be moved to config files as the project matures.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import math
import numpy as np

from .dynamics import PointMassState


# --- Configuration (can be migrated to YAML later) --- #

# IMU noise (1-sigma)
ACCEL_NOISE_MPS2 = 0.05
GYRO_NOISE_RADPS = 0.001

# GPS noise (1-sigma)
GPS_POS_NOISE_M = 1.5
GPS_VEL_NOISE_MPS = 0.2

# Baro noise (1-sigma)
BARO_ALT_NOISE_M = 0.5


@dataclass
class ImuMeasurement:
    accel_mps2: np.ndarray  # shape (3,)
    gyro_radps: np.ndarray  # shape (3,)


@dataclass
class GpsMeasurement:
    position_ned_m: np.ndarray  # [x, y, z]
    velocity_ned_mps: np.ndarray  # [vx, vy, vz]


@dataclass
class BaroMeasurement:
    altitude_m: float  # positive up (for convenience in logging)


def simulate_imu(state: PointMassState, dt_s: float) -> ImuMeasurement:
    """
    Simulate a crude IMU measurement from point-mass state.

    Assumptions:
    - No body-frame rotation (other than yaw) in this simple model.
    - Accelerometer mostly sees longitudinal acceleration; we ignore gravity
      for now since we're not modeling vertical motion in detail yet.
    """
    # Longitudinal acceleration is approximated as zero here; we will
    # refine this once we move to a richer dynamics model.
    accel_body = np.zeros(3, dtype=float)

    # Gyro only measures yaw rate in this simple model
    # For now, we treat yaw rate as zero; this is fine for the first iteration.
    gyro_body = np.zeros(3, dtype=float)

    # Add noise
    accel_body += np.random.normal(0.0, ACCEL_NOISE_MPS2, size=3)
    gyro_body += np.random.normal(0.0, GYRO_NOISE_RADPS, size=3)

    return ImuMeasurement(accel_mps2=accel_body, gyro_radps=gyro_body)


def simulate_gps(state: PointMassState) -> GpsMeasurement:
    """
    Simulate GPS position and velocity in NED frame.
    """
    pos = np.array([state.x_m, state.y_m, state.z_m], dtype=float)
    # Approximate velocity in NED from speed and heading
    vx = state.v_mps * math.cos(state.psi_rad)
    vy = state.v_mps * math.sin(state.psi_rad)
    vz = 0.0
    vel = np.array([vx, vy, vz], dtype=float)

    pos_meas = pos + np.random.normal(0.0, GPS_POS_NOISE_M, size=3)
    vel_meas = vel + np.random.normal(0.0, GPS_VEL_NOISE_MPS, size=3)

    return GpsMeasurement(position_ned_m=pos_meas, velocity_ned_mps=vel_meas)


def simulate_baro(state: PointMassState) -> BaroMeasurement:
    """
    Simulate a barometric altitude measurement (positive up).
    """
    true_alt_up = -state.z_m  # z is down, so altitude is -z
    alt_meas = true_alt_up + float(np.random.normal(0.0, BARO_ALT_NOISE_M))
    return BaroMeasurement(altitude_m=alt_meas)