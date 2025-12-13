"""
Check consistency between simulated "truth" kinematics and IMU measurements.

Usage (from repo root):

    python -m tools.plot_imu_consistency
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np


def _load_sim_log(path: Path) -> Dict[str, np.ndarray]:
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows: List[Dict[str, str]] = list(reader)

    if not rows:
        raise ValueError(f"File is empty: {path}")

    data: Dict[str, np.ndarray] = {}
    for key in rows[0].keys():
        data[key] = np.array([float(r[key]) for r in rows], dtype=float)
    return data


def plot_imu_consistency(sim_log: Path) -> None:
    data = _load_sim_log(sim_log)

    t = data["t_s"]
    x = data["n_m"]
    y = data["e_m"]
    z = data["d_m"]
    psi = data["psi_rad"]

    imu_ax = data["imu_ax_mps2"]
    imu_ay = data["imu_ay_mps2"]
    imu_az = data["imu_az_mps2"]
    imu_gz = data["imu_gz_radps"]

    # Unwrap heading to avoid large jumps at +/- pi
    psi_unwrapped = np.unwrap(psi)

    # Finite differences for velocities and yaw rate
    dt = np.diff(t)
    dt[dt <= 0.0] = np.min(dt[dt > 0.0])  # guard against zeros

    vx = np.diff(x) / dt
    vy = np.diff(y) / dt
    vz = np.diff(z) / dt

    # Pad to same length as original arrays
    vx = np.concatenate([[vx[0]], vx])
    vy = np.concatenate([[vy[0]], vy])
    vz = np.concatenate([[vz[0]], vz])

    ax_n = np.diff(vx) / dt
    ay_n = np.diff(vy) / dt
    az_n = np.diff(vz) / dt

    ax_n = np.concatenate([[ax_n[0]], ax_n])
    ay_n = np.concatenate([[ay_n[0]], ay_n])
    az_n = np.concatenate([[az_n[0]], az_n])

    # Truth yaw rate from psi
    yaw_rate_truth = np.diff(psi_unwrapped) / dt
    yaw_rate_truth = np.concatenate([[yaw_rate_truth[0]], yaw_rate_truth])

    # Convert NED acceleration -> body-frame specific force
    g_ned = np.array([0.0, 0.0, 9.81])
    f_body_x = np.zeros_like(t)
    for i in range(len(t)):
        a_ned = np.array([ax_n[i], ay_n[i], az_n[i]])
        # R_bn for yaw-only
        c = np.cos(psi[i])
        s = np.sin(psi[i])
        R_bn = np.array([[c, s, 0.0],
                         [-s, c, 0.0],
                         [0.0, 0.0, 1.0]])
        f_body = R_bn @ (a_ned - g_ned)
        f_body_x[i] = f_body[0]

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    ax1, ax2, ax3, ax4 = axes.ravel()

    # Yaw rate: truth vs IMU
    ax1.plot(t, yaw_rate_truth, label="truth yaw rate")
    ax1.plot(t, imu_gz, label="imu_gz", alpha=0.7)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Yaw rate [rad/s]")
    ax1.set_title("Yaw Rate: Truth vs IMU")
    ax1.grid(True)
    ax1.legend()

    # Body x specific force: truth vs IMU ax
    ax2.plot(t, f_body_x, label="truth f_body_x")
    ax2.plot(t, imu_ax, label="imu_ax", alpha=0.7)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Specific force X [m/s^2]")
    ax2.set_title("Body X Specific Force: Truth vs IMU")
    ax2.grid(True)
    ax2.legend()

    # Magnitude checks
    imu_a_mag = np.sqrt(imu_ax**2 + imu_ay**2 + imu_az**2)
    truth_a_mag = np.sqrt(ax_n**2 + ay_n**2 + az_n**2)

    ax3.plot(t, truth_a_mag, label="|a_ned| truth")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("|a_ned| [m/s^2]")
    ax3.set_title("Inertial Accel Magnitude (Truth)")
    ax3.grid(True)

    ax4.plot(t, imu_a_mag, label="|imu_accel|")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("|accel| [m/s^2]")
    ax4.set_title("IMU Accel Magnitude")
    ax4.grid(True)

    fig.tight_layout()
    plt.show()


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    sim_log = repo_root / "logs" / "sim_sensors.csv"
    print(f"Using sim log: {sim_log}")
    plot_imu_consistency(sim_log)


if __name__ == "__main__":
    main()