"""
Check consistency between simulated "truth" kinematics and IMU measurements.

Usage (from repo root):
    python -m tools.plot_imu_consistency

Notes:
- sim_sensors.csv includes truth position (n/e/d) and psi (yaw), but not roll/pitch.
- Therefore, any body-frame comparison from inertial truth is "yaw-only" approximate.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List, Iterable, Optional

import matplotlib.pyplot as plt
import numpy as np


def _to_float(s: str) -> float:
    try:
        return float(s)
    except Exception:
        return float("nan")


def _load_csv(path: Path) -> Dict[str, np.ndarray]:
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows: List[Dict[str, str]] = list(reader)

    if not rows:
        raise ValueError(f"File is empty: {path}")

    data: Dict[str, np.ndarray] = {}
    for key in rows[0].keys():
        data[key] = np.array([_to_float(r.get(key, "")) for r in rows], dtype=float)
    return data


def _pick(data: Dict[str, np.ndarray], names: Iterable[str], *, required: bool = True) -> Optional[np.ndarray]:
    for n in names:
        if n in data:
            return data[n]
    if required:
        available = ", ".join(sorted(data.keys()))
        raise KeyError(f"Missing required column. Tried {list(names)}. Available: {available}")
    return None


def plot_imu_consistency(sim_log: Path) -> None:
    data = _load_csv(sim_log)

    t = _pick(data, ["t_s", "t"])
    n = _pick(data, ["n_m", "x_m"])
    e = _pick(data, ["e_m", "y_m"])
    d = _pick(data, ["d_m", "z_m"])
    psi = _pick(data, ["psi_rad", "psi"])

    imu_ax = _pick(data, ["imu_ax_mps2", "imu_ax"])
    imu_ay = _pick(data, ["imu_ay_mps2", "imu_ay"])
    imu_az = _pick(data, ["imu_az_mps2", "imu_az"])
    imu_gz = _pick(data, ["imu_gz_radps", "imu_gz"])

    # Derivatives using gradient (handles nonuniform dt)
    psi_u = np.unwrap(psi)
    yaw_rate_truth = np.gradient(psi_u, t)

    vn = np.gradient(n, t)
    ve = np.gradient(e, t)
    vd = np.gradient(d, t)

    an = np.gradient(vn, t)
    ae = np.gradient(ve, t)
    ad = np.gradient(vd, t)

    # Approx "yaw-only" NED->body transform for specific force comparison
    # f_body ~= R_bn(psi) * (a_ned - g_ned)
    g_ned = np.array([0.0, 0.0, 9.81])  # +down in NED
    fbx = np.zeros_like(t)
    fby = np.zeros_like(t)
    fbz = np.zeros_like(t)

    for i in range(len(t)):
        a_ned = np.array([an[i], ae[i], ad[i]]) - g_ned
        c = np.cos(psi[i])
        s = np.sin(psi[i])
        # body <- NED (yaw-only)
        R_bn = np.array([[c, s, 0.0],
                         [-s, c, 0.0],
                         [0.0, 0.0, 1.0]])
        f_b = R_bn @ a_ned
        fbx[i], fby[i], fbz[i] = f_b

    imu_a_mag = np.sqrt(imu_ax**2 + imu_ay**2 + imu_az**2)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax1, ax2, ax3, ax4 = axes.ravel()

    # Yaw rate: truth vs IMU
    ax1.plot(t, yaw_rate_truth, label="truth yaw_rate (d/dt unwrap(psi))")
    ax1.plot(t, imu_gz, label="imu_gz", alpha=0.75)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Yaw rate [rad/s]")
    ax1.set_title("Yaw Rate: Truth vs IMU")
    ax1.grid(True)
    ax1.legend()

    # Forward-axis specific force (approx)
    ax2.plot(t, fbx, label="truth f_body_x (yaw-only approx)")
    ax2.plot(t, imu_ax, label="imu_ax", alpha=0.75)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Specific force [m/s^2]")
    ax2.set_title("Body X Specific Force (Approx) vs IMU")
    ax2.grid(True)
    ax2.legend()

    # Lateral/vertical (approx) to see sign sanity
    ax3.plot(t, fby, label="truth f_body_y (yaw-only approx)")
    ax3.plot(t, imu_ay, label="imu_ay", alpha=0.75)
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Specific force [m/s^2]")
    ax3.set_title("Body Y Specific Force (Approx) vs IMU")
    ax3.grid(True)
    ax3.legend()

    ax4.plot(t, imu_a_mag, label="|imu_accel|")
    ax4.axhline(9.81, linestyle="--", label="9.81 (ref)")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("|accel| [m/s^2]")
    ax4.set_title("IMU Accel Magnitude (Sanity Check)")
    ax4.grid(True)
    ax4.legend()

    fig.tight_layout()
    plt.show()


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    sim_log = repo_root / "logs" / "sim_sensors.csv"
    print(f"Using sim log: {sim_log}")
    plot_imu_consistency(sim_log)


if __name__ == "__main__":
    main()