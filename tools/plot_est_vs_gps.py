"""
Compare GPS (from sim_sensors.csv) vs estimated trajectory (from fsw_output.csv).

Usage (from repo root):
    python -m tools.plot_est_vs_gps
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np


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
        data[key] = np.array([float(r[key]) for r in rows], dtype=float)
    return data


def plot_est_vs_gps(sim_log: Path, fsw_log: Path) -> None:
    sim = _load_csv(sim_log)
    fsw = _load_csv(fsw_log)

    # Time – both logs have same dt and same number of samples
    t_sim = sim["t_s"]
    t_fsw = fsw["t"]

    # GPS from sim_sensors.csv (NED position)
    gps_x = sim["gps_x_m"]
    gps_y = sim["gps_y_m"]
    gps_z = sim["gps_z_m"]
    gps_alt = -gps_z  # NED z down -> altitude up

    # Estimated position from fsw_output.csv
    est_x = fsw["est_pn"]
    est_y = fsw["est_pe"]
    est_z = fsw["est_pd"]
    est_alt = -est_z

    # Simple assumption: same length & aligned; if not, you can resample later
    n = min(len(t_sim), len(t_fsw))
    t = t_sim[:n]
    gps_x, gps_y, gps_alt = gps_x[:n], gps_y[:n], gps_alt[:n]
    est_x, est_y, est_alt = est_x[:n], est_y[:n], est_alt[:n]

    # Position error in horizontal plane (norm of x,y diff)
    err_xy = np.sqrt((est_x - gps_x) ** 2 + (est_y - gps_y) ** 2)
    err_alt = est_alt - gps_alt

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    ax_xy, ax_alt, ax_err_xy, ax_err_alt = axes.ravel()

    # XY ground track
    ax_xy.plot(gps_x, gps_y, label="GPS", alpha=0.8)
    ax_xy.plot(est_x, est_y, label="Estimate", alpha=0.8)
    ax_xy.set_xlabel("x [m] (North)")
    ax_xy.set_ylabel("y [m] (East)")
    ax_xy.set_title("Ground Track: GPS vs Estimate")
    ax_xy.axis("equal")
    ax_xy.grid(True)
    ax_xy.legend()

    # Altitude vs time
    ax_alt.plot(t, gps_alt, label="GPS alt")
    ax_alt.plot(t, est_alt, label="Est alt", alpha=0.8)
    ax_alt.set_xlabel("Time [s]")
    ax_alt.set_ylabel("Altitude [m]")
    ax_alt.set_title("Altitude: GPS vs Estimate")
    ax_alt.grid(True)
    ax_alt.legend()

    # Horizontal position error
    ax_err_xy.plot(t, err_xy)
    ax_err_xy.set_xlabel("Time [s]")
    ax_err_xy.set_ylabel("XY error [m]")
    ax_err_xy.set_title("Horizontal Position Error")
    ax_err_xy.grid(True)

    # Altitude error
    ax_err_alt.plot(t, err_alt)
    ax_err_alt.set_xlabel("Time [s]")
    ax_err_alt.set_ylabel("Alt error [m]")
    ax_err_alt.set_title("Altitude Error (Est - GPS)")
    ax_err_alt.grid(True)

    fig.tight_layout()
    plt.show()


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    sim_log = repo_root / "logs" / "sim_sensors.csv"
    fsw_log = repo_root / "logs" / "fsw_output.csv"

    print(f"Using sim log: {sim_log}")
    print(f"Using FSW log: {fsw_log}")
    plot_est_vs_gps(sim_log, fsw_log)


if __name__ == "__main__":
    main()