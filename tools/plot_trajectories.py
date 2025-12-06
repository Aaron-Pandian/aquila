"""
Plotting utilities for Aquila simulation logs.

Usage (from repo root):

    python -m tools.plot_trajectories

This script currently:
- Loads logs/sim_sensors.csv
- Plots x-y ground track and altitude vs time

The structure is designed so that it can be extended to support
animations (e.g., using matplotlib.animation.FuncAnimation) later.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np


def _load_sensor_log(path: Path) -> Dict[str, np.ndarray]:
    """
    Load the sensor CSV log into numpy arrays.

    Parameters
    ----------
    path : Path
        Path to sim_sensors.csv.

    Returns
    -------
    dict
        Dictionary of column name -> numpy array.
    """
    if not path.exists():
        raise FileNotFoundError(f"Log file not found: {path}")

    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows: List[Dict[str, str]] = list(reader)

    if not rows:
        raise ValueError(f"Log file is empty: {path}")

    # Convert each column to a numpy array of floats
    data: Dict[str, np.ndarray] = {}
    for key in rows[0].keys():
        data[key] = np.array([float(r[key]) for r in rows], dtype=float)

    return data


def plot_trajectory(log_path: Path) -> None:
    """
    Plot x-y ground track and altitude vs time from the sensor log.

    Parameters
    ----------
    log_path : Path
        Path to sim_sensors.csv.
    """
    data = _load_sensor_log(log_path)

    t = data["t_s"]
    x = data["x_m"]
    y = data["y_m"]
    z = data["z_m"]

    alt = -z  # NED convention: z down, altitude = -z

    fig, (ax_xy, ax_alt) = plt.subplots(1, 2, figsize=(10, 4))

    # Ground track
    ax_xy.plot(x, y)
    ax_xy.set_xlabel("x [m] (North)")
    ax_xy.set_ylabel("y [m] (East)")
    ax_xy.set_title("Ground Track")
    ax_xy.axis("equal")
    ax_xy.grid(True)

    # Altitude vs time
    ax_alt.plot(t, alt)
    ax_alt.set_xlabel("Time [s]")
    ax_alt.set_ylabel("Altitude [m]")
    ax_alt.set_title("Altitude vs Time")
    ax_alt.grid(True)

    fig.tight_layout()
    plt.show()


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    log_path = repo_root / "logs" / "sim_sensors.csv"

    print(f"Using log file: {log_path}")
    plot_trajectory(log_path)


if __name__ == "__main__":
    main()