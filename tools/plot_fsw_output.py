"""
Plotting utilities for Aquila FSW output logs.

Usage (from repo root):

    python -m tools.plot_fsw_output

This script currently:
- Loads logs/fsw_output.csv
- Plots actuator commands vs time

The structure is designed so that it can be extended later for:
- Comparing estimated vs truth trajectories
- Visualizing estimator residuals
- Animations of the aircraft motion and command behavior
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt
import numpy as np


def _load_fsw_log(path: Path) -> Dict[str, np.ndarray]:
    """
    Load the FSW CSV log into numpy arrays.

    Parameters
    ----------
    path : Path
        Path to fsw_output.csv.

    Returns
    -------
    dict
        Dictionary of column name -> numpy array.
    """
    if not path.exists():
        raise FileNotFoundError(f"FSW log file not found: {path}")

    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows: List[Dict[str, str]] = list(reader)

    if not rows:
        raise ValueError(f"FSW log file is empty: {path}")

    data: Dict[str, np.ndarray] = {}
    for key in rows[0].keys():
        data[key] = np.array([float(r[key]) for r in rows], dtype=float)

    return data


def plot_actuator_commands(log_path: Path) -> None:
    """
    Plot actuator commands vs time from the FSW log.

    Parameters
    ----------
    log_path : Path
        Path to fsw_output.csv.
    """
    data = _load_fsw_log(log_path)

    t = data["t"]

    aileron  = data["cmd_aileron"]
    elevator = data["cmd_elevator"]
    rudder   = data["cmd_rudder"]
    throttle = data["cmd_throttle"]

    fig, axes = plt.subplots(2, 2, figsize=(10, 6))
    ax_ail, ax_elev, ax_rud, ax_thr = axes.ravel()

    # Aileron
    ax_ail.plot(t, aileron)
    ax_ail.set_xlabel("Time [s]")
    ax_ail.set_ylabel("Aileron cmd [-]")
    ax_ail.set_title("Aileron Command")
    ax_ail.grid(True)

    # Elevator
    ax_elev.plot(t, elevator)
    ax_elev.set_xlabel("Time [s]")
    ax_elev.set_ylabel("Elevator cmd [-]")
    ax_elev.set_title("Elevator Command")
    ax_elev.grid(True)

    # Rudder
    ax_rud.plot(t, rudder)
    ax_rud.set_xlabel("Time [s]")
    ax_rud.set_ylabel("Rudder cmd [-]")
    ax_rud.set_title("Rudder Command")
    ax_rud.grid(True)

    # Throttle
    ax_thr.plot(t, throttle)
    ax_thr.set_xlabel("Time [s]")
    ax_thr.set_ylabel("Throttle cmd [-]")
    ax_thr.set_title("Throttle Command")
    ax_thr.grid(True)

    fig.tight_layout()
    plt.show()


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    log_path = repo_root / "logs" / "fsw_output.csv"

    print(f"Using FSW log file: {log_path}")
    plot_actuator_commands(log_path)


if __name__ == "__main__":
    main()