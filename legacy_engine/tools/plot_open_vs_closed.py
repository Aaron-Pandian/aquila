"""
Compare open-loop vs closed-loop trajectories.

Open-loop:   logs/sim_sensors.csv   (Python autopilot drives plant)
Closed-loop: logs/sim_closedloop.csv (FSW commands drive plant)

Usage (from repo root):

    python -m tools.plot_open_vs_closed
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


def plot_open_vs_closed(open_log: Path, closed_log: Path) -> None:
    open_data = _load_csv(open_log)
    closed_data = _load_csv(closed_log)

    # Open-loop truth
    t_open = open_data["t_s"]
    x_open = open_data["n_m"]
    y_open = open_data["e_m"]
    z_open = open_data["d_m"]
    alt_open = -z_open  # NED z down

    # Closed-loop truth
    t_closed = closed_data["t_s"]
    x_closed = closed_data["n_m"]
    y_closed = closed_data["e_m"]
    z_closed = closed_data["d_m"]
    alt_closed = -z_closed

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    ax_xy, ax_alt = axes

    # Ground track
    ax_xy.plot(x_open, y_open, label="Open-loop", alpha=0.7)
    ax_xy.plot(x_closed, y_closed, label="Closed-loop", alpha=0.7)
    ax_xy.set_xlabel("x [m] (North)")
    ax_xy.set_ylabel("y [m] (East)")
    ax_xy.set_title("Ground Track: Open vs Closed")
    ax_xy.axis("equal")
    ax_xy.grid(True)
    ax_xy.legend()

    # Altitude vs time
    ax_alt.plot(t_open, alt_open, label="Open-loop", alpha=0.7)
    ax_alt.plot(t_closed, alt_closed, label="Closed-loop", alpha=0.7)
    ax_alt.set_xlabel("Time [s]")
    ax_alt.set_ylabel("Altitude [m]")
    ax_alt.set_title("Altitude: Open vs Closed")
    ax_alt.grid(True)
    ax_alt.legend()

    fig.tight_layout()
    plt.show()


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    open_log = repo_root / "logs" / "sim_sensors.csv"
    closed_log = repo_root / "logs" / "sim_closedloop.csv"

    print(f"Open-loop log:   {open_log}")
    print(f"Closed-loop log: {closed_log}")
    plot_open_vs_closed(open_log, closed_log)


if __name__ == "__main__":
    main()