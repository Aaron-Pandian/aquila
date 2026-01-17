"""
Plotting utilities for Aquila simulation logs.

Usage (from repo root):
    python -m tools.plot_trajectories

This script:
- Loads logs/sim_sensors.csv
- Plots:
  - N/E ground track
  - Altitude (truth + GPS + baro)
  - Speed
  - Heading
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
        raise FileNotFoundError(f"Log file not found: {path}")

    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows: List[Dict[str, str]] = list(reader)

    if not rows:
        raise ValueError(f"Log file is empty: {path}")

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


def plot_trajectory(log_path: Path) -> None:
    data = _load_csv(log_path)

    t = _pick(data, ["t_s", "t"])

    # Truth position (preferred): n/e/d
    n = _pick(data, ["n_m", "x_m", "pn", "N"])
    e = _pick(data, ["e_m", "y_m", "pe", "E"])
    d = _pick(data, ["d_m", "z_m", "pd", "D"])

    alt_truth = -d  # NED: D positive down -> altitude positive up

    # Optional altitude sources
    gps_d = _pick(data, ["gps_z_m", "gps_d_m", "gps_d"], required=False)
    gps_alt = (-gps_d) if gps_d is not None else None

    baro_alt = _pick(data, ["baro_alt_m", "baro_alt"], required=False)

    v = _pick(data, ["v_mps", "V_mps", "speed_mps"], required=False)
    psi = _pick(data, ["psi_rad", "psi"], required=False)

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    ax_xy, ax_alt, ax_v, ax_psi = axes.ravel()

    # Ground track
    ax_xy.plot(n, e)
    ax_xy.set_xlabel("North [m]")
    ax_xy.set_ylabel("East [m]")
    ax_xy.set_title("Ground Track (Truth)")
    ax_xy.axis("equal")
    ax_xy.grid(True)

    # Altitude vs time
    ax_alt.plot(t, alt_truth, label="truth alt (-d)")
    if gps_alt is not None:
        ax_alt.plot(t, gps_alt, label="GPS alt", alpha=0.8)
    if baro_alt is not None:
        ax_alt.plot(t, baro_alt, label="Baro alt", alpha=0.8)
    ax_alt.set_xlabel("Time [s]")
    ax_alt.set_ylabel("Altitude [m]")
    ax_alt.set_title("Altitude vs Time")
    ax_alt.grid(True)
    ax_alt.legend()

    # Speed
    if v is not None:
        ax_v.plot(t, v)
        ax_v.set_ylabel("Speed [m/s]")
        ax_v.set_title("Speed")
    else:
        ax_v.text(0.5, 0.5, "No speed column found", ha="center", va="center")
    ax_v.set_xlabel("Time [s]")
    ax_v.grid(True)

    # Heading
    if psi is not None:
        ax_psi.plot(t, np.unwrap(psi))
        ax_psi.set_ylabel("Heading [rad] (unwrapped)")
        ax_psi.set_title("Heading")
    else:
        ax_psi.text(0.5, 0.5, "No heading column found", ha="center", va="center")
    ax_psi.set_xlabel("Time [s]")
    ax_psi.grid(True)

    fig.tight_layout()
    plt.show()


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    log_path = repo_root / "logs" / "sim_sensors.csv"
    print(f"Using log file: {log_path}")
    plot_trajectory(log_path)


if __name__ == "__main__":
    main()