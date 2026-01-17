"""
Compare GPS (from sim_sensors.csv) vs estimated trajectory (from fsw_output.csv).

Usage (from repo root):
    python -m tools.plot_est_vs_gps
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


def _interp_to(t_src: np.ndarray, y_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    # numpy.interp expects increasing x
    order = np.argsort(t_src)
    t = t_src[order]
    y = y_src[order]
    return np.interp(t_dst, t, y)


def plot_est_vs_gps(sim_log: Path, fsw_log: Path) -> None:
    sim = _load_csv(sim_log)
    fsw = _load_csv(fsw_log)

    # Time
    t_sim = _pick(sim, ["t_s", "t"])
    t_fsw = _pick(fsw, ["t_s", "t", "time_s"])

    # GPS from sim_sensors.csv (NED position)
    gps_n = _pick(sim, ["gps_x_m", "gps_n_m", "gps_n", "gps_ned_n"])
    gps_e = _pick(sim, ["gps_y_m", "gps_e_m", "gps_e", "gps_ned_e"])
    gps_d = _pick(sim, ["gps_z_m", "gps_d_m", "gps_d", "gps_ned_d"])
    gps_alt = -gps_d

    # Estimated position from fsw_output.csv
    # NOTE: We try several likely logger column conventions.
    est_n = _pick(fsw, ["est_n_m", "est_pn", "est_pn_m", "pn_est", "nav_n_m", "state_n_m", "position_n_m", "pn"])
    est_e = _pick(fsw, ["est_e_m", "est_pe", "est_pe_m", "pe_est", "nav_e_m", "state_e_m", "position_e_m", "pe"])
    est_d = _pick(fsw, ["est_d_m", "est_pd", "est_pd_m", "pd_est", "nav_d_m", "state_d_m", "position_d_m", "pd"])
    est_alt = -est_d

    # Align estimates to sim time via interpolation
    est_n_i = _interp_to(t_fsw, est_n, t_sim)
    est_e_i = _interp_to(t_fsw, est_e, t_sim)
    est_alt_i = _interp_to(t_fsw, est_alt, t_sim)

    # Errors
    err_xy = np.sqrt((est_n_i - gps_n) ** 2 + (est_e_i - gps_e) ** 2)
    err_alt = est_alt_i - gps_alt

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    ax_xy, ax_alt, ax_err_xy, ax_err_alt = axes.ravel()

    # XY ground track
    ax_xy.plot(gps_n, gps_e, label="GPS", alpha=0.8)
    ax_xy.plot(est_n_i, est_e_i, label="Estimate (interp)", alpha=0.8)
    ax_xy.set_xlabel("North [m]")
    ax_xy.set_ylabel("East [m]")
    ax_xy.set_title("Ground Track: GPS vs Estimate")
    ax_xy.axis("equal")
    ax_xy.grid(True)
    ax_xy.legend()

    # Altitude vs time
    ax_alt.plot(t_sim, gps_alt, label="GPS alt")
    ax_alt.plot(t_sim, est_alt_i, label="Est alt (interp)", alpha=0.85)
    ax_alt.set_xlabel("Time [s]")
    ax_alt.set_ylabel("Altitude [m]")
    ax_alt.set_title("Altitude: GPS vs Estimate")
    ax_alt.grid(True)
    ax_alt.legend()

    # Horizontal position error
    ax_err_xy.plot(t_sim, err_xy)
    ax_err_xy.set_xlabel("Time [s]")
    ax_err_xy.set_ylabel("XY error [m]")
    ax_err_xy.set_title("Horizontal Position Error")
    ax_err_xy.grid(True)

    # Altitude error
    ax_err_alt.plot(t_sim, err_alt)
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