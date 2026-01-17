"""
Plotting utilities for Aquila FSW output logs.

Usage (from repo root):
    python -m tools.plot_fsw_output

Default behavior:
- Loads logs/fsw_output.csv
- Produces a comprehensive summary (commands + mode + altitude loop debug if available)

Optional:
    python -m tools.plot_fsw_output --commands-only
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Iterable, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


def _to_float(s: str) -> float:
    try:
        return float(s)
    except Exception:
        return float("nan")


def _load_csv(path: Path) -> Dict[str, np.ndarray]:
    if not path.exists():
        raise FileNotFoundError(f"FSW log file not found: {path}")

    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows: List[Dict[str, str]] = list(reader)

    if not rows:
        raise ValueError(f"FSW log file is empty: {path}")

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


def _maybe(data: Dict[str, np.ndarray], names: Iterable[str]) -> Optional[np.ndarray]:
    return _pick(data, names, required=False)


def plot_actuator_commands(data: Dict[str, np.ndarray]) -> None:
    t = _pick(data, ["t_s", "t", "time_s"])

    aileron  = _pick(data, ["cmd_aileron"], required=False)
    elevator = _pick(data, ["cmd_elevator"], required=False)
    rudder   = _pick(data, ["cmd_rudder"], required=False)
    throttle = _pick(data, ["cmd_throttle"], required=False)

    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    ax_ail, ax_elev, ax_rud, ax_thr = axes.ravel()

    def _plot_or_note(ax, y, title, ylabel):
        if y is None:
            ax.text(0.5, 0.5, f"Missing {title}", ha="center", va="center")
            ax.set_title(title)
            ax.grid(True)
            return
        ax.plot(t, y)
        ax.set_title(title)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(ylabel)
        ax.grid(True)

    _plot_or_note(ax_ail, aileron,  "Aileron Command",  "cmd [-]")
    _plot_or_note(ax_elev, elevator, "Elevator Command", "cmd [-]")
    _plot_or_note(ax_rud, rudder,   "Rudder Command",   "cmd [-]")
    _plot_or_note(ax_thr, throttle, "Throttle Command", "cmd [-]")

    fig.tight_layout()
    plt.show()


def plot_mode_and_altitude_debug(data: Dict[str, np.ndarray]) -> None:
    """
    Plots Mode (if present) and the altitude loop debug channels (if present).
    This is where your new logger fields should show up.
    """
    t = _pick(data, ["t_s", "t", "time_s"])

    mode = _maybe(data, ["mode_index", "mode", "flight_mode"])

    # Prefer your explicit debug fields first
    alt_cmd = _maybe(data, ["dbg_alt_cmd_m", "alt_cmd_m", "target_alt_m"])
    alt_est = _maybe(data, ["dbg_alt_m", "alt_est_m", "est_alt_m", "alt_m"])
    alt_err = _maybe(data, ["dbg_alt_err_m", "alt_err_m"])
    elev_unsat = _maybe(data, ["dbg_elevator_unsat", "elev_unsat", "elevator_unsat"])

    # Helpful to plot these too if present
    alt_dot = _maybe(data, ["dbg_alt_dot_mps", "alt_dot_mps", "vz_up_mps"])
    vd = _maybe(data, ["dbg_vd_mps", "vd_mps"])
    q = _maybe(data, ["dbg_q_radps", "q_radps"])
    elev = _maybe(data, ["cmd_elevator"])
    elev_times_q = _maybe(data, ["dbg_elev_times_q"])

    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    ax1, ax2, ax3, ax4, ax5, ax6 = axes.ravel()

    # Mode
    if mode is not None:
        ax1.step(t, mode, where="post")
        ax1.set_title("Flight Mode Index")
    else:
        ax1.text(0.5, 0.5, "No mode column found", ha="center", va="center")
        ax1.set_title("Flight Mode Index")
    ax1.set_xlabel("Time [s]")
    ax1.grid(True)

    # Altitude loop
    if alt_est is not None:
        if alt_cmd is not None:
            ax2.plot(t, alt_cmd, label="alt_cmd")
        ax2.plot(t, alt_est, label="alt_est", alpha=0.85)
        ax2.set_title("Altitude (Command vs Estimate)")
        ax2.legend()
    else:
        ax2.text(0.5, 0.5, "No altitude debug columns found", ha="center", va="center")
        ax2.set_title("Altitude (Command vs Estimate)")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Altitude [m]")
    ax2.grid(True)

    # Alt error
    if alt_err is not None:
        ax3.plot(t, alt_err)
        ax3.set_title("Altitude Error (cmd - est)")
    else:
        ax3.text(0.5, 0.5, "No alt error column found", ha="center", va="center")
        ax3.set_title("Altitude Error (cmd - est)")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("m")
    ax3.grid(True)

    # Elevator unsat vs cmd elevator
    if elev is not None:
        ax4.plot(t, elev, label="cmd_elevator")
    if elev_unsat is not None:
        ax4.plot(t, elev_unsat, label="elev_unsat", alpha=0.85)
    if elev is None and elev_unsat is None:
        ax4.text(0.5, 0.5, "No elevator columns found", ha="center", va="center")
    ax4.set_title("Elevator: cmd vs unsat (if available)")
    ax4.set_xlabel("Time [s]")
    ax4.grid(True)
    ax4.legend()

    # Vertical rates
    if alt_dot is not None:
        ax5.plot(t, alt_dot, label="alt_dot (+up)")
    if vd is not None:
        ax5.plot(t, vd, label="vd (+down)", alpha=0.85)
    if alt_dot is None and vd is None:
        ax5.text(0.5, 0.5, "No vertical-rate debug columns found", ha="center", va="center")
    ax5.set_title("Vertical Rate Debug")
    ax5.set_xlabel("Time [s]")
    ax5.grid(True)
    ax5.legend()

    # q and elev*q sign check
    if q is not None:
        ax6.plot(t, q, label="q_radps")
    if elev_times_q is not None:
        ax6.plot(t, elev_times_q, label="elev*q", alpha=0.85)
    if q is None and elev_times_q is None:
        ax6.text(0.5, 0.5, "No q/elev*q debug columns found", ha="center", va="center")
    ax6.set_title("Pitch-rate / Sign Check")
    ax6.set_xlabel("Time [s]")
    ax6.grid(True)
    ax6.legend()

    fig.tight_layout()
    plt.show()


def plot_attitude_debug(data: Dict[str, np.ndarray]) -> None:
    t = _pick(data, ["t_s", "t", "time_s"])

    phi = _maybe(data, ["dbg_est_phi_rad", "est_phi_rad", "phi_rad"])
    theta = _maybe(data, ["dbg_est_theta_rad", "est_theta_rad", "theta_rad"])
    psi = _maybe(data, ["dbg_est_psi_rad", "est_psi_rad", "psi_rad"])

    if phi is None and theta is None and psi is None:
        return  # nothing to plot

    fig, ax = plt.subplots(1, 1, figsize=(12, 4))
    if phi is not None:
        ax.plot(t, phi, label="phi (roll)")
    if theta is not None:
        ax.plot(t, theta, label="theta (pitch)")
    if psi is not None:
        ax.plot(t, np.unwrap(psi), label="psi (yaw, unwrap)")
    ax.set_title("Estimated Attitude (from FSW log debug if present)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("[rad]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fsw", default=None, help="Path to fsw_output.csv (default: logs/fsw_output.csv)")
    ap.add_argument("--commands-only", action="store_true", help="Only plot actuator commands")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    log_path = Path(args.fsw) if args.fsw else (repo_root / "logs" / "fsw_output.csv")

    print(f"Using FSW log file: {log_path}")
    data = _load_csv(log_path)

    if args.commands_only:
        plot_actuator_commands(data)
        return

    # Comprehensive summary
    plot_actuator_commands(data)
    plot_mode_and_altitude_debug(data)
    plot_attitude_debug(data)


if __name__ == "__main__":
    main()