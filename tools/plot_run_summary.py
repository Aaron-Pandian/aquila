"""
Aquila: final run summary plot + aligned bundle export (truth/GPS/estimate/commands/debug).

Usage (from repo root):
  python -m tools.plot_run_summary
  python -m tools.plot_run_summary --export logs/run_bundle.npz
  python -m tools.plot_run_summary --no-show --export logs/run_bundle.npz

This script is intentionally structured like an "offline playback" pipeline:
- Load logs
- Align everything to a single timeline
- Create a bundle (NPZ) that later animation / Gazebo / ROS playback can consume
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt


# ---------------------------
# CSV loading
# ---------------------------

def _load_csv(path: Path) -> Dict[str, np.ndarray]:
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows: List[Dict[str, str]] = list(reader)

    if not rows:
        raise ValueError(f"File is empty: {path}")

    cols = list(rows[0].keys())
    out: Dict[str, List[float]] = {c: [] for c in cols}

    for r in rows:
        for c in cols:
            s = r.get(c, "")
            try:
                out[c].append(float(s))
            except Exception:
                # tolerate blanks / non-numeric
                out[c].append(float("nan"))

    return {c: np.asarray(v, dtype=float) for c, v in out.items()}


def _has(d: Dict[str, np.ndarray], k: str) -> bool:
    return (k in d) and (d[k] is not None) and (len(d[k]) > 0)


def _finite_last(x: np.ndarray) -> float:
    m = np.isfinite(x)
    if not np.any(m):
        return float("nan")
    return float(x[np.where(m)[0][-1]])


def _finite_minmax(x: np.ndarray) -> Tuple[float, float]:
    m = np.isfinite(x)
    if not np.any(m):
        return (-1.0, 1.0)
    return float(np.min(x[m])), float(np.max(x[m]))


def _pad_limits(lo: float, hi: float, frac: float = 0.08) -> Tuple[float, float]:
    if not np.isfinite(lo) or not np.isfinite(hi):
        return (-1.0, 1.0)
    if abs(hi - lo) < 1e-9:
        return (lo - 1.0, hi + 1.0)
    d = (hi - lo) * frac
    return (lo - d, hi + d)


# ---------------------------
# Time alignment
# ---------------------------

def _ensure_sorted(t: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if len(t) != len(y):
        n = min(len(t), len(y))
        t = t[:n]
        y = y[:n]
    if len(t) < 2:
        return t, y
    if np.all(np.diff(t) >= 0):
        return t, y
    idx = np.argsort(t)
    return t[idx], y[idx]


def _interp_to(base_t: np.ndarray, src_t: np.ndarray, src_y: np.ndarray) -> np.ndarray:
    """
    Linear interpolate src_y(src_t) onto base_t.
    Handles NaNs by interpolating only over finite samples.
    """
    src_t, src_y = _ensure_sorted(src_t, src_y)
    m = np.isfinite(src_t) & np.isfinite(src_y)
    if np.count_nonzero(m) < 2:
        return np.full_like(base_t, np.nan, dtype=float)

    tt = src_t[m]
    yy = src_y[m]

    # guard if base_t extends beyond src range
    yb = np.interp(base_t, tt, yy, left=yy[0], right=yy[-1])
    return yb


def _nearest_to(base_t: np.ndarray, src_t: np.ndarray, src_y: np.ndarray) -> np.ndarray:
    """
    Nearest-neighbor sample src_y(src_t) onto base_t (good for discrete mode).
    """
    src_t, src_y = _ensure_sorted(src_t, src_y)
    m = np.isfinite(src_t) & np.isfinite(src_y)
    if np.count_nonzero(m) < 1:
        return np.full_like(base_t, np.nan, dtype=float)

    tt = src_t[m]
    yy = src_y[m]

    idx = np.searchsorted(tt, base_t, side="left")
    idx = np.clip(idx, 0, len(tt) - 1)

    # compare with previous sample to pick nearer
    prev = np.clip(idx - 1, 0, len(tt) - 1)
    use_prev = (np.abs(base_t - tt[prev]) <= np.abs(base_t - tt[idx]))
    idx = np.where(use_prev, prev, idx)

    return yy[idx].astype(float)


# ---------------------------
# Mode names
# ---------------------------

@dataclass
class ModeNames:
    mapping: Dict[int, str]

    @staticmethod
    def default() -> "ModeNames":
        return ModeNames(mapping={
            0: "STANDBY",
            1: "CLIMB",
            2: "CRUISE",
            3: "RTL",
            4: "FAILSAFE",
        })

    def name(self, v: float) -> str:
        if not np.isfinite(v):
            return "?"
        i = int(round(float(v)))
        return self.mapping.get(i, str(i))


# ---------------------------
# Bundle build
# ---------------------------

def build_bundle(
    sim_sensors: Dict[str, np.ndarray],
    sim_closed: Dict[str, np.ndarray],
    fsw: Dict[str, np.ndarray],
) -> Dict[str, np.ndarray]:
    """
    Create a single time-aligned bundle dict.

    Base timeline:
      - Prefer sim_closedloop t_s (truth timeline)
      - else fall back to sim_sensors t_s
      - else fall back to fsw t
    """
    if _has(sim_closed, "t_s"):
        t = sim_closed["t_s"]
    elif _has(sim_sensors, "t_s"):
        t = sim_sensors["t_s"]
    elif _has(fsw, "t"):
        t = fsw["t"]
    else:
        raise ValueError("No time column found in any log.")

    t = np.asarray(t, dtype=float)
    if len(t) < 2:
        raise ValueError("Timeline too short.")

    # ----- Truth (from sim_closedloop) -----
    truth_n = sim_closed.get("n_m")
    truth_e = sim_closed.get("e_m")
    truth_d = sim_closed.get("d_m")
    truth_alt = (-truth_d) if truth_d is not None else None

    truth_v = sim_closed.get("v_mps")
    truth_theta = sim_closed.get("theta_rad")
    truth_q = sim_closed.get("q_radps")

    elevator_applied = sim_closed.get("elevator_applied")
    truth_cmd_e = sim_closed.get("cmd_elevator")
    truth_cmd_t = sim_closed.get("cmd_throttle")
    truth_cmd_a = sim_closed.get("cmd_aileron")
    truth_mode = None  # sim_closedloop currently doesn’t log mode

    # ----- GPS (from sim_sensors) -----
    if _has(sim_sensors, "t_s"):
        ts = sim_sensors["t_s"]
    else:
        ts = t

    gps_n_s = _interp_to(t, ts, sim_sensors.get("gps_x_m", np.full_like(ts, np.nan)))
    gps_e_s = _interp_to(t, ts, sim_sensors.get("gps_y_m", np.full_like(ts, np.nan)))
    gps_d_s = _interp_to(t, ts, sim_sensors.get("gps_z_m", np.full_like(ts, np.nan)))
    gps_alt_s = -gps_d_s

    # ----- Estimate + FSW debug (from fsw_output) -----
    if _has(fsw, "t"):
        tf = fsw["t"]
    else:
        tf = t

    est_n = _interp_to(t, tf, fsw.get("est_pn", np.full_like(tf, np.nan)))
    est_e = _interp_to(t, tf, fsw.get("est_pe", np.full_like(tf, np.nan)))
    est_d = _interp_to(t, tf, fsw.get("est_pd", np.full_like(tf, np.nan)))
    est_alt = -est_d

    est_vn = _interp_to(t, tf, fsw.get("est_vn", np.full_like(tf, np.nan)))
    est_ve = _interp_to(t, tf, fsw.get("est_ve", np.full_like(tf, np.nan)))
    est_vd = _interp_to(t, tf, fsw.get("est_vd", np.full_like(tf, np.nan)))
    est_v = np.sqrt(est_vn**2 + est_ve**2 + est_vd**2)

    # Use dbg_est_theta_rad / dbg_q_radps if available (best for sign debug)
    est_theta = _interp_to(t, tf, fsw.get("dbg_est_theta_rad", np.full_like(tf, np.nan)))
    est_q = _interp_to(t, tf, fsw.get("dbg_q_radps", np.full_like(tf, np.nan)))

    cmd_a = _interp_to(t, tf, fsw.get("cmd_aileron", np.full_like(tf, np.nan)))
    cmd_e = _interp_to(t, tf, fsw.get("cmd_elevator", np.full_like(tf, np.nan)))
    cmd_t = _interp_to(t, tf, fsw.get("cmd_throttle", np.full_like(tf, np.nan)))

    mode = _nearest_to(t, tf, fsw.get("mode", np.full_like(tf, np.nan)))

    dbg_alt_cmd = _interp_to(t, tf, fsw.get("dbg_alt_cmd_m", np.full_like(tf, np.nan)))
    dbg_alt_m = _interp_to(t, tf, fsw.get("dbg_alt_m", np.full_like(tf, np.nan)))
    dbg_alt_err = _interp_to(t, tf, fsw.get("dbg_alt_err_m", np.full_like(tf, np.nan)))
    dbg_elev_unsat = _interp_to(t, tf, fsw.get("dbg_elevator_unsat", np.full_like(tf, np.nan)))
    dbg_elev_times_q = _interp_to(t, tf, fsw.get("dbg_elev_times_q", np.full_like(tf, np.nan)))
    dbg_vd = _interp_to(t, tf, fsw.get("dbg_vd_mps", np.full_like(tf, np.nan)))
    dbg_alt_dot = _interp_to(t, tf, fsw.get("dbg_alt_dot_mps", np.full_like(tf, np.nan)))

    # Also keep fsw-side GPS if you want (optional)
    gps_n_f = _interp_to(t, tf, fsw.get("gps_pn", np.full_like(tf, np.nan)))
    gps_e_f = _interp_to(t, tf, fsw.get("gps_pe", np.full_like(tf, np.nan)))
    gps_d_f = _interp_to(t, tf, fsw.get("gps_pd", np.full_like(tf, np.nan)))
    gps_alt_f = -gps_d_f

    # Bundle dict (keys expected by animate_run_2d.py)
    bundle: Dict[str, np.ndarray] = {
        "t": t,

        # Truth
        "truth_n": truth_n if truth_n is not None else np.full_like(t, np.nan),
        "truth_e": truth_e if truth_e is not None else np.full_like(t, np.nan),
        "truth_alt": truth_alt if truth_alt is not None else np.full_like(t, np.nan),
        "truth_v": truth_v if truth_v is not None else np.full_like(t, np.nan),
        "truth_theta": truth_theta if truth_theta is not None else np.full_like(t, np.nan),
        "truth_q": truth_q if truth_q is not None else np.full_like(t, np.nan),

        # Estimate
        "est_n": est_n,
        "est_e": est_e,
        "est_alt": est_alt,
        "est_v": est_v,
        "est_theta": est_theta,
        "est_q": est_q,

        # GPS (sensors timeline)
        "gps_n_s": gps_n_s,
        "gps_e_s": gps_e_s,
        "gps_alt_s": gps_alt_s,

        # GPS (fsw timeline, optional)
        "gps_n_f": gps_n_f,
        "gps_e_f": gps_e_f,
        "gps_alt_f": gps_alt_f,

        # Commands / applied
        "cmd_a": cmd_a,
        "cmd_e": cmd_e,
        "cmd_t": cmd_t,
        "elevator_applied": elevator_applied if elevator_applied is not None else np.full_like(t, np.nan),

        # FSW debug
        "mode": mode,
        "dbg_alt_cmd": dbg_alt_cmd,
        "dbg_alt_m": dbg_alt_m,
        "dbg_alt_err": dbg_alt_err,
        "dbg_elevator_unsat": dbg_elev_unsat,
        "dbg_elev_times_q": dbg_elev_times_q,
        "dbg_vd": dbg_vd,
        "dbg_alt_dot": dbg_alt_dot,
    }

    return bundle


# ---------------------------
# Plotting
# ---------------------------

def _plot_summary(bundle: Dict[str, np.ndarray]) -> Tuple[plt.Figure, plt.Figure]:
    t = bundle["t"]

    truth_n = bundle["truth_n"]; truth_e = bundle["truth_e"]; truth_alt = bundle["truth_alt"]
    est_n = bundle["est_n"]; est_e = bundle["est_e"]; est_alt = bundle["est_alt"]
    gps_n = bundle["gps_n_s"]; gps_e = bundle["gps_e_s"]; gps_alt = bundle["gps_alt_s"]

    cmd_e = bundle["cmd_e"]; cmd_a = bundle["cmd_a"]; cmd_t = bundle["cmd_t"]
    elev_applied = bundle["elevator_applied"]
    mode = bundle["mode"]

    truth_v = bundle["truth_v"]; est_v = bundle["est_v"]
    truth_theta = bundle["truth_theta"]; est_theta = bundle["est_theta"]
    truth_q = bundle["truth_q"]; est_q = bundle["est_q"]

    dbg_alt_cmd = bundle["dbg_alt_cmd"]
    dbg_alt_err = bundle["dbg_alt_err"]
    dbg_elev_unsat = bundle["dbg_elevator_unsat"]
    dbg_elev_times_q = bundle["dbg_elev_times_q"]
    dbg_alt_dot = bundle["dbg_alt_dot"]
    dbg_vd = bundle["dbg_vd"]

    # Errors
    err_xy = np.sqrt((est_n - gps_n) ** 2 + (est_e - gps_e) ** 2)
    err_alt = est_alt - gps_alt

    # ---- Figure 1 (Navigation / Estimation) ----
    fig1 = plt.figure(figsize=(15, 9))
    axs1 = fig1.subplot_mosaic(
        [
            ["map",   "map",   "alt",   "alt"],
            ["map",   "map",   "alt",   "alt"],
            ["speed", "speed", "xyerr", "xyerr"],
            ["pitch", "pitch", "alterr","alterr"],
            ["mode",  "mode",  "mode",  "mode"],
        ]
    )

    # Map
    ax = axs1["map"]
    ax.plot(truth_n, truth_e, label="Truth", alpha=0.9)
    ax.plot(est_n, est_e, label="Estimate", alpha=0.9)
    ax.plot(gps_n, gps_e, label="GPS (sim_sensors)", alpha=0.7)
    ax.set_title("Ground Track (N/E)")
    ax.set_xlabel("North [m]")
    ax.set_ylabel("East [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend(loc="upper right")

    # Altitude
    ax = axs1["alt"]
    ax.plot(t, truth_alt, label="Truth alt", alpha=0.9)
    ax.plot(t, est_alt, label="Est alt", alpha=0.9)
    ax.plot(t, gps_alt, label="GPS alt", alpha=0.7)
    if np.any(np.isfinite(dbg_alt_cmd)):
        ax.plot(t, dbg_alt_cmd, label="Alt cmd (dbg)", alpha=0.8)
    ax.set_title("Altitude vs Time")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Altitude [m]")
    ax.grid(True)
    ax.legend(loc="upper right")

    # Speed
    ax = axs1["speed"]
    if np.any(np.isfinite(truth_v)):
        ax.plot(t, truth_v, label="Truth V", alpha=0.9)
    ax.plot(t, est_v, label="Est V", alpha=0.9)
    ax.set_title("Speed")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Speed [m/s]")
    ax.grid(True)
    ax.legend(loc="upper right")

    # Pitch
    ax = axs1["pitch"]
    if np.any(np.isfinite(truth_theta)):
        ax.plot(t, truth_theta, label="Truth theta", alpha=0.9)
    if np.any(np.isfinite(est_theta)):
        ax.plot(t, est_theta, label="Est theta (dbg)", alpha=0.9)
    ax.set_title("Pitch")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Pitch [rad]")
    ax.grid(True)
    ax.legend(loc="upper right")

    # XY error
    ax = axs1["xyerr"]
    ax.plot(t, err_xy, alpha=0.9)
    ax.set_title("Horizontal Error |Est - GPS|")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("XY error [m]")
    ax.grid(True)

    # Alt error
    ax = axs1["alterr"]
    ax.plot(t, err_alt, alpha=0.9)
    ax.set_title("Altitude Error (Est - GPS)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Alt error [m]")
    ax.grid(True)

    # Mode (from FSW)
    ax = axs1["mode"]
    ax.step(t, mode, where="post", alpha=0.9)
    ax.set_title("Mode (from FSW)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("mode")
    ax.grid(True)

    fig1.tight_layout()

    # ---- Figure 2 (Controls / Debug) ----
    fig2 = plt.figure(figsize=(15, 9))
    axs2 = fig2.subplot_mosaic(
        [
            ["elev",      "elev",      "throttle", "throttle"],
            ["elev",      "elev",      "throttle", "throttle"],
            ["ail",       "ail",       "qrate",    "qrate"],
            ["altloop",   "altloop",   "sign",     "sign"],
            ["altdot",    "altdot",    "vdown",    "vdown"],
        ]
    )

    # Elevator cmd + applied + unsat
    ax = axs2["elev"]
    ax.plot(t, cmd_e, label="cmd_elevator (FSW)", alpha=0.9)
    if np.any(np.isfinite(elev_applied)):
        ax.plot(t, elev_applied, label="elevator_applied (plant)", alpha=0.9)
    if np.any(np.isfinite(dbg_elev_unsat)):
        ax.plot(t, dbg_elev_unsat, label="dbg_elevator_unsat", alpha=0.8)
    ax.set_title("Elevator")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("rad / cmd")
    ax.grid(True)
    ax.legend(loc="upper right")

    # Throttle
    ax = axs2["throttle"]
    ax.plot(t, cmd_t, label="cmd_throttle (FSW)", alpha=0.9)
    ax.set_title("Throttle")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("cmd")
    ax.grid(True)
    ax.legend(loc="upper right")

    # Aileron
    ax = axs2["ail"]
    ax.plot(t, cmd_a, label="cmd_aileron (FSW)", alpha=0.9)
    ax.set_title("Aileron")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("cmd")
    ax.grid(True)
    ax.legend(loc="upper right")

    # Pitch rate
    ax = axs2["qrate"]
    if np.any(np.isfinite(truth_q)):
        ax.plot(t, truth_q, label="truth q (sim_closedloop)", alpha=0.9)
    if np.any(np.isfinite(est_q)):
        ax.plot(t, est_q, label="est q (dbg)", alpha=0.9)
    ax.set_title("Pitch Rate q")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("rad/s")
    ax.grid(True)
    ax.legend(loc="upper right")

    # Alt loop debug
    ax = axs2["altloop"]
    if np.any(np.isfinite(dbg_alt_err)):
        ax.plot(t, dbg_alt_err, label="dbg_alt_err_m", alpha=0.9)
    ax.set_title("Altitude Loop Debug")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("m")
    ax.grid(True)
    ax.legend(loc="upper right")

    # Quick sign check
    ax = axs2["sign"]
    if np.any(np.isfinite(dbg_elev_times_q)):
        ax.plot(t, dbg_elev_times_q, label="dbg_elev_times_q", alpha=0.9)
    ax.axhline(0.0, lw=1.0)
    ax.set_title("Sign Check (elevator * q)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("rad^2/s")
    ax.grid(True)
    ax.legend(loc="upper right")

    # Altitude rate (dbg)
    ax = axs2["altdot"]
    if np.any(np.isfinite(dbg_alt_dot)):
        ax.plot(t, dbg_alt_dot, label="dbg_alt_dot_mps (+up)", alpha=0.9)
    ax.axhline(0.0, lw=1.0)
    ax.set_title("Altitude Rate (dbg)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("m/s")
    ax.grid(True)
    ax.legend(loc="upper right")

    # NED down velocity (dbg)
    ax = axs2["vdown"]
    if np.any(np.isfinite(dbg_vd)):
        ax.plot(t, dbg_vd, label="dbg_vd_mps (+down)", alpha=0.9)
    ax.axhline(0.0, lw=1.0)
    ax.set_title("Vertical Velocity vd (dbg)")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("m/s")
    ax.grid(True)
    ax.legend(loc="upper right")

    fig2.tight_layout()

    return fig1, fig2


def _print_final_snapshot(bundle: Dict[str, np.ndarray]) -> None:
    mn = ModeNames.default()

    t = bundle["t"]
    truth_n = bundle["truth_n"]; truth_e = bundle["truth_e"]; truth_alt = bundle["truth_alt"]
    est_n = bundle["est_n"]; est_e = bundle["est_e"]; est_alt = bundle["est_alt"]
    gps_n = bundle["gps_n_s"]; gps_e = bundle["gps_e_s"]; gps_alt = bundle["gps_alt_s"]
    mode = bundle["mode"]

    err_xy = np.sqrt((est_n - gps_n) ** 2 + (est_e - gps_e) ** 2)
    err_alt = est_alt - gps_alt

    tf = _finite_last(t)

    print("\n--- Final snapshot ---")
    print(f"t = {tf:.3f} s")
    print(f"Truth: N/E/Alt = {_finite_last(truth_n):.3f}, {_finite_last(truth_e):.3f}, {_finite_last(truth_alt):.3f}")
    print(f"Est:   N/E/Alt = {_finite_last(est_n):.3f}, {_finite_last(est_e):.3f}, {_finite_last(est_alt):.3f}")
    print(f"GPS:   N/E/Alt = {_finite_last(gps_n):.3f}, {_finite_last(gps_e):.3f}, {_finite_last(gps_alt):.3f}")
    print(f"XY err (Est-GPS) = {_finite_last(err_xy):.3f} m")
    print(f"Alt err (Est-GPS) = {_finite_last(err_alt):.3f} m")
    print(f"Mode = {_finite_last(mode):.3f}  ({mn.name(_finite_last(mode))})")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sensors", default="logs/sim_sensors.csv")
    ap.add_argument("--closedloop", default="logs/sim_closedloop.csv")
    ap.add_argument("--fsw", default="logs/fsw_output.csv")
    ap.add_argument("--export", default=None, help="Optional output bundle NPZ path (e.g., logs/run_bundle.npz)")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    sensors_path = (repo_root / args.sensors).resolve()
    closed_path = (repo_root / args.closedloop).resolve()
    fsw_path = (repo_root / args.fsw).resolve()

    print(f"Using sim sensors:   {sensors_path}")
    print(f"Using sim closedloop:{closed_path}")
    print(f"Using fsw output:    {fsw_path}")

    sim_sensors = _load_csv(sensors_path)
    sim_closed = _load_csv(closed_path)
    fsw = _load_csv(fsw_path)

    bundle = build_bundle(sim_sensors, sim_closed, fsw)
    _print_final_snapshot(bundle)

    if args.export:
        out = (repo_root / args.export).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(out, **bundle)
        print(f"\nSaved bundle: {out}")

    fig1, fig2 = _plot_summary(bundle)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()