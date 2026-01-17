"""
Aquila: 2D run animation playback (offline bundle -> matplotlib animation)

Usage:
  python3 -m tools.animate_run_2d --bundle logs/run_bundle.npz
  python3 -m tools.animate_run_2d --bundle logs/run_bundle.npz --fps 30
  python3 -m tools.animate_run_2d --bundle logs/run_bundle.npz --follow
  python3 -m tools.animate_run_2d --bundle logs/run_bundle.npz --save_mp4 logs/run_anim.mp4

Bundle format:
  This script expects NPZ keys produced by tools.plot_run_summary (run_bundle.npz), e.g.
    t
    truth_n, truth_e, truth_alt
    est_n, est_e, est_alt
    gps_n_s, gps_e_s, gps_alt_s (preferred)
    gps_n_f, gps_e_f, gps_alt_f (fallback)
    cmd_e, cmd_t, cmd_a
    mode
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple, List

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ---------------------------
# Helpers
# ---------------------------

def load_npz(path: Path) -> Dict[str, np.ndarray]:
    if not path.exists():
        raise FileNotFoundError(f"Bundle not found: {path}")
    npz = np.load(path)
    return {k: np.asarray(npz[k]) for k in npz.files}


def get(bundle: Dict[str, np.ndarray], key: str) -> Optional[np.ndarray]:
    v = bundle.get(key, None)
    if v is None:
        return None
    v = np.asarray(v)
    if v.size == 0:
        return None
    return v


def pick(bundle: Dict[str, np.ndarray], primary: str, fallback: str) -> Optional[np.ndarray]:
    v = get(bundle, primary)
    if v is None:
        v = get(bundle, fallback)
    return v


def clip_all_to_min_len(arrs: List[Optional[np.ndarray]]) -> List[Optional[np.ndarray]]:
    lens = [len(a) for a in arrs if a is not None]
    if not lens:
        return arrs
    n = min(lens)
    out: List[Optional[np.ndarray]] = []
    for a in arrs:
        out.append(a[:n] if a is not None else None)
    return out


def finite_bounds(*arrs: np.ndarray) -> Tuple[float, float, float, float]:
    """
    Returns (xmin, xmax, ymin, ymax) over all provided arrays, ignoring NaNs.
    Expects pairs (x,y,x,y,...) or just concatenated x then y is fine as long as consistent use.
    """
    xs = []
    ys = []
    for i, a in enumerate(arrs):
        if i % 2 == 0:
            xs.append(a)
        else:
            ys.append(a)
    x = np.concatenate([z[np.isfinite(z)] for z in xs if z is not None and np.any(np.isfinite(z))]) if xs else np.array([0.0])
    y = np.concatenate([z[np.isfinite(z)] for z in ys if z is not None and np.any(np.isfinite(z))]) if ys else np.array([0.0])
    if x.size == 0: x = np.array([0.0])
    if y.size == 0: y = np.array([0.0])

    xmin, xmax = float(np.min(x)), float(np.max(x))
    ymin, ymax = float(np.min(y)), float(np.max(y))
    return xmin, xmax, ymin, ymax


def pad_limits(lo: float, hi: float, frac: float = 0.10) -> Tuple[float, float]:
    if not np.isfinite(lo) or not np.isfinite(hi):
        return (-1.0, 1.0)
    if abs(hi - lo) < 1e-9:
        return (lo - 1.0, hi + 1.0)
    d = (hi - lo) * frac
    return (lo - d, hi + d)


def heading_from_track(n: np.ndarray, e: np.ndarray) -> np.ndarray:
    """
    Approximate heading (psi) from track derivative: atan2(dE/dt, dN/dt).
    Returns radians, same length as inputs.
    """
    dn = np.gradient(n)
    de = np.gradient(e)
    psi = np.arctan2(de, dn)
    # unwrap for smooth arrow rotation
    return np.unwrap(psi)


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
# Animation core
# ---------------------------

def animate_run_2d(
    bundle: Dict[str, np.ndarray],
    fps: int = 30,
    speed: float = 1.0,
    skip: int = 1,
    tail_s: float = 4.0,
    follow: bool = False,
    show_truth: bool = True,
    show_est: bool = True,
    show_gps: bool = True,
) -> FuncAnimation:
    mn = ModeNames.default()

    t = get(bundle, "t")
    if t is None:
        raise ValueError("Bundle is missing key 't'.")

    # Preferred GPS: sensors timeline; fallback: fsw-embedded GPS
    gps_n = pick(bundle, "gps_n_s", "gps_n_f")
    gps_e = pick(bundle, "gps_e_s", "gps_e_f")
    gps_alt = pick(bundle, "gps_alt_s", "gps_alt_f")

    truth_n = get(bundle, "truth_n")
    truth_e = get(bundle, "truth_e")
    truth_alt = get(bundle, "truth_alt")

    est_n = get(bundle, "est_n")
    est_e = get(bundle, "est_e")
    est_alt = get(bundle, "est_alt")

    cmd_e = get(bundle, "cmd_e")
    cmd_t = get(bundle, "cmd_t")
    cmd_a = get(bundle, "cmd_a")
    mode = get(bundle, "mode")

    # Clip to same length (defensive)
    t, gps_n, gps_e, gps_alt, truth_n, truth_e, truth_alt, est_n, est_e, est_alt, cmd_e, cmd_t, cmd_a, mode = clip_all_to_min_len(
        [t, gps_n, gps_e, gps_alt, truth_n, truth_e, truth_alt, est_n, est_e, est_alt, cmd_e, cmd_t, cmd_a, mode]
    )

    assert t is not None
    n_samples = len(t)
    if n_samples < 2:
        raise ValueError("Bundle timeline too short to animate.")

    # Decide which track drives the "vehicle marker"
    drive_n = truth_n if (truth_n is not None and np.any(np.isfinite(truth_n))) else est_n
    drive_e = truth_e if (truth_e is not None and np.any(np.isfinite(truth_e))) else est_e
    drive_alt = truth_alt if (truth_alt is not None and np.any(np.isfinite(truth_alt))) else est_alt

    if drive_n is None or drive_e is None:
        raise ValueError("Need at least truth or estimate N/E to animate.")

    psi = heading_from_track(drive_n, drive_e)

    # Tail length in frames (based on median dt)
    dt = np.diff(t)
    dt = dt[np.isfinite(dt) & (dt > 0)]
    dt_med = float(np.median(dt)) if dt.size else 0.05
    tail_frames = max(1, int((tail_s / dt_med) / max(1, skip)))

    # Frame indices
    frame_idx = np.arange(0, n_samples, skip, dtype=int)

    # Figure layout (industry-ish “playback dashboard”)
    fig = plt.figure(figsize=(15, 9))
    axs = fig.subplot_mosaic(
        [
            ["map", "map", "alt", "alt"],
            ["map", "map", "cmd", "cmd"],
            ["map", "map", "mode", "mode"],
        ]
    )

    # ----- Map axis -----
    ax_map = axs["map"]

    # Background full tracks (faint)
    if show_truth and truth_n is not None and truth_e is not None:
        ax_map.plot(truth_n, truth_e, alpha=0.35, label="Truth")
    if show_est and est_n is not None and est_e is not None:
        ax_map.plot(est_n, est_e, alpha=0.35, label="Estimate")
    if show_gps and gps_n is not None and gps_e is not None:
        ax_map.plot(gps_n, gps_e, alpha=0.25, label="GPS")

    # Tail + marker + heading arrow
    tail_line, = ax_map.plot([], [], lw=2.0, label="Tail")
    marker, = ax_map.plot([], [], marker="o", markersize=8, linestyle="None", label="Vehicle")

    # Heading arrow as a small line segment
    heading_line, = ax_map.plot([], [], lw=2.0, label="Heading")

    ax_map.set_title("2D Ground Track (N/E)")
    ax_map.set_xlabel("North [m]")
    ax_map.set_ylabel("East [m]")
    ax_map.axis("equal")
    ax_map.grid(True)
    ax_map.legend(loc="upper right")

    # Compute map bounds
    xs = []
    ys = []
    if show_truth and truth_n is not None and truth_e is not None:
        xs.append(truth_n); ys.append(truth_e)
    if show_est and est_n is not None and est_e is not None:
        xs.append(est_n); ys.append(est_e)
    if show_gps and gps_n is not None and gps_e is not None:
        xs.append(gps_n); ys.append(gps_e)
    if not xs:
        xs = [drive_n]; ys = [drive_e]

    x_all = np.concatenate([x[np.isfinite(x)] for x in xs if np.any(np.isfinite(x))]) if xs else np.array([0.0])
    y_all = np.concatenate([y[np.isfinite(y)] for y in ys if np.any(np.isfinite(y))]) if ys else np.array([0.0])
    xmin, xmax = float(np.min(x_all)), float(np.max(x_all))
    ymin, ymax = float(np.min(y_all)), float(np.max(y_all))
    xlim = pad_limits(xmin, xmax)
    ylim = pad_limits(ymin, ymax)
    ax_map.set_xlim(*xlim)
    ax_map.set_ylim(*ylim)

    # Status text
    status = ax_map.text(
        0.02, 0.02, "",
        transform=ax_map.transAxes,
        va="bottom", ha="left",
        fontsize=10,
        bbox=dict(boxstyle="round", alpha=0.7)
    )

    # ----- Altitude axis -----
    ax_alt = axs["alt"]
    if show_truth and truth_alt is not None:
        ax_alt.plot(t, truth_alt, label="Truth alt", alpha=0.9)
    if show_est and est_alt is not None:
        ax_alt.plot(t, est_alt, label="Est alt", alpha=0.9)
    if show_gps and gps_alt is not None:
        ax_alt.plot(t, gps_alt, label="GPS alt", alpha=0.7)

    alt_cursor = ax_alt.axvline(t[0], lw=1.5)
    ax_alt.set_title("Altitude")
    ax_alt.set_xlabel("Time [s]")
    ax_alt.set_ylabel("Altitude [m]")
    ax_alt.grid(True)
    ax_alt.legend(loc="upper right")

    # ----- Commands axis -----
    ax_cmd = axs["cmd"]
    if cmd_e is not None:
        ax_cmd.plot(t, cmd_e, label="cmd_elevator", alpha=0.9)
    if cmd_t is not None:
        ax_cmd.plot(t, cmd_t, label="cmd_throttle", alpha=0.9)
    if cmd_a is not None:
        ax_cmd.plot(t, cmd_a, label="cmd_aileron", alpha=0.6)

    cmd_cursor = ax_cmd.axvline(t[0], lw=1.5)
    ax_cmd.set_title("FSW Commands")
    ax_cmd.set_xlabel("Time [s]")
    ax_cmd.set_ylabel("cmd / rad")
    ax_cmd.grid(True)
    ax_cmd.legend(loc="upper right")

    # ----- Mode axis -----
    ax_mode = axs["mode"]
    if mode is not None:
        ax_mode.step(t, mode, where="post", alpha=0.9)
    mode_cursor = ax_mode.axvline(t[0], lw=1.5)
    ax_mode.set_title("Mode")
    ax_mode.set_xlabel("Time [s]")
    ax_mode.set_ylabel("mode")
    ax_mode.grid(True)

    fig.tight_layout()

    # Heading arrow geometry (in meters)
    arrow_len = 10.0

    def update(frame_k: int):
        i = int(frame_idx[frame_k])
        ti = float(t[i])

        # Vehicle position
        ni = float(drive_n[i])
        ei = float(drive_e[i])
        alti = float(drive_alt[i]) if drive_alt is not None else float("nan")

        # Tail window
        i0 = max(0, frame_k - tail_frames)
        idx_tail = frame_idx[i0:frame_k + 1]
        tail_line.set_data(drive_n[idx_tail], drive_e[idx_tail])

        marker.set_data([ni], [ei])

        # Heading arrow from approximated heading
        psi_i = float(psi[i])
        hn = ni + arrow_len * np.cos(psi_i)
        he = ei + arrow_len * np.sin(psi_i)
        heading_line.set_data([ni, hn], [ei, he])

        # Cursors
        alt_cursor.set_xdata([ti, ti])
        cmd_cursor.set_xdata([ti, ti])
        mode_cursor.set_xdata([ti, ti])

        # Error vs GPS (if available)
        xy_err = float("nan")
        alt_err = float("nan")
        if gps_n is not None and gps_e is not None:
            if np.isfinite(gps_n[i]) and np.isfinite(gps_e[i]) and np.isfinite(ni) and np.isfinite(ei):
                xy_err = float(np.hypot(ni - gps_n[i], ei - gps_e[i]))
        if gps_alt is not None and np.isfinite(gps_alt[i]) and np.isfinite(alti):
            alt_err = float(alti - gps_alt[i])

        # Mode label
        mode_val = float(mode[i]) if mode is not None else float("nan")
        mode_name = mn.name(mode_val)

        status.set_text(
            f"t = {ti:6.2f} s\n"
            f"N/E = {ni:8.2f}, {ei:8.2f} m\n"
            f"Alt = {alti:7.2f} m\n"
            f"XY err (vs GPS) = {xy_err:6.2f} m\n"
            f"Alt err (vs GPS) = {alt_err:6.2f} m\n"
            f"Mode = {mode_name}"
        )

        # Follow camera
        if follow and np.isfinite(ni) and np.isfinite(ei):
            half_w = 120.0  # meters (tune)
            ax_map.set_xlim(ni - half_w, ni + half_w)
            ax_map.set_ylim(ei - half_w, ei + half_w)

        return (tail_line, marker, heading_line, alt_cursor, cmd_cursor, mode_cursor, status)

    interval_ms = int(1000.0 / max(1, fps) / max(1e-6, speed))

    anim = FuncAnimation(
        fig,
        update,
        frames=len(frame_idx),
        interval=interval_ms,
        blit=False,  # more robust across backends
        repeat=False,
    )

    return anim


# ---------------------------
# CLI
# ---------------------------

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle", required=True, help="Path to run_bundle.npz (e.g. logs/run_bundle.npz)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    ap.add_argument("--skip", type=int, default=1, help="Frame skip (1 = no skip)")
    ap.add_argument("--tail_s", type=float, default=4.0, help="Tail duration (seconds)")
    ap.add_argument("--follow", action="store_true", help="Camera follows vehicle")
    ap.add_argument("--save_mp4", default=None, help="Optional mp4 output path")
    ap.add_argument("--no_show", action="store_true")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    bundle_path = (repo_root / args.bundle).resolve()

    print(f"Using bundle: {bundle_path}")
    bundle = load_npz(bundle_path)

    anim = animate_run_2d(
        bundle=bundle,
        fps=args.fps,
        speed=args.speed,
        skip=max(1, args.skip),
        tail_s=args.tail_s,
        follow=args.follow,
    )

    if args.save_mp4:
        out = (repo_root / args.save_mp4).resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        try:
            from matplotlib.animation import FFMpegWriter
            writer = FFMpegWriter(fps=args.fps)
            anim.save(str(out), writer=writer)
            print(f"Saved mp4: {out}")
        except Exception as e:
            print(f"ERROR: failed to save mp4 (need ffmpeg?): {e}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()