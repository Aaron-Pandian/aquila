"""
Plot flight mode vs time from fsw_output.csv.

Usage:
    python -m tools.plot_modes
"""

from pathlib import Path
import csv
import matplotlib.pyplot as plt

def load_modes(path: Path):
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        t = []
        mode = []
        for row in reader:
            t.append(float(row["t"]))
            mode.append(int(row["mode"]))
    return t, mode

def main():
    repo_root = Path(__file__).resolve().parents[1]
    fsw_log = repo_root / "logs" / "fsw_output.csv"
    print(f"Using FSW log: {fsw_log}")

    t, mode = load_modes(fsw_log)

    plt.figure(figsize=(8, 3))
    plt.step(t, mode, where="post")
    plt.yticks(
        [0, 1, 2, 3, 4],
        ["STBY", "CLIMB", "CRUISE", "RTL", "FAILSAFE"]
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Mode")
    plt.title("Flight Mode vs Time")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()