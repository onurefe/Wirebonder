#!/usr/bin/env python3
"""Plot debug-motor-position capture CSVs.

New captures include columns: tick,time_s,position,magA,magB.
Older position-only captures are still supported.

Usage:
    python3 plot_motor_position.py                # newest capture
    python3 plot_motor_position.py --all          # overlay all captures
    python3 plot_motor_position.py file1.csv ...  # specific files
"""

import argparse
import csv
import glob
import os
import sys

import matplotlib.pyplot as plt


def read(path):
    t, p, mag_a, mag_b = [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["time_s"]))
            p.append(float(row["position"]))
            if "magA" in row and "magB" in row:
                mag_a.append(float(row["magA"]))
                mag_b.append(float(row["magB"]))
    return t, p, mag_a, mag_b


def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", help="CSV files (default: newest capture)")
    ap.add_argument("--captures", default=os.path.join(repo_root, "captures"),
                    help="captures directory")
    ap.add_argument("--all", action="store_true", help="overlay all captures")
    args = ap.parse_args()

    files = args.files
    if not files:
        found = sorted(glob.glob(os.path.join(args.captures, "motor_position_*.csv")),
                       key=os.path.getmtime)
        if not found:
            sys.exit("no motor_position CSVs found in " + args.captures)
        files = found if args.all else [found[-1]]

    captures = [(path, *read(path)) for path in files]
    has_magnitudes = any(mag_a and mag_b
                         for _path, _t, _p, mag_a, mag_b in captures)

    if has_magnitudes:
        fig, (position_ax, magnitude_ax) = plt.subplots(
            2, 1, figsize=(10, 7), sharex=True)
    else:
        fig, position_ax = plt.subplots(figsize=(10, 5))
        magnitude_ax = None

    for path, t, p, mag_a, mag_b in captures:
        label = os.path.basename(path)
        position_ax.plot(t, p, label=label)

        if magnitude_ax is not None and mag_a and mag_b:
            magnitude_ax.plot(t, mag_a, label=label + " magA")
            magnitude_ax.plot(t, mag_b, label=label + " magB", linestyle="--")

    position_ax.axhline(0, color="gray", linewidth=0.8)
    position_ax.set_ylabel("position")
    position_ax.set_title("Motor position")
    position_ax.grid(True, alpha=0.3)

    if magnitude_ax is None:
        position_ax.set_xlabel("time (s)")
    else:
        magnitude_ax.set_xlabel("time (s)")
        magnitude_ax.set_ylabel("magnitude")
        magnitude_ax.set_title("LVDT secondary magnitudes")
        magnitude_ax.grid(True, alpha=0.3)

    if len(files) > 1:
        position_ax.legend(fontsize=8)
        if magnitude_ax is not None:
            magnitude_ax.legend(fontsize=8)
    elif magnitude_ax is not None:
        magnitude_ax.legend(fontsize=8)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
