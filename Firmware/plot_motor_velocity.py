#!/usr/bin/env python3
"""Plot debug-motor-velocity capture CSVs (columns: tick,time_s,velocity).

Usage:
    python3 plot_motor_velocity.py                # newest capture
    python3 plot_motor_velocity.py --all          # overlay all captures
    python3 plot_motor_velocity.py file1.csv ...  # specific files
"""

import argparse
import csv
import glob
import os
import sys

import matplotlib.pyplot as plt


def read(path):
    t, v = [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["time_s"]))
            v.append(float(row["velocity"]))
    return t, v


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="*", help="CSV files (default: newest capture)")
    ap.add_argument("--captures", default=os.path.join(here, "captures"),
                    help="captures directory")
    ap.add_argument("--all", action="store_true", help="overlay all captures")
    args = ap.parse_args()

    files = args.files
    if not files:
        found = sorted(glob.glob(os.path.join(args.captures, "motor_velocity_*.csv")),
                       key=os.path.getmtime)
        if not found:
            sys.exit("no motor_velocity CSVs found in " + args.captures)
        files = found if args.all else [found[-1]]

    plt.figure(figsize=(10, 5))
    for path in files:
        t, v = read(path)
        plt.plot(t, v, label=os.path.basename(path))

    plt.axhline(0, color="gray", linewidth=0.8)
    plt.xlabel("time (s)")
    plt.ylabel("velocity")
    plt.title("Motor velocity")
    plt.grid(True, alpha=0.3)
    if len(files) > 1:
        plt.legend(fontsize=8)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
