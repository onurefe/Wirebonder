#!/usr/bin/env python3
"""Plot debug-force-coil capture CSVs.

New captures include columns: tick,time_s,command,mode,current_a.
Older current-only captures are still supported.

Usage:
    python3 plot_force_coil.py                # newest capture
    python3 plot_force_coil.py --all          # overlay all captures
    python3 plot_force_coil.py file1.csv ...  # specific files
"""

import argparse
import csv
import glob
import os
import sys

import matplotlib.pyplot as plt


def read(path):
    t, current, command, mode = [], [], [], None
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(float(row["time_s"]))
            current.append(read_float(row, "current_a", "current"))

            if "command" in row:
                command.append(float(row["command"]))
            if mode is None and "mode" in row:
                mode = row["mode"]

    return t, current, command, mode


def read_float(row, preferred, legacy):
    if preferred in row:
        return float(row[preferred])
    return float(row[legacy])


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
        found = sorted(
            glob.glob(os.path.join(args.captures, "force_coil_current_*.csv")),
            key=os.path.getmtime)
        if not found:
            sys.exit("no force_coil_current CSVs found in " + args.captures)
        files = found if args.all else [found[-1]]

    plt.figure(figsize=(10, 5))

    captures = [(path, *read(path)) for path in files]
    has_command = any(command for _path, _t, _current, command, _mode in captures)

    for path, t, current, command, mode in captures:
        label = os.path.basename(path)
        if mode:
            label += " " + mode

        plt.plot(t, current, label=label)

        if command:
            plt.plot(t, command, label=label + " command", linestyle=":")

    plt.axhline(0, color="gray", linewidth=0.8)
    plt.xlabel("time (s)")
    plt.ylabel("current (A)")
    plt.title("Force coil current")
    plt.grid(True, alpha=0.3)
    if len(files) > 1 or has_command:
        plt.legend(fontsize=8)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
