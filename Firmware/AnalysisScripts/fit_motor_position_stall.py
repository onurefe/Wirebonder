#!/usr/bin/env python3
"""Fit a line to a motor-position-stall capture interval.

By default this uses the newest captures/motor_position_stall_*.csv file and
fits:

    position = slope * drive + intercept

Usage:
    python3 fit_motor_position_stall.py
    python3 fit_motor_position_stall.py --start 0.05 --end 0.20
    python3 fit_motor_position_stall.py --x duty --start 0.55 --end 0.70
    python3 fit_motor_position_stall.py captures/motor_position_stall_*.csv
"""

import argparse
import csv
import glob
import math
import os
import sys


def newest_capture(captures_dir):
    found = sorted(
        glob.glob(os.path.join(captures_dir, "motor_position_stall_*.csv")),
        key=os.path.getmtime)
    if not found:
        sys.exit("no motor_position_stall CSVs found in " + captures_dir)
    return found[-1]


def read_capture(path, x_column):
    samples = []

    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            drive = float(row["drive"])
            if x_column == "duty":
                x = float(row["duty"]) if "duty" in row else 0.5 + drive
            else:
                x = drive

            samples.append((x, float(row["position"])))

    if len(samples) < 2:
        sys.exit("capture needs at least two samples: " + path)

    return samples


def ask_float(label, default=None):
    suffix = "" if default is None else " [%g]" % default
    while True:
        text = input("%s%s: " % (label, suffix)).strip()
        if not text and default is not None:
            return default
        try:
            return float(text)
        except ValueError:
            print("please enter a number")


def select_interval(samples, start, end):
    low = min(start, end)
    high = max(start, end)
    selected = [(x, y) for x, y in samples if low <= x <= high]

    if len(selected) < 2:
        sys.exit("selected interval needs at least two samples")

    return selected


def linear_fit(samples):
    n = len(samples)
    xs = [x for x, _y in samples]
    ys = [y for _x, y in samples]

    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    variance_x = sum((x - mean_x) ** 2 for x in xs)

    if variance_x == 0.0:
        sys.exit("selected interval has no x span")

    slope = sum((x - mean_x) * (y - mean_y)
                for x, y in samples) / variance_x
    intercept = mean_y - slope * mean_x

    residual = sum((y - (slope * x + intercept)) ** 2
                   for x, y in samples)
    total = sum((y - mean_y) ** 2 for y in ys)
    r_squared = 1.0 - residual / total if total > 0.0 else math.nan
    rmse = math.sqrt(residual / n)

    return slope, intercept, r_squared, rmse


def main():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file", nargs="?", help="motor_position_stall CSV")
    parser.add_argument("--captures", default=os.path.join(repo_root, "captures"),
                        help="captures directory")
    parser.add_argument("--x", choices=("drive", "duty"), default="drive",
                        help="x-axis quantity used for the fit")
    parser.add_argument("--start", type=float,
                        help="fit interval start in x units")
    parser.add_argument("--end", type=float,
                        help="fit interval end in x units")
    args = parser.parse_args()

    path = args.file or newest_capture(args.captures)
    samples = read_capture(path, args.x)

    min_x = min(x for x, _y in samples)
    max_x = max(x for x, _y in samples)
    start = args.start
    end = args.end

    if start is None:
        start = ask_float("fit interval start %s" % args.x, min_x)
    if end is None:
        end = ask_float("fit interval end %s" % args.x, max_x)

    selected = select_interval(samples, start, end)
    slope, intercept, r_squared, rmse = linear_fit(selected)

    print("capture: %s" % path)
    print("x column: %s" % args.x)
    print("interval: %.9g to %.9g (%d samples)" %
          (min(start, end), max(start, end), len(selected)))
    print("")
    print("position = slope * %s + intercept" % args.x)
    print("slope:      %.9g" % slope)
    print("intercept:  %.9g" % intercept)
    print("equation:   position = %.9g * %s %+.9g" %
          (slope, args.x, intercept))
    print("R^2:        %.9g" % r_squared)
    print("RMSE:       %.9g" % rmse)


if __name__ == "__main__":
    main()
