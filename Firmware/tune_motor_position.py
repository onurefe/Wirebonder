#!/usr/bin/env python3
"""Estimate position-loop Kp from a motor-position open-loop ramp capture.

The script fits position = slope * time + intercept over a selected ramp
window. For a position controller that outputs velocity setpoint:

    K = slope / velocity_command
    Kp ~= 3 / (K * settling_time_95)

It also recommends a light first-order low-pass time constant for the
position measurement. The recommendation keeps the filter faster than the
position loop, so it reduces noise without adding too much lag.

Usage:
    python3 tune_motor_position.py
    python3 tune_motor_position.py captures/motor_position_20260706_212136.csv
    python3 tune_motor_position.py --start 0.25 --end 0.5 --command 1.0 --settling 1.0
"""

import argparse
import csv
import glob
import math
import os
import sys


def newest_capture(captures_dir):
    found = sorted(glob.glob(os.path.join(captures_dir, "motor_position_*.csv")),
                   key=os.path.getmtime)
    if not found:
        sys.exit("no motor_position CSVs found in " + captures_dir)
    return found[-1]


def read_position(path):
    samples = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            samples.append((float(row["time_s"]), float(row["position"])))
    if not samples:
        sys.exit("empty capture: " + path)
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


def linear_fit(samples):
    n = len(samples)
    mean_t = sum(t for t, _p in samples) / n
    mean_p = sum(p for _t, p in samples) / n

    variance_t = sum((t - mean_t) ** 2 for t, _p in samples)
    if variance_t == 0.0:
        sys.exit("selected window has no time span")

    slope = sum((t - mean_t) * (p - mean_p) for t, p in samples) / variance_t
    intercept = mean_p - slope * mean_t

    residual = sum((p - (intercept + slope * t)) ** 2 for t, p in samples)
    total = sum((p - mean_p) ** 2 for _t, p in samples)
    r_squared = 1.0 - residual / total if total > 0.0 else math.nan

    return slope, intercept, r_squared


def select_window(samples, start, end):
    selected = [(t, p) for t, p in samples if start <= t <= end]
    if len(selected) < 2:
        sys.exit("selected window needs at least two samples")
    return selected


def estimate_sample_period(samples):
    deltas = [
        samples[i][0] - samples[i - 1][0]
        for i in range(1, len(samples))
        if samples[i][0] > samples[i - 1][0]
    ]
    if not deltas:
        return 0.0

    deltas.sort()
    return deltas[len(deltas) // 2]


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file", nargs="?", help="motor_position CSV")
    parser.add_argument("--captures", default=os.path.join(here, "captures"),
                        help="captures directory")
    parser.add_argument("--start", type=float, help="ramp start time in seconds")
    parser.add_argument("--end", type=float, help="ramp end time in seconds")
    parser.add_argument("--command", type=float,
                        help="open-loop velocity command used for the capture")
    parser.add_argument("--settling", type=float,
                        help="desired 95%% settling time in seconds")
    args = parser.parse_args()

    path = args.file or newest_capture(args.captures)
    samples = read_position(path)

    print("capture: %s" % path)
    print("time range: %.6g s to %.6g s, %d samples" %
          (samples[0][0], samples[-1][0], len(samples)))

    start = args.start
    if start is None:
        start = ask_float("ramp start time (s)", 0.25)

    end = args.end
    if end is None:
        end = ask_float("ramp end time (s)", 0.50)

    if end <= start:
        sys.exit("end time must be greater than start time")

    command = args.command
    if command is None:
        command = ask_float("open-loop velocity command")

    if command == 0.0:
        sys.exit("open-loop velocity command must be nonzero")

    settling = args.settling
    if settling is None:
        settling = ask_float("desired 95% settling time (s)", 1.0)

    if settling <= 0.0:
        sys.exit("settling time must be positive")

    selected = select_window(samples, start, end)
    slope, intercept, r_squared = linear_fit(selected)

    t0, p0 = selected[0]
    t1, p1 = selected[-1]
    endpoint_slope = (p1 - p0) / (t1 - t0)

    plant_gain = slope / command
    tau_pos = settling / 3.0
    kp = 1.0 / (plant_gain * tau_pos)
    sample_period = estimate_sample_period(samples)
    filter_min = max(tau_pos / 20.0, 2.0 * sample_period)
    filter_max = tau_pos / 5.0
    filter_tc = max(tau_pos / 10.0, 2.0 * sample_period)

    if filter_min > filter_max:
        filter_min = filter_max
        filter_tc = filter_max

    print()
    print("window: %.6g s to %.6g s (%d samples)" % (t0, t1, len(selected)))
    print("position: %.6g -> %.6g, delta %.6g" % (p0, p1, p1 - p0))
    print("endpoint slope: %.6g position-unit/s" % endpoint_slope)
    print("fit slope:      %.6g position-unit/s" % slope)
    print("fit intercept:  %.6g position-unit" % intercept)
    print("fit R^2:        %.6g" % r_squared)
    print()
    print("plant gain K = slope / command = %.6g" % plant_gain)
    print("tau_pos = settling / 3 = %.6g s" % tau_pos)
    print("suggested P gain Kp ~= %.6g" % kp)
    if sample_period > 0.0:
        print("sample period ~= %.6g s" % sample_period)
    print("position filter TC range: %.6g s to %.6g s" %
          (filter_min, filter_max))
    print("suggested position filter TC ~= %.6g s" % filter_tc)
    print()
    print("Use this as a conservative P-only starting point. Increase settling")
    print("time for gentler/no-overshoot motion; decrease only after testing.")
    print("Use a longer filter TC only if position noise is driving noisy")
    print("velocity commands; too much filtering can add overshoot.")


if __name__ == "__main__":
    main()
