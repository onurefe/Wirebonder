#!/usr/bin/env python3
"""Tune force-coil PI gains from an open-loop current capture.

The input should be a `debug-force-coil ... open` CSV. The script estimates a
first-order coil model:

    I(s) / D(s) = Kc / (tau_c*s + 1)

where D is PWM duty and I is current in amps. With the firmware PI controller:

    D(s) / E(s) = Kp * (Ti*s + 1) / (Ti*s)

the closed-loop denominator is:

    Ti*tau_c*s^2 + Ti*(1 + Kc*Kp)*s + Kc*Kp

Matching this to a second-order target:

    s^2 + 2*zeta*omega_n*s + omega_n^2

gives:

    Kp = (2*zeta*omega_n*tau_c - 1) / Kc
    Ti = Kc*Kp / (omega_n^2*tau_c)

`zeta` sets damping. `speed-factor` sets closed-loop speed relative to the
measured coil time constant:

    omega_n = speed_factor / tau_c

Usage:
    python3 tune_force_coil.py
    python3 tune_force_coil.py captures/force_coil_current_20260708_200526.csv
    python3 tune_force_coil.py --start 0.002 --end 0.02 --duty-step 0.8
    python3 tune_force_coil.py --start 0.002 --end 0.02 --command 0.8
    python3 tune_force_coil.py --zeta 1.0 --speed-factor 3.0
"""

import argparse
import csv
import glob
import math
import os
import sys


def newest_capture(captures_dir):
    found = sorted(
        glob.glob(os.path.join(captures_dir, "force_coil_current_*.csv")),
        key=os.path.getmtime)
    if not found:
        sys.exit("no force_coil_current CSVs found in " + captures_dir)
    return found[-1]


def read_capture(path):
    samples = []
    command = None
    mode = None

    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            if command is None and "command" in row:
                command = float(row["command"])
            if mode is None and "mode" in row:
                mode = row["mode"]

            current_key = "current_a" if "current_a" in row else "current"
            samples.append((float(row["time_s"]), float(row[current_key])))

    if len(samples) < 4:
        sys.exit("capture needs at least four samples: " + path)

    return samples, command, mode


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


def select_window(samples, start, end):
    selected = [(t, current) for t, current in samples if start <= t <= end]
    if len(selected) < 4:
        sys.exit("selected window needs at least four samples")
    return selected


def mean(values):
    return sum(values) / len(values) if values else 0.0


def linear_fit(x, y):
    count = len(x)
    mean_x = sum(x) / count
    mean_y = sum(y) / count
    variance_x = sum((value - mean_x) ** 2 for value in x)

    if variance_x == 0.0:
        sys.exit("selected window has no time span")

    slope = sum((xi - mean_x) * (yi - mean_y)
                for xi, yi in zip(x, y)) / variance_x
    intercept = mean_y - slope * mean_x

    residual = sum((yi - (intercept + slope * xi)) ** 2
                   for xi, yi in zip(x, y))
    total = sum((yi - mean_y) ** 2 for yi in y)
    r_squared = 1.0 - residual / total if total > 0.0 else math.nan

    return slope, intercept, r_squared


def crossing_time(samples, target_value, rising):
    for index in range(1, len(samples)):
        t0, y0 = samples[index - 1]
        t1, y1 = samples[index]

        if rising:
            crossed = y0 <= target_value <= y1
        else:
            crossed = y0 >= target_value >= y1

        if not crossed or y1 == y0:
            continue

        fraction = (target_value - y0) / (y1 - y0)
        return t0 + fraction * (t1 - t0)

    return None


def fit_time_constant(samples, initial_current, steady_current,
                      min_progress, max_progress):
    delta = steady_current - initial_current
    if abs(delta) < 1e-9:
        sys.exit("current step is too small to tune from")

    fit_samples = []
    t0 = samples[0][0]

    for t, current in samples:
        progress = (current - initial_current) / delta
        if progress <= min_progress or progress >= max_progress:
            continue

        remaining = 1.0 - progress
        if remaining <= 0.0:
            continue

        fit_samples.append((t - t0, math.log(remaining)))

    if len(fit_samples) < 3:
        return None, None, None

    x = [t for t, _log_remaining in fit_samples]
    y = [log_remaining for _t, log_remaining in fit_samples]
    slope, intercept, r_squared = linear_fit(x, y)

    if slope >= 0.0:
        return None, intercept, r_squared

    return -1.0 / slope, intercept, r_squared


def closed_loop_poles(coil_tau, kp, ti, coil_gain):
    a = ti * coil_tau
    b = ti * (1.0 + coil_gain * kp)
    c = coil_gain * kp
    discriminant = b * b - 4.0 * a * c

    if discriminant >= 0.0:
        root = math.sqrt(discriminant)
        return [(-b + root) / (2.0 * a), (-b - root) / (2.0 * a)]

    real = -b / (2.0 * a)
    imag = math.sqrt(-discriminant) / (2.0 * a)
    return [complex(real, imag), complex(real, -imag)]


def overshoot_percent(zeta):
    if zeta >= 1.0:
        return 0.0
    return 100.0 * math.exp(
        -math.pi * zeta / math.sqrt(1.0 - zeta * zeta))


def format_poles(poles):
    formatted = []
    for pole in poles:
        if isinstance(pole, complex):
            formatted.append("%.8g%+.8gj" % (pole.real, pole.imag))
        else:
            formatted.append("%.8g" % pole)
    return ", ".join(formatted)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file", nargs="?", help="force_coil_current CSV")
    parser.add_argument("--captures", default=os.path.join(here, "captures"),
                        help="captures directory")
    parser.add_argument("--start", type=float,
                        help="fit window start time in seconds")
    parser.add_argument("--end", type=float,
                        help="fit window end time in seconds")
    parser.add_argument("--duty-step", type=float,
                        help="open-loop duty step applied in the capture")
    parser.add_argument("--command", type=float,
                        help="absolute open-loop duty command used in the capture")
    parser.add_argument("--initial-duty", type=float, default=0.0,
                        help="duty before the open-loop step")
    parser.add_argument("--zeta", type=float, default=1.0,
                        help="target damping ratio")
    parser.add_argument("--speed-factor", type=float, default=2.0,
                        help="omega_n = speed_factor / coil_tau")
    parser.add_argument("--settle-tail", type=float, default=0.15,
                        help="fraction of selected window used as steady tail")
    parser.add_argument("--min-progress", type=float, default=0.1,
                        help="ignore earlier response below this fraction")
    parser.add_argument("--max-progress", type=float, default=0.9,
                        help="ignore late response above this fraction")
    parser.add_argument("--min-r2", type=float, default=0.8,
                        help="minimum fit R^2 needed to trust fitted tau")
    args = parser.parse_args()

    path = args.file or newest_capture(args.captures)
    samples, captured_command, mode = read_capture(path)

    print("capture: %s" % path)
    print("time range: %.6g s to %.6g s, %d samples" %
          (samples[0][0], samples[-1][0], len(samples)))
    if mode is not None:
        print("mode: %s" % mode)

    if mode is not None and mode != "open":
        print("warning: this does not look like an open-loop capture")

    start = args.start
    if start is None:
        start = ask_float("fit start time (s)", samples[0][0])

    end = args.end
    if end is None:
        end = ask_float("fit end time (s)", samples[-1][0])

    if end <= start:
        sys.exit("end time must be greater than start time")

    command = args.command if args.command is not None else captured_command
    if args.duty_step is not None:
        duty_step = args.duty_step
        if command is None:
            command = args.initial_duty + duty_step
        elif abs((command - args.initial_duty) - duty_step) > 1e-6:
            print("warning: recorded/absolute command implies duty step %.8g, "
                  "but --duty-step is %.8g" %
                  (command - args.initial_duty, duty_step))
    else:
        if command is None:
            command = ask_float("open-loop duty command")
        duty_step = command - args.initial_duty
    if abs(duty_step) < 1e-9:
        sys.exit("open-loop duty step must be nonzero")

    if args.zeta <= 0.0:
        sys.exit("zeta must be positive")

    if args.speed_factor <= 0.0:
        sys.exit("speed factor must be positive")

    if args.settle_tail <= 0.0 or args.settle_tail > 1.0:
        sys.exit("settle-tail must be in the range (0, 1]")

    selected = select_window(samples, start, end)
    tail_count = max(1, int(len(selected) * args.settle_tail))

    initial_current = selected[0][1]
    steady_current = mean([current for _t, current in selected[-tail_count:]])
    current_delta = steady_current - initial_current
    coil_gain = current_delta / duty_step

    if abs(coil_gain) < 1e-9:
        sys.exit("coil gain is too small to tune from")

    if coil_gain <= 0.0:
        sys.exit("coil gain is negative; fix current-sense/PWM sign before tuning")

    tau_fit, fit_intercept, r_squared = fit_time_constant(
        selected,
        initial_current,
        steady_current,
        args.min_progress,
        args.max_progress)

    target_632 = initial_current + 0.6321205588 * current_delta
    t632 = crossing_time(selected, target_632, current_delta > 0.0)
    tau_crossing = None if t632 is None else t632 - selected[0][0]
    fit_is_trusted = (
        tau_fit is not None and
        r_squared is not None and
        not math.isnan(r_squared) and
        r_squared >= args.min_r2
    )
    coil_tau = tau_fit if fit_is_trusted else tau_crossing

    if coil_tau is None or coil_tau <= 0.0:
        sys.exit("could not estimate coil time constant from this window")

    omega_n = args.speed_factor / coil_tau
    kp = (2.0 * args.zeta * omega_n * coil_tau - 1.0) / coil_gain
    if kp <= 0.0:
        min_speed = 1.0 / (2.0 * args.zeta)
        sys.exit("selected speed factor gives non-positive Kp; "
                 "use --speed-factor greater than %.8g" % min_speed)

    ti = coil_gain * kp / (omega_n * omega_n * coil_tau)
    if ti <= 0.0:
        sys.exit("computed integral time constant is non-positive")

    ki = kp / ti
    expected_settling = (
        4.0 / (args.zeta * omega_n) if args.zeta < 1.0
        else 4.0 / omega_n)
    poles = closed_loop_poles(coil_tau, kp, ti, coil_gain)

    print()
    print("window: %.6g s to %.6g s (%d samples)" %
          (selected[0][0], selected[-1][0], len(selected)))
    print("initial duty:              %.8g" % args.initial_duty)
    print("command duty:              %.8g" % command)
    print("duty step:                 %.8g" % duty_step)
    print("initial current:           %.8g A" % initial_current)
    print("steady current:            %.8g A" % steady_current)
    print("current delta:             %.8g A" % current_delta)
    print("coil gain Kc:              %.8g A/duty" % coil_gain)
    print()
    if tau_fit is not None:
        print("fit tau_c:                 %.8g s" % tau_fit)
        print("fit intercept:             %.8g" % fit_intercept)
        print("fit R^2:                   %.8g" % r_squared)
    if tau_crossing is not None:
        print("63.2%% crossing tau_c:      %.8g s" % tau_crossing)
    if tau_fit is not None and not fit_is_trusted:
        print("fit below R^2 %.3g; using crossing tau" % args.min_r2)
    print("selected coil tau_c:       %.8g s" % coil_tau)
    print()
    print("target zeta:               %.8g" % args.zeta)
    print("speed factor:              %.8g" % args.speed_factor)
    print("target omega_n:            %.8g rad/s" % omega_n)
    print("model overshoot:           %.8g %%" %
          overshoot_percent(args.zeta))
    print("approx 2%% settling:        %.8g s" % expected_settling)
    print("closed-loop poles:         %s" % format_poles(poles))
    print()
    print("PI proportional Kp:        %.8g duty/A" % kp)
    print("PI integral Ki:            %.8g duty/(A*s)" % ki)
    print("PI integral Tc Ti:         %.8g s" % ti)
    print()
    print("Set:")
    print("#define FORCE_COIL_MODULE_PID_GAIN %.8gf" % kp)
    print("#define FORCE_COIL_MODULE_PID_INTEGRAL_TC %.8gf" % ti)
    print("#define FORCE_COIL_MODULE_PID_DERIVATIVE_TC 0.0f")


if __name__ == "__main__":
    main()
