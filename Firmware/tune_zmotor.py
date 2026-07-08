#!/usr/bin/env python3
"""Tune the Z-motor velocity preamplifier gain from an open-loop capture.

The input should be a `debug-motor-velocity ... open` CSV. It estimates the
open-loop motor response:

    velocity(t) ~= v0 + dv * (1 - exp(-(t - t0) / tau_m))

From that it computes:

    Km    = steady_velocity_delta / duty_offset
    tau_m = fitted motor time constant

The firmware velocity controller is modeled as a fixed leaky integrator with
a tunable preamplifier:

    Vc / error = Kpre * (Rf / Ri) / (Rf*Cf*s + 1)

The velocity module then maps controller voltage to duty:

    duty_offset = Vc * voltage_to_duty_scale

With the first-order motor model:

    M(s) = Km / (tau_m*s + 1)

the closed-loop velocity response is:

    V/Vref = L / (tau_c*tau_m*s^2 + (tau_c + tau_m)*s + 1 + L)

where:

    tau_c = Rf*Cf
    L     = Kpre * (Rf/Ri) * voltage_to_duty_scale * Km

This script keeps Ri/Rf/Cf fixed and chooses Kpre as large as possible while
respecting the requested damping target and safety margin.

Usage:
    python3 tune_zmotor.py
    python3 tune_zmotor.py captures/motor_velocity_20260707_195623.csv
    python3 tune_zmotor.py --start 0.02 --end 0.30
    python3 tune_zmotor.py --target-damping 0.8 --safety-margin 0.1
"""

import argparse
import csv
import glob
import math
import os
import sys


DEFAULT_RI = 1e3
DEFAULT_RF = 121e3
DEFAULT_CF = 1e-7
DEFAULT_VOLTAGE_TO_DUTY_SCALE = 1.0 / 30.0


def newest_capture(captures_dir):
    found = sorted(glob.glob(os.path.join(captures_dir, "motor_velocity_*.csv")),
                   key=os.path.getmtime)
    if not found:
        sys.exit("no motor_velocity CSVs found in " + captures_dir)
    return found[-1]


def read_capture(path):
    samples = []
    command = None
    mode = None

    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            if command is None and "command" in row:
                command = float(row["command"])
            if mode is None and "mode" in row:
                mode = row["mode"]
            samples.append((float(row["time_s"]), float(row["velocity"])))

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
    selected = [(t, v) for t, v in samples if start <= t <= end]
    if len(selected) < 4:
        sys.exit("selected window needs at least four samples")
    return selected


def mean(values):
    return sum(values) / len(values) if values else 0.0


def linear_fit(x, y):
    n = len(x)
    mean_x = sum(x) / n
    mean_y = sum(y) / n
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
    for i in range(1, len(samples)):
        t0, v0 = samples[i - 1]
        t1, v1 = samples[i]

        if rising:
            crossed = v0 <= target_value <= v1
        else:
            crossed = v0 >= target_value >= v1

        if not crossed or v1 == v0:
            continue

        fraction = (target_value - v0) / (v1 - v0)
        return t0 + fraction * (t1 - t0)

    return None


def fit_time_constant(samples, initial_velocity, steady_velocity, min_progress,
                      max_progress):
    delta = steady_velocity - initial_velocity
    if abs(delta) < 1e-9:
        sys.exit("velocity step is too small to tune from")

    fit_samples = []
    t0 = samples[0][0]

    for t, velocity in samples:
        progress = (velocity - initial_velocity) / delta
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


def damping_for(controller_tau, motor_tau, loop_gain):
    return ((controller_tau + motor_tau) /
            (2.0 * math.sqrt(controller_tau * motor_tau *
                              (1.0 + loop_gain))))


def loop_gain_for_damping(controller_tau, motor_tau, damping):
    if damping <= 0.0:
        sys.exit("target damping must be positive")

    numerator = (controller_tau + motor_tau) ** 2
    denominator = 4.0 * damping * damping * controller_tau * motor_tau
    return numerator / denominator - 1.0


def closed_loop_poles(controller_tau, motor_tau, loop_gain):
    a = controller_tau * motor_tau
    b = controller_tau + motor_tau
    c = 1.0 + loop_gain
    discriminant = b * b - 4.0 * a * c

    if discriminant >= 0.0:
        root = math.sqrt(discriminant)
        return [(-b + root) / (2.0 * a), (-b - root) / (2.0 * a)]

    real = -b / (2.0 * a)
    imag = math.sqrt(-discriminant) / (2.0 * a)
    return [complex(real, imag), complex(real, -imag)]


def settling_time_2pct(controller_tau, motor_tau, loop_gain, damping):
    omega_n = math.sqrt((1.0 + loop_gain) / (motor_tau * controller_tau))
    if damping < 1.0:
        return 4.0 / (damping * omega_n), omega_n

    poles = closed_loop_poles(controller_tau, motor_tau, loop_gain)
    slow_pole = min(abs(pole.real if isinstance(pole, complex) else pole)
                    for pole in poles)
    return 4.0 / slow_pole, omega_n


def overshoot_percent(damping):
    if damping >= 1.0:
        return 0.0
    return 100.0 * math.exp(
        -math.pi * damping / math.sqrt(1.0 - damping * damping))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file", nargs="?", help="motor_velocity CSV")
    parser.add_argument("--captures", default=os.path.join(here, "captures"),
                        help="captures directory")
    parser.add_argument("--start", type=float,
                        help="fit window start time in seconds")
    parser.add_argument("--end", type=float,
                        help="fit window end time in seconds")
    parser.add_argument("--command", type=float,
                        help="open-loop duty offset used in the capture")
    parser.add_argument("--target-damping", type=float, default=1.0,
                        help="target damping ratio; 1 is critical damping")
    parser.add_argument("--safety-margin", type=float, default=0.25,
                        help="fraction backed off from the target damping gain")
    parser.add_argument("--ri", type=float, default=DEFAULT_RI,
                        help="fixed leaky-integrator input resistance")
    parser.add_argument("--rf", type=float, default=DEFAULT_RF,
                        help="fixed leaky-integrator feedback resistance")
    parser.add_argument("--cf", type=float, default=DEFAULT_CF,
                        help="fixed leaky-integrator feedback capacitance")
    parser.add_argument("--voltage-to-duty-scale", type=float,
                        default=DEFAULT_VOLTAGE_TO_DUTY_SCALE,
                        help="bridge duty offset per controller volt")
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
    if command is None:
        command = ask_float("open-loop duty offset command")

    if abs(command) < 1e-9:
        sys.exit("open-loop command must be nonzero")

    if args.safety_margin < 0.0 or args.safety_margin >= 1.0:
        sys.exit("safety margin must be in the range [0, 1)")

    if args.ri <= 0.0 or args.rf <= 0.0 or args.cf <= 0.0:
        sys.exit("ri/rf/cf must be positive")

    if args.voltage_to_duty_scale <= 0.0:
        sys.exit("voltage-to-duty scale must be positive")

    selected = select_window(samples, start, end)
    tail_count = max(1, int(len(selected) * args.settle_tail))

    initial_velocity = selected[0][1]
    steady_velocity = mean([v for _t, v in selected[-tail_count:]])
    delta_velocity = steady_velocity - initial_velocity
    motor_gain = delta_velocity / command

    if abs(motor_gain) < 1e-9:
        sys.exit("motor gain is too small to tune from")

    if motor_gain <= 0.0:
        sys.exit("motor gain is negative; fix tachometer/motor sign before tuning")

    tau_fit, fit_intercept, r_squared = fit_time_constant(
        selected,
        initial_velocity,
        steady_velocity,
        args.min_progress,
        args.max_progress)

    target_632 = initial_velocity + 0.6321205588 * delta_velocity
    t632 = crossing_time(selected, target_632, delta_velocity > 0.0)
    tau_crossing = None if t632 is None else t632 - selected[0][0]
    fit_is_trusted = (
        tau_fit is not None and
        r_squared is not None and
        not math.isnan(r_squared) and
        r_squared >= args.min_r2
    )
    motor_tau = tau_fit if fit_is_trusted else tau_crossing

    if motor_tau is None or motor_tau <= 0.0:
        sys.exit("could not estimate motor time constant from this window")

    controller_tau = args.rf * args.cf
    loop_gain_at_target = loop_gain_for_damping(
        controller_tau,
        motor_tau,
        args.target_damping)

    if loop_gain_at_target <= 0.0:
        sys.exit("fixed Rf*Cf cannot reach this damping target with positive "
                 "preamplifier gain; lower --target-damping or change Rf/Cf")

    selected_loop_gain = (1.0 - args.safety_margin) * loop_gain_at_target
    analog_dc_gain_without_preamp = args.rf / args.ri
    duty_gain_per_preamp = (
        analog_dc_gain_without_preamp * args.voltage_to_duty_scale)
    preamplifier_gain = selected_loop_gain / (motor_gain * duty_gain_per_preamp)
    controller_duty_gain = preamplifier_gain * duty_gain_per_preamp
    controller_voltage_gain = preamplifier_gain * analog_dc_gain_without_preamp
    actual_damping = damping_for(controller_tau, motor_tau, selected_loop_gain)
    steady_fraction = selected_loop_gain / (1.0 + selected_loop_gain)
    steady_error_fraction = 1.0 / (1.0 + selected_loop_gain)
    settling_2pct, omega_n = settling_time_2pct(
        controller_tau,
        motor_tau,
        selected_loop_gain,
        actual_damping)
    poles = closed_loop_poles(controller_tau, motor_tau, selected_loop_gain)

    print()
    print("window: %.6g s to %.6g s (%d samples)" %
          (selected[0][0], selected[-1][0], len(selected)))
    print("command duty offset:        %.8g" % command)
    print("initial velocity:           %.8g mm/s" % initial_velocity)
    print("steady velocity:            %.8g mm/s" % steady_velocity)
    print("velocity delta:             %.8g mm/s" % delta_velocity)
    print("motor gain Km:              %.8g (mm/s)/duty" % motor_gain)
    print()
    if tau_fit is not None:
        print("fit tau_m:                  %.8g s" % tau_fit)
        print("fit intercept:              %.8g" % fit_intercept)
        print("fit R^2:                    %.8g" % r_squared)
    if tau_crossing is not None:
        print("63.2%% crossing tau_m:       %.8g s" % tau_crossing)
    if tau_fit is not None and not fit_is_trusted:
        print("fit below R^2 %.3g; using crossing tau" % args.min_r2)
    print("selected motor tau_m:       %.8g s" % motor_tau)
    print()
    print("fixed Ri:                   %.8g ohm" % args.ri)
    print("fixed Rf:                   %.8g ohm" % args.rf)
    print("fixed Cf:                   %.8g F" % args.cf)
    print("fixed controller tau_c:     %.8g s" % controller_tau)
    print("voltage-to-duty scale:      %.8g duty/V" %
          args.voltage_to_duty_scale)
    print("Rf/Ri gain:                 %.8g" % analog_dc_gain_without_preamp)
    print()
    print("target damping:             %.8g" % args.target_damping)
    print("safety margin:              %.8g" % args.safety_margin)
    print("loop gain at target damp:   %.8g" % loop_gain_at_target)
    print("selected loop gain L:       %.8g" % selected_loop_gain)
    print("actual damping:             %.8g" % actual_damping)
    print("model overshoot:            %.8g %%" %
          overshoot_percent(actual_damping))
    print("model natural freq:         %.8g rad/s" % omega_n)
    print("model 2%% settling:          %.8g s" % settling_2pct)
    print("steady command fraction:    %.8g" % steady_fraction)
    print("steady error fraction:      %.8g" % steady_error_fraction)
    print("closed-loop poles:          %s" % format_poles(poles))
    print()
    print("controller voltage gain:    %.8g V/(mm/s)" %
          controller_voltage_gain)
    print("controller duty gain:       %.8g duty/(mm/s)" %
          controller_duty_gain)
    print("suggested preamp gain:      %.8g V/(mm/s)" %
          preamplifier_gain)
    print()
    print("Set:")
    print("#define DCMOTOR_VELOCITY_MODULE_CONTROLLER_PREAMPLIFIER_GAIN %.8gf" %
          preamplifier_gain)
    print()
    print("Keep:")
    print("#define DCMOTOR_VELOCITY_MODULE_LEAKY_INTEGRATOR_RI %.8g" %
          args.ri)
    print("#define DCMOTOR_VELOCITY_MODULE_LEAKY_INTEGRATOR_RF %.8g" %
          args.rf)
    print("#define DCMOTOR_VELOCITY_MODULE_LEAKY_INTEGRATOR_CF %.8g" %
          args.cf)


def format_poles(poles):
    formatted = []
    for pole in poles:
        if isinstance(pole, complex):
            formatted.append("%.8g%+.8gj" % (pole.real, pole.imag))
        else:
            formatted.append("%.8g" % pole)
    return ", ".join(formatted)


if __name__ == "__main__":
    main()
