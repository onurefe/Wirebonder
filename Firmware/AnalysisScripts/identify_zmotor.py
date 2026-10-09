#!/usr/bin/env python3
"""Identify the Z motor for the LVDT-only servo and tune its velocity loop.

Works from `debug-motor-velocity` captures (FIRMWARE_MODE_DEBUG_MOTOR_VELOCITY),
which record every control tick: the LVDT position, the Kalman velocity
estimate, and the voltage applied for the following tick. Each step prints the
configuration.h lines to paste.

1. noise -- LVDT noise with the head at rest

     (gdb) debug-motor-velocity 0 4 open
     python3 identify_zmotor.py noise

   If the head creeps with no drive, push it gently onto the bottom stop
   instead (negative is down), e.g. debug-motor-velocity -0.02 4 open.

2. fit -- motor gain, time constant and load from open-loop steps

     (gdb) debug-motor-velocity 0.05 0.5 open
     (gdb) debug-motor-velocity -0.05 0.5 open
     (gdb) debug-motor-velocity 0.1 0.3 open
     (gdb) debug-motor-velocity -0.1 0.3 open
     python3 identify_zmotor.py fit captures/motor_velocity_*.csv

   Alternate up and down so the head stays inside its 9 mm of travel; a run
   that reaches a stop is trimmed automatically. At least two drive levels are
   needed to separate the motor gain from the gravity/friction load.

3. pid -- velocity PI gains for the identified motor and Kalman filter

     python3 identify_zmotor.py pid --gain 1.8 --tau 0.025 --noise 0.0015

   Simulates the firmware loop (Kalman filter, PID, duty clamp, input delay)
   and picks the fastest gains that stay within overshoot and drive-noise
   limits. Check the result on the machine with a closed-loop step, e.g.
   debug-motor-velocity 3 1.

4. coil -- what the force coil does to the Z drive (FIRMWARE_MODE_DEBUG_
   MOTOR_POSITION). Measured while moving, not holding: the geared iron-core
   motor holds the head against a wide band of drive through static friction,
   so a holding voltage says little. Make the same climb and descent with the
   coil off and on:

     (gdb) debug-motor-position -6 3
     (gdb) debug-motor-position 4 2
     (gdb) debug-motor-position -6 3
     (gdb) debug-motor-position 4 2 force=35
     (gdb) debug-motor-position -6 3 force=35
     python3 identify_zmotor.py coil <the last four captures>

   The voltage over each run's constant-speed stretch is compared. A load from
   the coil shifts climbs and descents the same way; extra friction from it
   opposes both, so it shifts them apart. A diagnostic only: the firmware has
   no coil-load compensation, because the 2026-10-09 measurement found none.

5. moves -- how smoothly closed-loop moves run (FIRMWARE_MODE_DEBUG_MOTOR_
   POSITION), e.g. profiled climbs like the protocols make:

     (gdb) debug-motor-position -6 3
     (gdb) debug-motor-position 4 2 speed=10
     python3 identify_zmotor.py moves <captures>

   Reports, per move: its duration, how unevenly it ran, how far it trailed
   the setpoint, how often the motor voltage changed sign (each change takes
   the gears across their backlash), and how much of it was spent slower than
   the speed below which the spring outruns a climb.

The fit uses only position and voltage, never the velocity column: that is the
estimate made from the very model being identified.
"""

import argparse
import csv
import glob
import math
import os
import random
import sys

DT = 1.0e-3                      # control period, DCMOTOR_*_CONTROL_FREQUENCY
PID_OUTPUT_LIMIT = 13.5          # DCMOTOR_VELOCITY_MODULE_PID_OUTPUT_MIN/MAX (V)
DRIVE_LIMIT = (0.95 - 0.5) * 27.0  # duty clamp expressed in volts
PID_LEAK_TC = 10.0               # DCMOTOR_VELOCITY_MODULE_PID_LEAKAGE_TC

# Readings ignored at the start of a capture. Firmware since the LVDT settling
# fix already drops them (LVDT_MODULE_SETTLING_SAMPLES); captures taken before
# it begin with readings that can be millimetres off.
SETTLING_SKIP = 5
SKIP_HELP = "readings to ignore at the start of each capture (LVDT settling)"

# configuration.h placeholders, used when a value is not given.
DEFAULT_POSITION_NOISE = 0.002   # mm
DEFAULT_ACCEL_NOISE = 200.0      # mm/s^2
DEFAULT_DISTURBANCE_DRIFT = 2.0  # V/sqrt(s)


# -----------------------------------------------------------------------------
# Captures
# -----------------------------------------------------------------------------

def captures_dir_default():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(repo_root, "captures")


def newest_capture(captures_dir):
    found = sorted(glob.glob(os.path.join(captures_dir, "motor_velocity_*.csv")),
                   key=os.path.getmtime)
    if not found:
        sys.exit("no motor_velocity CSVs found in " + captures_dir)
    return found[-1]


def read_capture(path):
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or "position" not in reader.fieldnames:
            sys.exit("%s has no position column -- it predates the LVDT-only "
                     "firmware (tachometer capture)" % path)
        capture = {"path": path, "mode": None, "time": [], "position": [],
                   "velocity": [], "voltage": []}
        for row in reader:
            capture["mode"] = row.get("mode", capture["mode"])
            capture["time"].append(float(row["time_s"]))
            capture["position"].append(float(row["position"]))
            capture["velocity"].append(float(row["velocity"]))
            capture["voltage"].append(float(row["voltage"]))
    if len(capture["position"]) < 20:
        sys.exit("%s is too short to analyse" % path)
    return capture


def mean(values):
    return sum(values) / len(values) if values else 0.0


# -----------------------------------------------------------------------------
# Small linear algebra (the scripts here avoid numpy)
# -----------------------------------------------------------------------------

def solve(matrix, rhs):
    """Gaussian elimination with partial pivoting. None if singular."""
    n = len(rhs)
    a = [list(matrix[i]) + [rhs[i]] for i in range(n)]
    scale = max(abs(a[i][i]) for i in range(n)) or 1.0
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-12 * scale:
            return None
        a[col], a[pivot] = a[pivot], a[col]
        for r in range(col + 1, n):
            factor = a[r][col] / a[col][col]
            if factor:
                for c in range(col, n + 1):
                    a[r][c] -= factor * a[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (a[r][n] - sum(a[r][c] * x[c] for c in range(r + 1, n))) / a[r][r]
    return x


def linear_fit(xs, ys):
    mx, my = mean(xs), mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx else 0.0
    return slope, my - slope * mx


# -----------------------------------------------------------------------------
# Motor model and Kalman filter -- the same discretisation as
# ZAxisKalmanFilter::initialize()
# -----------------------------------------------------------------------------

def discretise(tau):
    a = math.exp(-DT / tau)
    return a, tau * (1.0 - a)


def kalman_gain(km, tau, position_noise, accel_noise, disturbance_drift):
    a, g = discretise(tau)
    f = [[1.0, g, km * (DT - g)],
         [0.0, a, km * (1.0 - a)],
         [0.0, 0.0, 1.0]]
    q = [0.0, (accel_noise * DT) ** 2, disturbance_drift ** 2 * DT]
    r = position_noise ** 2
    p = [[r, 0.0, 0.0], [0.0, 1e2, 0.0], [0.0, 0.0, 1e2]]
    k = [0.0, 0.0, 0.0]
    for iteration in range(20000):
        s = p[0][0] + r
        k_next = [p[i][0] / s for i in range(3)]
        change = max(abs(k_next[i] - k[i]) / (1.0 + abs(k_next[i])) for i in range(3))
        k = k_next
        post = [[p[i][j] - k[i] * p[0][j] for j in range(3)] for i in range(3)]
        fp = [[sum(f[i][m] * post[m][j] for m in range(3)) for j in range(3)]
              for i in range(3)]
        p = [[sum(fp[i][m] * f[j][m] for m in range(3)) + (q[i] if i == j else 0.0)
              for j in range(3)] for i in range(3)]
        if iteration > 0 and change < 1e-9:
            return f, k
    sys.exit("Kalman gain did not converge for these values")


class Estimator:
    def __init__(self, f, k, km, tau):
        a, g = discretise(tau)
        self.f = f
        self.k = k
        self.b = [km * (DT - g), km * (1.0 - a), 0.0]
        self.x = [0.0, 0.0, 0.0]

    def correct(self, z):
        innovation = z - self.x[0]
        self.x = [self.x[i] + self.k[i] * innovation for i in range(3)]

    def predict(self, voltage):
        self.x = [sum(self.f[i][j] * self.x[j] for j in range(3)) +
                  self.b[i] * voltage for i in range(3)]


class Pid:
    """PidController::execute(), including the integral clamp and leak."""

    def __init__(self, gain, ti, td, filter_tc):
        self.gain, self.ti, self.td = gain, ti, td
        self.alpha = DT / (filter_tc + DT) if filter_tc > 0.0 else 1.0
        self.decay = math.exp(-DT / PID_LEAK_TC)
        self.integral = 0.0
        self.previous = 0.0

    def execute(self, setpoint, measured):
        error = self.alpha * (setpoint - measured) + (1.0 - self.alpha) * self.previous
        derivative = (error - self.previous) / DT
        step = error * DT if self.ti > 0.0 else 0.0
        self.integral = (self.integral + step) * self.decay
        integral_term = self.integral / self.ti if self.ti > 0.0 else 0.0
        raw = self.gain * (error + integral_term + self.td * derivative)
        out = max(-PID_OUTPUT_LIMIT, min(PID_OUTPUT_LIMIT, raw))
        if raw != out and self.gain * step * raw > 0.0:
            self.integral -= step
        self.previous = error
        return out


# -----------------------------------------------------------------------------
# noise
# -----------------------------------------------------------------------------

def run_noise(args):
    path = args.file or newest_capture(args.captures)
    cap = read_capture(path)
    t, x = cap["time"][args.skip:], cap["position"][args.skip:]

    slope, intercept = linear_fit(t, x)
    residual = [xi - (slope * ti + intercept) for ti, xi in zip(t, x)]
    sigma = math.sqrt(mean([r * r for r in residual]))
    diffs = [residual[i + 1] - residual[i] for i in range(len(residual) - 1)]
    diff_velocity = math.sqrt(mean([d * d for d in diffs])) / DT
    lag1 = (sum(residual[i] * residual[i + 1] for i in range(len(residual) - 1)) /
            sum(r * r for r in residual)) if sigma > 0.0 else 0.0
    drive = mean(cap["voltage"])
    duration = t[-1] - t[0]

    print("capture          : %s (%d samples, %.2f s)" % (path, len(x), duration))
    print("mean drive       : %.3f V" % drive)
    print("drift            : %.4f mm/s (%.1f um over the capture)" %
          (slope, 1000.0 * slope * duration))
    print("LVDT noise       : %.2f um RMS, %.2f um peak-to-peak" %
          (1000.0 * sigma, 1000.0 * (max(residual) - min(residual))))
    print("lag-1 correlation: %.2f" % lag1)
    print("raw-difference velocity noise: %.2f mm/s RMS (what the filter avoids)" %
          diff_velocity)

    if abs(slope) * duration > 10.0 * sigma:
        print("\nwarning: the head moved during the capture; the trend was removed,"
              " but a capture pushed onto the bottom stop is cleaner")
    if lag1 > 0.5:
        print("\nwarning: the noise is strongly correlated from tick to tick, so it"
              " is not the white noise the filter assumes. The RMS above overstates"
              " how much it hurts velocity; consider a smaller value after checking"
              " the closed loop")

    print("\n#define ZAXIS_KALMAN_POSITION_NOISE                                  "
          "%.4ff /* mm, 1 sigma */" % sigma)


# -----------------------------------------------------------------------------
# fit
# -----------------------------------------------------------------------------

def smoothed_velocity(x, half_width=10):
    n = len(x)
    v = [0.0] * n
    for k in range(n):
        lo, hi = max(0, k - half_width), min(n - 1, k + half_width)
        v[k] = (x[hi] - x[lo]) / ((hi - lo) * DT) if hi > lo else 0.0
    return v


def fit_window(cap, end_s):
    """Samples to fit: from the start until the head stops at a travel limit."""
    x = cap["position"]
    n = len(x)
    if end_s is not None:
        return min(n, max(2, int(end_s / DT)))
    v = smoothed_velocity(x)
    peak = max(range(n), key=lambda k: abs(v[k]))
    for k in range(peak, n):
        if abs(v[k]) < 0.3 * abs(v[peak]):
            return max(2, k - 10)
    return n


class Segment:
    def __init__(self, cap, end_s, skip):
        self.path = cap["path"]
        self.mode = cap["mode"]
        n = fit_window(cap, end_s)
        self.voltage = cap["voltage"][:n]
        x0 = cap["position"][0]
        # Relative to the first sample, so the sums stay well conditioned.
        self.position = [p - x0 for p in cap["position"][:n]]
        # The model still runs from the first tick, where the drive starts;
        # only the readings taken while the LVDT settled are left out.
        self.skip = min(skip, n - 2)
        settled = self.position[self.skip:]
        self.travel = max(settled) - min(settled)
        self.direction = "up" if mean(self.voltage) >= 0.0 else "down"


def responses(segment, tau, delay):
    """Position responses to the drive and to a unit load, unit motor gain."""
    a, g = discretise(tau)
    out_u, out_1 = [], []
    xu = vu = x1 = v1 = 0.0
    for k in range(len(segment.position)):
        out_u.append(xu)
        out_1.append(x1)
        active = k >= delay
        w = segment.voltage[k - delay] if active else 0.0
        one = 1.0 if active else 0.0
        xu, vu = xu + g * vu + (DT - g) * w, a * vu + (1.0 - a) * w
        x1, v1 = x1 + g * v1 + (DT - g) * one, a * v1 + (1.0 - a) * one
    return out_u, out_1


def regress(segments, tau, delay, with_load):
    """Least squares for [Km, Km*load_up, Km*load_down, offset per segment]."""
    columns = ["gain"]
    if with_load:
        columns += [d for d in ("up", "down") if any(s.direction == d for s in segments)]
    columns += ["offset%d" % i for i in range(len(segments))]
    size = len(columns)
    ata = [[0.0] * size for _ in range(size)]
    aty = [0.0] * size
    rows = []
    for index, segment in enumerate(segments):
        s_u, s_1 = responses(segment, tau, delay)
        for k, y in enumerate(segment.position):
            if k < segment.skip:
                continue
            row = [0.0] * size
            row[0] = s_u[k]
            if with_load:
                row[columns.index(segment.direction)] = s_1[k]
            row[columns.index("offset%d" % index)] = 1.0
            rows.append((index, row, y))
            for i in range(size):
                if row[i]:
                    aty[i] += row[i] * y
                    for j in range(size):
                        if row[j]:
                            ata[i][j] += row[i] * row[j]
    theta = solve(ata, aty)
    if theta is None:
        return None
    sse = [0.0] * len(segments)
    for index, row, y in rows:
        e = y - sum(r * t for r, t in zip(row, theta))
        sse[index] += e * e
    return dict(zip(columns, theta)), sse


def best_tau(segments, delay, with_load):
    def cost(log_tau):
        result = regress(segments, math.exp(log_tau), delay, with_load)
        return (sum(result[1]) if result else float("inf")), result

    lo, hi = math.log(0.5e-3), math.log(2.0)
    grid = [lo + (hi - lo) * i / 39 for i in range(40)]
    costs = [cost(g)[0] for g in grid]
    i = min(range(len(grid)), key=lambda j: costs[j])
    a, b = grid[max(0, i - 1)], grid[min(len(grid) - 1, i + 1)]
    ratio = (math.sqrt(5.0) - 1.0) / 2.0
    for _ in range(30):
        c, d = b - ratio * (b - a), a + ratio * (b - a)
        if cost(c)[0] < cost(d)[0]:
            b = d
        else:
            a = c
    log_tau = (a + b) / 2.0
    total, result = cost(log_tau)
    at_edge = i in (0, len(grid) - 1)
    return math.exp(log_tau), total, result, at_edge


def run_fit(args):
    paths = list(args.files)
    if args.all:
        paths += sorted(glob.glob(os.path.join(args.captures, "motor_velocity_*.csv")),
                        key=os.path.getmtime)
    if not paths:
        sys.exit("give the step captures to fit, or --all")

    segments = []
    for path in paths:
        segment = Segment(read_capture(path), args.end, args.skip)
        if max(abs(v) for v in segment.voltage) < 0.3:
            print("skipped %s: no drive (a rest capture?)" % path)
            continue
        if segment.travel < 0.02:
            print("skipped %s: the head did not move (stiction? raise the drive)" % path)
            continue
        segments.append(segment)
    if not segments:
        sys.exit("nothing to fit")

    # With a single drive size per direction the load and the gain move the
    # head in exactly the same shape, so they cannot be told apart.
    with_load = regress(segments, 0.02, 0, True) is not None
    if not with_load:
        print("gain and load are not separable from these captures, so the load"
              " is assumed zero and the gain will absorb it -- add captures at a"
              " second drive size (not just the opposite sign)")

    best = None
    for delay in range(0, 4):
        tau, total, result, at_edge = best_tau(segments, delay, with_load)
        if result is not None and (best is None or total < best[1]):
            best = (delay, total, tau, result, at_edge)
    if best is None:
        sys.exit("the fit failed; check the captures")

    delay, _, tau, (theta, sse), at_edge = best
    km = theta["gain"]

    print("\n%-44s %6s %7s %8s %9s" % ("capture", "dir", "drive V", "travel", "fit RMS"))
    for segment, error in zip(segments, sse):
        print("%-44s %6s %7.2f %6.3fmm %7.2fum" % (
            os.path.basename(segment.path)[-44:], segment.direction,
            mean(segment.voltage), segment.travel,
            1000.0 * math.sqrt(error / (len(segment.position) - segment.skip))))

    poor = [seg for seg, error in zip(segments, sse)
            if math.sqrt(error / (len(seg.position) - seg.skip)) > 0.02 * seg.travel]
    if poor:
        print("\nwarning: the model does not fit %s well (residual above 2%% of the"
              " travel). Check for a run that hit something, or add drive levels."
              % ", ".join(os.path.basename(seg.path) for seg in poor))

    print("\nmotor gain     : %.4f mm/s per V" % km)
    print("time constant  : %.4f s%s" % (tau, "  (at the search limit!)" if at_edge else ""))
    print("input delay    : %d tick(s)" % delay)
    if km <= 0.0:
        print("\nERROR: the gain is negative -- a positive voltage moved the head"
              " down. ZMOTOR_DRIVE_DIRECTION and LVDT_MODULE_DIRECTION disagree;"
              " closing the loop like this runs away. Fix those first.")
        return
    if with_load:
        up = theta.get("up")
        down = theta.get("down")
        if up is not None:
            print("load moving up : %+.3f V" % (up / km))
        if down is not None:
            print("load moving down: %+.3f V" % (down / km))
        if up is not None and down is not None:
            print("  -> gravity %+.3f V, friction %.3f V" %
                  ((up + down) / (2.0 * km), (down - up) / (2.0 * km)))
            print("  (the Kalman disturbance state carries these at run time)")
    if delay >= 2:
        print("\nnote: %d ticks of delay is more than the filter models; if the"
              " closed loop rings, the model needs a delay state" % delay)

    print("\n#define ZMOTOR_MODEL_GAIN                                            "
          "%.4ff /* mm/s per V */" % km)
    print("#define ZMOTOR_MODEL_TIME_CONSTANT                                   "
          "%.4ff /* s */" % tau)
    print("\nnext: python3 identify_zmotor.py pid --gain %.4f --tau %.4f --delay %d"
          " --noise <from the noise step>" % (km, tau, delay))


# -----------------------------------------------------------------------------
# pid
# -----------------------------------------------------------------------------

def simulate(args, f, k, gain, ti, noisy, seconds=0.6, seed=1):
    """The firmware tick against the identified plant; returns metrics.

    The step shape (rise, overshoot, settling) is read from a noise-free run;
    a noisy run gives the drive and velocity noise. Mixing them would let a
    noise spike pass for overshoot or keep the response from ever settling."""
    rng = random.Random(seed)
    sigma = args.noise if noisy else 0.0
    estimator = Estimator(f, k, args.gain, args.tau)
    pid = Pid(gain, ti, 0.0, 0.0)
    a, g = discretise(args.tau)
    x = v = 0.0
    pending = [0.0] * args.delay
    start = int(0.05 / DT)
    velocity, drive = [], []
    for tick in range(int(seconds / DT)):
        target = args.step if tick >= start else 0.0
        estimator.correct(x + rng.gauss(0.0, sigma))
        out = pid.execute(target, estimator.x[1])
        applied = max(-DRIVE_LIMIT, min(DRIVE_LIMIT, out))
        estimator.predict(applied)
        pending.append(applied)
        u = pending.pop(0) + args.load
        x, v = x + g * v + args.gain * (DT - g) * u, a * v + args.gain * (1.0 - a) * u
        velocity.append(v)
        drive.append(applied)

    after = velocity[start:]
    tail = after[int(0.6 * len(after)):]
    steady = mean(tail)
    peak = max(after) if args.step > 0 else min(after)
    overshoot = max(0.0, (peak - steady) / args.step * 100.0)

    def first_at(fraction):
        for i, value in enumerate(after):
            if value >= fraction * args.step:
                return i * DT
        return None

    t10, t90 = first_at(0.1), first_at(0.9)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    settle = None
    for i in range(len(after) - 1, -1, -1):
        if abs(after[i] - args.step) > 0.05 * abs(args.step):
            settle = (i + 1) * DT if i + 1 < len(after) else None
            break
    drive_tail = drive[start + int(0.6 * len(after)):]
    drive_mean = mean(drive_tail)
    drive_noise = math.sqrt(mean([(d - drive_mean) ** 2 for d in drive_tail]))
    saturated = sum(1 for d in drive_tail if abs(d) >= DRIVE_LIMIT - 1e-6)
    velocity_noise = math.sqrt(mean([(w - steady) ** 2 for w in tail]))
    return {"rise": rise, "overshoot": overshoot, "settle": settle,
            "drive_noise": drive_noise, "velocity_noise": velocity_noise,
            "steady_error": steady - args.step, "saturated": saturated > 0}


def run_pid(args):
    if args.gain <= 0.0 or args.tau <= 0.0:
        sys.exit("--gain and --tau must be positive (from the fit step)")

    f, k = kalman_gain(args.gain, args.tau, args.noise, args.accel_noise,
                       args.disturbance_drift)
    print("Kalman gain    : position %.4f, velocity %.2f, disturbance %.2f" % tuple(k))

    # PI with the integral time on the motor's time constant, so the zero
    # cancels the motor pole and the loop gain alone sets the bandwidth.
    ti = args.tau
    if args.bandwidth:
        bandwidths = [args.bandwidth]
    else:
        bandwidths = [1.0 + i for i in range(0, 80)]

    rows = []
    for bandwidth in bandwidths:
        gain = 2.0 * math.pi * bandwidth * args.tau / args.gain
        metrics = simulate(args, f, k, gain, ti, noisy=False)
        noisy = simulate(args, f, k, gain, ti, noisy=True)
        for key in ("drive_noise", "velocity_noise", "saturated"):
            metrics[key] = noisy[key]
        ok = (metrics["overshoot"] <= args.max_overshoot and
              metrics["drive_noise"] <= args.max_drive_noise and
              not metrics["saturated"] and metrics["settle"] is not None)
        rows.append((bandwidth, gain, metrics, ok))

    def fmt(value, scale=1.0, unit=""):
        return "-" if value is None else "%.1f%s" % (value * scale, unit)

    print("\n%7s %8s %8s %9s %9s %10s %10s" % (
        "bw Hz", "Kp V/mm/s", "rise", "overshoot", "settle", "drive rms", "vel rms"))
    shown = rows if args.bandwidth else [r for r in rows if r[0] in
                                         (2, 5, 10, 15, 20, 30, 40, 60, 80)]
    for bandwidth, gain, m, ok in shown:
        print("%7.0f %8.4f %8s %8.1f%% %9s %8.2f V %7.3f mm/s%s" % (
            bandwidth, gain, fmt(m["rise"], 1000.0, "ms"), m["overshoot"],
            fmt(m["settle"], 1000.0, "ms"), m["drive_noise"], m["velocity_noise"],
            "" if ok else "  x"))

    good = [r for r in rows if r[3]]
    if not good:
        print("\nno candidate met the limits (overshoot <= %.0f%%, drive noise <= %.1f V);"
              " lower the bandwidth, or revisit the noise values" %
              (args.max_overshoot, args.max_drive_noise))
        return
    bandwidth, gain, m, _ = good[-1]
    print("\nchosen: %.0f Hz -- rise %s, overshoot %.1f%%, settle %s, drive noise %.2f V" % (
        bandwidth, fmt(m["rise"], 1000.0, " ms"), m["overshoot"],
        fmt(m["settle"], 1000.0, " ms"), m["drive_noise"]))
    print("(simulated against the identified model; confirm with a closed-loop"
          " step on the machine)")

    # The position loop is a proportional gain (1/s) wrapped around this one;
    # it wants the inner loop at least ~3x faster than itself.
    position_gain = 2.0 * math.pi * bandwidth / 3.0
    print("position loop: keep DCMOTOR_POSITION_MODULE_PROPORTIONAL_GAIN at or below"
          " %.0f" % position_gain)

    print("\n#define DCMOTOR_VELOCITY_MODULE_PID_GAIN                             %.4ff" % gain)
    print("#define DCMOTOR_VELOCITY_MODULE_PID_INTEGRAL_TC                      %.4ff" % ti)
    print("#define DCMOTOR_VELOCITY_MODULE_PID_DERIVATIVE_TC                    0.0f")
    print("#define DCMOTOR_VELOCITY_MODULE_PID_INPUT_FILTER_TC                  0.0f")
    if (args.noise, args.accel_noise, args.disturbance_drift) != (
            DEFAULT_POSITION_NOISE, DEFAULT_ACCEL_NOISE, DEFAULT_DISTURBANCE_DRIFT):
        print("#define ZAXIS_KALMAN_POSITION_NOISE                                  "
              "%.4ff /* mm, 1 sigma */" % args.noise)
        print("#define ZAXIS_KALMAN_ACCELERATION_NOISE                              "
              "%.1ff /* mm/s^2, 1 sigma */" % args.accel_noise)
        print("#define ZAXIS_KALMAN_DISTURBANCE_DRIFT                               "
              "%.2ff   /* V/sqrt(s) */" % args.disturbance_drift)


# -----------------------------------------------------------------------------
# coil
# -----------------------------------------------------------------------------

def cruise(path):
    """Mean voltage, coil current and speed over the middle of a run's travel,
    and how unevenly it moved there, or None if it barely moved."""
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows or "coil_a" not in rows[0]:
        sys.exit("%s has no coil current -- it predates the force= option" % path)

    position = [float(r["position"]) for r in rows]
    voltage = [float(r["voltage"]) for r in rows]
    current = [float(r["coil_a"]) for r in rows]
    n = len(position)

    start = sorted(position[:5])[2]
    end = sorted(position[-100:])[50]
    travel = end - start
    if abs(travel) < 1.0:
        return None

    # The middle 20-80 % of the travel: past the start-up and before the
    # position loop's final approach. A run this uneven has no clean
    # constant-speed stretch, so the stretch is chosen by distance, and its
    # speed is distance over time rather than an average of noisy rates.
    def progress(k):
        return (position[k] - start) / travel
    first = next((k for k in range(n) if progress(k) >= 0.2), None)
    last = next((k for k in range(n - 1, -1, -1) if progress(k) <= 0.8), None)
    if first is None or last is None or last - first < 50:
        return None
    window = range(first, last + 1)

    speed = (position[last] - position[first]) / ((last - first) * DT)
    # Unevenness: the 10 ms speed against the mean, as a fraction of it.
    local = smoothed_velocity(position, half_width=5)
    rough = math.sqrt(mean([(local[k] - speed) ** 2 for k in window])) / abs(speed)

    return {
        "path": path,
        "direction": "up" if travel > 0 else "down",
        "speed": speed,
        "rough": rough,
        "voltage": mean([voltage[k] for k in window]),
        "voltage_sd": math.sqrt(mean([(voltage[k] - mean([voltage[m] for m in window])) ** 2
                                      for k in window])),
        "current": mean([current[k] for k in window]),
        "samples": len(window),
    }


def run_coil(args):
    runs = []
    for path in args.files:
        run = cruise(path)
        if run is None:
            print("skipped %s: it hardly moved" % path)
            continue
        run["coil"] = run["current"] > args.on_threshold
        runs.append(run)

    print("%-36s %5s %5s %10s %8s %16s %9s" %
          ("capture", "dir", "coil", "speed", "uneven", "voltage (sd)", "current"))
    for run in runs:
        print("%-36s %5s %5s %6.2fmm/s %7.0f%% %+7.3fV (%.2f) %8.4fA" % (
            os.path.basename(run["path"])[-36:], run["direction"],
            "on" if run["coil"] else "off", run["speed"], 100.0 * run["rough"],
            run["voltage"], run["voltage_sd"], run["current"]))

    groups = {}
    for run in runs:
        groups.setdefault((run["direction"], run["coil"]), []).append(run)
    missing = [key for key in (("up", False), ("up", True),
                               ("down", False), ("down", True))
               if key not in groups]
    if missing:
        sys.exit("\nneed a climb and a descent with the coil both off and on; "
                 "missing: %s" % ", ".join("%s/coil %s" % (d, "on" if c else "off")
                                           for d, c in missing))

    def average(direction, coil, key):
        return mean([run[key] for run in groups[(direction, coil)]])

    dv_up = average("up", True, "voltage") - average("up", False, "voltage")
    dv_down = average("down", True, "voltage") - average("down", False, "voltage")
    di = mean([average(d, True, "current") - average(d, False, "current")
               for d in ("up", "down")])
    speed_change = max(abs(average(d, True, "speed") / average(d, False, "speed") - 1.0)
                       for d in ("up", "down"))

    # The coil's load adds the same to both directions; extra friction
    # opposes the motion, so it adds to climbs and subtracts from descents.
    load = -(dv_up + dv_down) / 2.0
    friction = (dv_up - dv_down) / 2.0

    print("\nwith the coil on (%.4f A more):" % di)
    print("  climb voltage   %+.3f V" % dv_up)
    print("  descent voltage %+.3f V" % dv_down)
    print("  -> load from the coil : %+.3f V (%+.3f V per A)%s" % (
        load, load / di,
        "  pulls the head down" if load < 0 else "  pushes the head up"))
    print("  -> extra friction     : %.3f V" % friction)
    if speed_change > 0.1:
        print("\nwarning: the cruise speed differs by %.0f%% between coil off and"
              " on; the loop did not hold speed, so the voltages compare unlike"
              " runs" % (100.0 * speed_change))
    if abs(load) < 0.1 and abs(friction) < 0.1:
        print("\nNeither is measurable (both under 0.1 V): the coil has no"
              " effect on the Z drive worth compensating.")
    elif abs(friction) > abs(load):
        print("\nThe extra friction is larger than the load. Compensating the"
              " load alone will not smooth the motion much; the coil's effect is"
              " mostly friction, which no known-load term can cancel.")


# -----------------------------------------------------------------------------
# moves
# -----------------------------------------------------------------------------

def analyse_move(path, slow_speed, hysteresis):
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows or "voltage" not in rows[0]:
        return None

    position = [float(r["position"]) for r in rows]
    voltage = [float(r["voltage"]) for r in rows]
    setpoint = ([float(r["setpoint"]) for r in rows]
                if "setpoint" in rows[0] else None)
    n = len(position)

    start = sorted(position[:5])[2]
    end = sorted(position[-100:])[50]
    travel = end - start
    if abs(travel) < 0.5:
        return None

    # The move: from leaving the start to settling within 50 um of the end.
    # A slow move can outlast the capture; it is then measured to the end.
    first = next((k for k in range(n) if abs(position[k] - start) > 0.02), None)
    settled = next((k for k in range(n)
                    if all(abs(position[m] - end) < 0.05 for m in range(k, min(n, k + 50)))),
                   None)
    finished = settled is not None and settled < n - 50
    if not finished:
        settled = n
    if first is None or settled <= first + 20:
        return None
    move = range(first, settled)

    local = smoothed_velocity(position, half_width=5)
    lo = next(k for k in range(n) if (position[k] - start) / travel >= 0.2)
    hi = next(k for k in range(n - 1, -1, -1) if (position[k] - start) / travel <= 0.8)
    mid = range(lo, max(lo + 1, hi + 1))
    speed = (position[hi] - position[lo]) / (max(1, hi - lo) * DT)
    uneven = (math.sqrt(mean([(local[k] - speed) ** 2 for k in mid])) / abs(speed)
              if speed else float("nan"))

    # Sign changes of the drive, with a little hysteresis so noise around
    # zero is not counted: each one is a trip across the gear backlash.
    flips, sign = 0, 0
    for k in move:
        if voltage[k] > hysteresis:
            new = 1
        elif voltage[k] < -hysteresis:
            new = -1
        else:
            continue
        if sign and new != sign:
            flips += 1
        sign = new

    slow = sum(1 for k in move if abs(local[k]) < slow_speed) / len(move)
    # Lag only means something against a walked setpoint; a plain step jumps
    # straight to the target.
    profiled = (setpoint is not None and
                max(setpoint) - min(setpoint) > 0.5 and
                max(abs(setpoint[k + 1] - setpoint[k]) for k in range(n - 1)) < 0.05)
    lag = (math.sqrt(mean([(position[k] - setpoint[k]) ** 2 for k in move]))
           if profiled else None)

    # Past the target in the direction of travel, against the walked
    # setpoint's end (or the settled position for a plain step); and the
    # drive left on once the head should be resting.
    target = setpoint[-1] if profiled else end
    beyond = [(position[k] - target) * (1.0 if travel > 0 else -1.0)
              for k in range(first, n)]
    overshoot = max(0.0, max(beyond))
    tail = range(max(first, n - int(0.3 / DT)), n)
    rest_drive = mean([abs(voltage[k]) for k in tail])

    return {
        "path": path, "travel": travel, "duration": len(move) * DT,
        "speed": speed, "uneven": uneven, "flips": flips, "slow": slow,
        "lag": lag, "finished": finished, "overshoot": overshoot,
        "rest_drive": rest_drive,
    }


def run_moves(args):
    print("%-36s %5s %7s %8s %10s %7s %8s %6s %6s %8s %7s" % (
        "capture", "dir", "travel", "time", "mid speed", "uneven", "lag rms",
        "flips", "slow", "overshot", "rest |V|"))
    for path in args.files:
        m = analyse_move(path, args.slow_speed, args.hysteresis)
        if m is None:
            print("%-36s  (no closed-loop move found)" % os.path.basename(path)[-36:])
            continue
        print("%-36s %5s %6.2fmm %7.3fs %6.2fmm/s %6.0f%% %8s %6d %5.0f%% %6.0fum %6.2fV" % (
            os.path.basename(m["path"])[-36:],
            "up" if m["travel"] > 0 else "down", abs(m["travel"]), m["duration"],
            m["speed"], 100.0 * m["uneven"],
            "-" if m["lag"] is None else "%.0fum" % (1000.0 * m["lag"]),
            m["flips"], 100.0 * m["slow"], 1000.0 * m["overshoot"],
            m["rest_drive"]) +
            ("" if m["finished"] else "  (still moving at capture end)"))
    print("\nflips: drive sign changes during the move (gear backlash crossings);"
          " slow: share of the move below %.1f mm/s;\novershot: furthest past the"
          " target; rest |V|: mean drive over the capture's last 0.3 s"
          % args.slow_speed)


# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--captures", default=captures_dir_default(),
                        help="captures directory")
    sub = parser.add_subparsers(dest="step", required=True)

    noise = sub.add_parser("noise", help="LVDT noise from a capture at rest")
    noise.add_argument("file", nargs="?", help="capture (default: newest)")
    noise.add_argument("--skip", type=int, default=SETTLING_SKIP, help=SKIP_HELP)
    noise.set_defaults(run=run_noise)

    fit = sub.add_parser("fit", help="motor model from open-loop steps")
    fit.add_argument("files", nargs="*", help="step captures")
    fit.add_argument("--all", action="store_true",
                     help="also use every motor_velocity capture in --captures")
    fit.add_argument("--end", type=float,
                     help="fit only the first END seconds of each capture "
                          "(default: until the head stops at a limit)")
    fit.add_argument("--skip", type=int, default=SETTLING_SKIP, help=SKIP_HELP)
    fit.set_defaults(run=run_fit)

    coil = sub.add_parser("coil", help="force-coil load from moving captures")
    coil.add_argument("files", nargs="+",
                      help="motor_position captures: climbs and descents, coil off and on")
    coil.add_argument("--on-threshold", type=float, default=0.01,
                      help="coil current (A) above which a run counts as coil on")
    coil.set_defaults(run=run_coil)

    moves = sub.add_parser("moves", help="smoothness of closed-loop moves")
    moves.add_argument("files", nargs="+", help="motor_position captures")
    moves.add_argument("--slow-speed", type=float, default=4.0,
                       help="speed (mm/s) below which the spring outruns a climb")
    moves.add_argument("--hysteresis", type=float, default=0.2,
                       help="volts around zero ignored when counting sign changes")
    moves.set_defaults(run=run_moves)

    pid = sub.add_parser("pid", help="velocity PI gains by simulation")
    pid.add_argument("--gain", type=float, required=True, help="motor gain, mm/s per V")
    pid.add_argument("--tau", type=float, required=True, help="time constant, s")
    pid.add_argument("--delay", type=int, default=1, help="input delay, ticks")
    pid.add_argument("--noise", type=float, default=DEFAULT_POSITION_NOISE,
                     help="LVDT noise, mm (from the noise step)")
    pid.add_argument("--accel-noise", type=float, default=DEFAULT_ACCEL_NOISE,
                     help="Kalman acceleration noise, mm/s^2")
    pid.add_argument("--disturbance-drift", type=float, default=DEFAULT_DISTURBANCE_DRIFT,
                     help="Kalman disturbance drift, V/sqrt(s)")
    pid.add_argument("--load", type=float, default=0.0,
                     help="constant load on the plant, V (e.g. gravity from the fit)")
    pid.add_argument("--step", type=float, default=3.0, help="velocity step, mm/s")
    pid.add_argument("--bandwidth", type=float, help="evaluate one bandwidth, Hz")
    pid.add_argument("--max-overshoot", type=float, default=10.0, help="percent")
    # Chatter on the drive is current through a motor that is not moving
    # anywhere -- heat. Kept low on purpose.
    pid.add_argument("--max-drive-noise", type=float, default=0.5, help="V RMS")
    pid.set_defaults(run=run_pid)

    args = parser.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
