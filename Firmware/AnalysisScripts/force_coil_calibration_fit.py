#!/usr/bin/env python3
"""Fit force-coil current -> bond force (grams) and check linearity.

Reads Core/Inc/force_coil_current_to_gram.txt (two columns: current in A,
weight in grams) and fits:

    grams = a * current + b            (linear)
    grams = p * current^2 + q * current + r   (quadratic, for comparison)

Reports R^2 for both, per-point residuals for the linear fit, and a verdict
on whether the linear model is an adequate description of the data. Pure
stdlib, no numpy/scipy dependency (matches tune_zmotor.py's convention so it
runs on the bench laptop with no extra installs).

Usage:
    python3 force_coil_calibration_fit.py [path/to/force_coil_current_to_gram.txt]
"""

import math
import os
import re
import sys

DEFAULT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "Core", "Inc", "force_coil_current_to_gram.txt")


def parse_data(path):
    """Returns (fit_points, excluded_points) as lists of (current, grams)."""
    fit_points = []
    excluded_points = []

    with open(path) as f:
        lines = f.readlines()

    row_re = re.compile(r"^\s*(<=)?\s*([0-9.]+)\s+([0-9.]+)\s*$")

    for line in lines:
        if not line.strip():
            continue
        m = row_re.match(line)
        if not m:
            continue  # header or unparsable line

        is_threshold, current_str, weight_str = m.groups()
        current = float(current_str)
        weight = float(weight_str)

        if is_threshold:
            # "<=0.05" isn't a clean single-current data point (it reads as
            # a floor/deadband below which the coil doesn't develop
            # measurable force) -- excluded from the fit, reported separately.
            excluded_points.append((current, weight))
        else:
            fit_points.append((current, weight))

    return fit_points, excluded_points


def linear_fit(points):
    n = len(points)
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    sxy = sum(x * y for x, y in points)

    denom = n * sxx - sx * sx
    a = (n * sxy - sx * sy) / denom          # slope
    b = (sxx * sy - sx * sxy) / denom        # intercept
    return a, b


def quadratic_fit(points):
    # Normal equations for grams = p*x^2 + q*x + r, solved with Cramer's rule.
    n = len(points)
    s1 = n
    sx = sum(x for x, _ in points)
    sx2 = sum(x**2 for x, _ in points)
    sx3 = sum(x**3 for x, _ in points)
    sx4 = sum(x**4 for x, _ in points)
    sy = sum(y for _, y in points)
    sxy = sum(x * y for x, y in points)
    sx2y = sum(x * x * y for x, y in points)

    # [ sx4 sx3 sx2 ] [p]   [sx2y]
    # [ sx3 sx2 sx  ] [q] = [sxy ]
    # [ sx2 sx  s1  ] [r]   [sy  ]
    A = [[sx4, sx3, sx2], [sx3, sx2, sx], [sx2, sx, s1]]
    B = [sx2y, sxy, sy]

    def det3(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

    d = det3(A)
    coeffs = []
    for col in range(3):
        Ac = [row[:] for row in A]
        for r in range(3):
            Ac[r][col] = B[r]
        coeffs.append(det3(Ac) / d)
    p, q, r = coeffs
    return p, q, r


def r_squared(points, predict):
    y_mean = sum(y for _, y in points) / len(points)
    ss_tot = sum((y - y_mean) ** 2 for _, y in points)
    ss_res = sum((y - predict(x)) ** 2 for x, y in points)
    return 1.0 - ss_res / ss_tot


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    fit_points, excluded_points = parse_data(path)

    if len(fit_points) < 3:
        print("Not enough data points to fit.")
        return 1

    a, b = linear_fit(fit_points)
    lin_predict = lambda x: a * x + b
    r2_lin = r_squared(fit_points, lin_predict)

    p, q, r = quadratic_fit(fit_points)
    quad_predict = lambda x: p * x * x + q * x + r
    r2_quad = r_squared(fit_points, quad_predict)

    residuals = [(x, y, y - lin_predict(x)) for x, y in fit_points]
    max_abs_resid = max(abs(res) for _, _, res in residuals)
    rms_resid = math.sqrt(sum(res**2 for _, _, res in residuals) / len(residuals))
    max_pct = max(abs(res) / y * 100.0 for _, y, res in residuals if y != 0)

    print("Force coil current -> grams: linear fit")
    print("=" * 60)
    print(f"  points used         : {len(fit_points)}"
          f"  (excluded: {len(excluded_points)})")
    for x, y in excluded_points:
        print(f"    excluded: current<={x:g} A -> {y:g} g"
              " (deadband/threshold row, not a clean single-x point)")
    print()
    print(f"  grams = {a:.4f} * current + {b:.4f}")
    print(f"  R^2 (linear)        : {r2_lin:.6f}")
    print(f"  R^2 (quadratic)     : {r2_quad:.6f}"
          f"   (delta: {r2_quad - r2_lin:+.6f})")
    print(f"  RMS residual        : {rms_resid:.3f} g")
    print(f"  max |residual|      : {max_abs_resid:.3f} g"
          f"  ({max_pct:.1f}% of that point's reading)")
    print()
    print("  per-point residuals (measured - linear fit):")
    for x, y, res in residuals:
        bar_len = int(abs(res) / max_abs_resid * 20)
        bar = ("+" if res >= 0 else "-") * bar_len
        print(f"    {x:6.3f} A  {y:7.1f} g   resid {res:+7.2f} g  {bar}")

    print()
    print("  verdict:")
    if r2_lin > 0.999 and (r2_quad - r2_lin) < 0.001:
        print("    Linear model fits well; the quadratic term adds"
              " negligible explanatory power.")
        print("    -> current-to-force can be treated as a line over this"
              " range.")
    elif r2_lin > 0.995:
        print("    Linear fit is good (R^2 > 0.995) but the quadratic term"
              " measurably improves it")
        print(f"    (delta R^2 = {r2_quad - r2_lin:+.6f}) -- there's a mild,"
              " probably real curvature.")
        print("    A line is a reasonable working approximation; check the"
              " residual pattern above")
        print("    for whether it's systematic (curved) vs. scattered"
              " (noise).")
    else:
        print("    Linear fit is not a good description of this data"
              f" (R^2 = {r2_lin:.4f}).")
        print("    Consider a quadratic or piecewise model instead.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
