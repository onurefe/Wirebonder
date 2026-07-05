"""Force-coil current step command for DebugForceCoil."""

import csv
import datetime
import os

from ..service import Channel, DONE
from .base import BridgeCommand

CMD_START = 1
# Capture cadence = force-coil control loop rate. Keep in sync with the
# firmware's FORCE_COIL_MODULE_CONTROL_FREQUENCY (configuration.h).
SAMPLE_RATE_HZ = 250.0
DEFAULT_DURATION_S = 2.0
CAPTURE_DIR = "captures"


class DebugForceCoil(BridgeCommand):
    """debug-force-coil <current_A | duty[0..1]> [duration_s] [open]

    Closed loop (default): arg 1 is a current setpoint in amps.
    Open loop ('open'): the PID is bypassed and arg 1 is a raw duty in
    [0, 1] applied directly to the coil PWM (unidirectional, 0 = off)."""

    NAME = "debug-force-coil"

    def invoke(self, arg, from_tty):
        current_setpoint, duration, bypass_pid = parse_args(arg)

        if current_setpoint is None:
            print("usage: debug-force-coil <current_A | duty[0..1]> "
                  "[duration_s] [open]")
            return

        if not self.svc.ensure_booted():
            return

        if not self.svc.run_request(Channel.FORCE_COIL, CMD_START,
                                    "capturing force-coil current",
                                    [current_setpoint, duration,
                                     1.0 if bypass_pid else 0.0]):
            print("interrupted; status = %s" % self.svc.status_name())
            return

        if self.svc.status() != DONE:
            print("force-coil capture failed: status = %s" %
                  self.svc.status_name())
            return

        self._report(current_setpoint, bypass_pid)

    def _report(self, current_setpoint, bypass_pid):
        address = self.svc.result_pointer(0)
        if address == 0:
            print("no force-coil telemetry published - check debug-status")
            return

        count = self.svc.result_count()
        response = self.t.read_floats(address, count)
        dt = 1.0 / SAMPLE_RATE_HZ

        rows = [[i, i * dt, current] for i, current in enumerate(response)]
        path = write_capture("force_coil_current",
                             ["tick", "time_s", "current_a"],
                             rows)

        if bypass_pid:
            print("%d current samples -> %s (open-loop duty %.4f)" %
                  (count, path, current_setpoint))
            print_metrics(step_response_metrics(response, dt, None))
        else:
            print("%d current samples -> %s" % (count, path))
            print_metrics(step_response_metrics(response, dt, current_setpoint))


def parse_args(arg):
    args = arg.split()

    if not args:
        return None, None, None

    current_setpoint = float(args[0])
    duration = float(args[1]) if len(args) > 1 else DEFAULT_DURATION_S
    bypass_pid = len(args) > 2 and args[2].lower() in ("open", "bypass", "1")

    return current_setpoint, duration, bypass_pid


def write_capture(prefix, header, rows):
    os.makedirs(CAPTURE_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(CAPTURE_DIR, "%s_%s.csv" % (prefix, stamp))

    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)

    return path


def mean(values):
    return sum(values) / len(values) if values else 0.0


def step_response_metrics(response, dt, target=None):
    count = len(response)

    if count < 4:
        return None

    initial = response[0]
    steady = mean(response[-max(1, count // 10):])
    delta = steady - initial
    metrics = {
        "v0": initial,
        "steady": steady,
        "peak": max(response) if delta >= 0 else min(response),
        "rise_time_s": None,
        "overshoot_pct": None,
        "settling_time_s": None,
    }

    if target is not None:
        metrics["steady_error"] = steady - target

    if abs(delta) < 1e-9:
        return metrics

    normalized = [(value - initial) / delta for value in response]
    metrics["rise_time_s"] = rise_time(normalized, dt)
    metrics["overshoot_pct"] = max(0.0, (max(normalized) - 1.0) * 100.0)
    metrics["settling_time_s"] = settling_time(normalized, dt)

    return metrics


def rise_time(normalized_response, dt):
    ten_percent = first_index_at_or_above(normalized_response, 0.1)
    ninety_percent = first_index_at_or_above(normalized_response, 0.9)

    if ten_percent is None or ninety_percent is None:
        return None
    if ninety_percent < ten_percent:
        return None

    return (ninety_percent - ten_percent) * dt


def first_index_at_or_above(values, threshold):
    for index, value in enumerate(values):
        if value >= threshold:
            return index

    return None


def settling_time(normalized_response, dt):
    last_outside = -1

    for index, value in enumerate(normalized_response):
        if abs(value - 1.0) > 0.02:
            last_outside = index

    if last_outside >= len(normalized_response) - 1:
        return None

    return (last_outside + 1) * dt


def print_metrics(metrics):
    if metrics is None:
        print("capture too short for step-response metrics")
        return

    print("  initial current : %.4f A" % metrics["v0"])
    print("  steady current  : %.4f A" % metrics["steady"])
    print("  peak current    : %.4f A" % metrics["peak"])

    if metrics["rise_time_s"] is not None:
        print("  rise time       : %.4f s" % metrics["rise_time_s"])
    if metrics["settling_time_s"] is not None:
        print("  settling time   : %.4f s" % metrics["settling_time_s"])
    if metrics["overshoot_pct"] is not None:
        print("  overshoot       : %.2f %%" % metrics["overshoot_pct"])
    if metrics.get("steady_error") is not None:
        print("  steady error    : %.4f A" % metrics["steady_error"])
