"""PLL commands for the DebugPll channel."""

import csv
import datetime
import os

from ..service import Channel, DONE
from .base import BridgeCommand

CMD_START = 1
CMD_STOP = 2
SAMPLE_FLOATS = 4
CAPTURE_DIR = "captures"


def mean(values):
    return sum(values) / len(values) if values else 0.0


def stddev(values):
    if not values:
        return 0.0

    average = mean(values)
    variance = sum((value - average) ** 2 for value in values) / len(values)

    return variance ** 0.5


def write_capture(prefix, header, rows):
    os.makedirs(CAPTURE_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(CAPTURE_DIR, "%s_%s.csv" % (prefix, stamp))

    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)

    return path


class DebugPll(BridgeCommand):
    """debug-pll <amplitude> [maxDuration_s [centerFrequency_hz [energy_J]]]."""

    NAME = "debug-pll"

    def invoke(self, arg, from_tty):
        args = arg.split()
        if not args:
            print("usage: debug-pll <amplitude> [duration [center [energy]]]")
            return

        if not self.svc.ensure_booted():
            return

        amplitude = float(args[0])
        duration = float(args[1]) if len(args) > 1 else 0.0
        center = float(args[2]) if len(args) > 2 else 0.0
        energy = float(args[3]) if len(args) > 3 else 0.0

        if not self.svc.run_request(Channel.PLL, CMD_START,
                                    "pll running",
                                    [center, amplitude, energy, duration]):
            print("interrupted; status = %s - if the PLL is still driving, "
                  "use debug-stop" % self.svc.status_name())
            return

        if self.svc.status() != DONE:
            print("pll run failed: status = %s" % self.svc.status_name())
            return

        self._report()

    def _report(self):
        outcome = ("energy target reached" if self.svc.result_code() == 0
                   else "max duration timeout")
        count = self.svc.result_count()
        flat = self.t.read_floats(self.svc.result_pointer(0),
                                  count * SAMPLE_FLOATS)
        samples = [tuple(flat[SAMPLE_FLOATS * i:SAMPLE_FLOATS * (i + 1)])
                   for i in range(count)]
        rows = [[i] + list(sample) for i, sample in enumerate(samples)]
        path = write_capture("pll",
                             ["tick", "phase_error_rad",
                              "freq_correction_hz", "real_power",
                              "bonding_energy_j"],
                             rows)

        print("outcome: %s, %d telemetry samples -> %s" %
              (outcome, count, path))
        summarise(samples)


class DebugStop(BridgeCommand):
    """debug-stop: stop a running PLL."""

    NAME = "debug-stop"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        self.svc.submit(Channel.PLL, CMD_STOP)
        self.svc.pump()
        print("stopped; status = %s" % self.svc.status_name())


def summarise(samples):
    if len(samples) < 100:
        return

    tail = samples[-100:]
    phase_errors = [sample[0] for sample in tail]
    corrections = [sample[1] for sample in tail]
    phase_std = stddev(phase_errors)

    print("  final phase error : %+.4f rad, jitter %.4f rad RMS (last 100)" %
          (mean(phase_errors), phase_std))
    print("  final correction  : %+.1f Hz, jitter %.1f Hz RMS" %
          (mean(corrections), stddev(corrections)))
    print("  final real power  : %.4f" %
          mean([sample[2] for sample in tail]))
    print("  delivered energy  : %.4f J" % samples[-1][3])

    if phase_std > 0.3:
        print("  WARNING: phase jitter says this is an oscillation, not a lock")
