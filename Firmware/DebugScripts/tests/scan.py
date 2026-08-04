"""Impedance scan command for the DebugImpedanceScanner channel."""

import math

from ..service import Channel, DONE
from .base import BridgeCommand

CMD_SCAN = 1
IMPEDANCE_RESULT_POINTER_INDEX = 2


class DebugScan(BridgeCommand):
    """debug-scan [numFrequencies [minFrequency [frequencyStep]]]."""

    NAME = "debug-scan"

    def invoke(self, arg, from_tty):
        count, fmin, step = parse_args(arg)

        if not self.svc.ensure_booted():
            return

        if not self._run_scan(count, fmin, step):
            return

        count = self.svc.result_count()
        fmin = self.svc.arg(1)
        step = self.svc.arg(2)
        impedances = self._read_impedances(count)

        print_impedances(fmin, step, impedances)

    def _run_scan(self, count, fmin, step):
        success = self.svc.run_request(Channel.IMPEDANCE, CMD_SCAN,
                                       "scanning", [count, fmin, step])

        if not success:
            print("interrupted; status = %s" % self.svc.status_name())
            return False

        if self.svc.status() != DONE:
            print("scan failed: status = %s" % self.svc.status_name())
            return False

        return True

    def _read_impedances(self, count):
        address = self.svc.result_pointer(IMPEDANCE_RESULT_POINTER_INDEX)
        flat = self.t.read_floats(address, count * 2)

        return [(flat[2 * i], flat[2 * i + 1]) for i in range(count)]


def parse_args(arg):
    args = arg.split()

    count = int(args[0]) if len(args) > 0 else 0
    fmin = float(args[1]) if len(args) > 1 else 0.0
    step = float(args[2]) if len(args) > 2 else 0.0

    return count, fmin, step


def print_impedances(fmin, step, impedances):
    print("  %-12s %-12s %-12s %-12s %s" %
          ("freq_hz", "z_re", "z_im", "|z|", "phase_deg"))

    for index, (real, imag) in enumerate(impedances):
        freq = fmin + index * step
        mag = math.hypot(real, imag)
        phase = math.degrees(math.atan2(imag, real))

        print("  %-12.1f %-12.3f %-12.3f %-12.3f %+.2f" %
              (freq, real, imag, mag, phase))
