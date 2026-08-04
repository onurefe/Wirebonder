"""Tone commands for the DebugToneGenerator channel."""

import math
import struct

from ..service import Channel
from .base import BridgeCommand

CMD_START = 1
CMD_STOP = 2

RESULT_FORMAT = "<fIfIffff"
RESULT_SIZE = struct.calcsize(RESULT_FORMAT)


class DebugTone(BridgeCommand):
    """debug-tone <amplitude> [frequency_hz]."""

    NAME = "debug-tone"

    def invoke(self, arg, from_tty):
        args = arg.split()
        if not args:
            print("usage: debug-tone <amplitude> [frequency_hz]")
            return

        if not self.svc.ensure_booted():
            return

        amplitude = float(args[0])
        frequency = float(args[1]) if len(args) > 1 else 0.0

        self.svc.submit(Channel.TONE, CMD_START, [frequency, amplitude])

        print("tone: amplitude %s at %s Hz - press Ctrl-C when done..." %
              (args[0], args[1] if len(args) > 1 else "60000 (default)"))
        self.t.cont()

        self._report()
        print("tone still playing - debug-tone-stop to silence, "
              "debug-tone to change parameters")

    def _report(self):
        address = self.svc.result_pointer(0)
        if address == 0:
            print("no tone result published - check debug-status")
            return

        result = struct.unpack(RESULT_FORMAT,
                               self.t.read_bytes(address, RESULT_SIZE))
        vmag, vupd, imag, iupd, vre, vim, ire, iim = result

        if vupd == 0 and iupd == 0:
            print("no measurements arrived - check debug-status")
            return

        print("measured V-sense amplitude: %.4f V (%d measurements)" %
              (vmag, vupd))
        print("measured I-sense amplitude: %.4f A (%d measurements)" %
              (imag, iupd))

        if imag > 1e-6:
            print("implied |Z| = V/I         : %.1f ohm" % (vmag / imag))
            phase = math.degrees(math.atan2(vim * ire - vre * iim,
                                            vre * ire + vim * iim))
            print("phase arg(V*conj(I))      : %+.2f deg" % phase)


class DebugToneStop(BridgeCommand):
    """debug-tone-stop: stop the tone output."""

    NAME = "debug-tone-stop"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        self.svc.submit(Channel.TONE, CMD_STOP)
        self.svc.pump()
        print("tone stopped; status = %s" % self.svc.status_name())
