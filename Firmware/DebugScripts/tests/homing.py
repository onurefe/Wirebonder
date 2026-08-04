"""Homing commands for the DebugHoming channel."""

import struct

from ..service import Channel, DONE
from .base import BridgeCommand

CMD_HOME = 1
CMD_CENTER = 2
CMD_STOP = 3

LOG_DEPTH = 32
ENTRY_SIZE = 8  # uint32 tickMs + uint32 event

RESULT_NAMES = {
    0: "home complete",
    1: "center complete",
    2: "stopped",
    3: "HOMING FAILED - see event log for the state it died in",
    201: "not homed - run debug-home first",
    202: "homing rejected - module not idle or router refused",
}

EVENT_NAMES = {
    0: "COMPLETED",
    1: "FAILED",
    2: "CLEARING_LIMIT (switch active at start; backing away)",
    3: "SEEKING_LIMIT (moving toward the switch)",
    4: "BACKING_OFF (switch found; leaving it)",
}


class HomingCommand(BridgeCommand):
    """Shared result/event-log reporting for the homing channel."""

    def run(self, local_command, what):
        if not self.svc.ensure_booted():
            return

        if not self.svc.run_request(Channel.HOMING, local_command, what):
            print("interrupted; status = %s" % self.svc.status_name())
            print("debug-homing-stop can stop the active operation")
            return

        code = self.svc.result_code()
        outcome = RESULT_NAMES.get(code, "resultCode = %d" % code)

        if self.svc.status() == DONE:
            print(outcome)
        else:
            print("failed: status = %s, %s" % (self.svc.status_name(),
                                               outcome))

        self.print_events()

    def print_events(self):
        result_address = self.svc.result_pointer(0)
        if result_address == 0:
            return

        limit_active = self.t.eval_int(homing_result_expr(result_address) +
                                       ".limitActive")
        print("limit switch now: %s" %
              ("ACTIVE" if limit_active else "inactive"))

        count = self.t.eval_int(homing_result_expr(result_address) + ".count")
        if count == 0:
            print("no homing events recorded")
            return

        entries_address = self.t.eval_int(
            "(unsigned long)&%s.entries[0]" % homing_result_expr(result_address))
        raw = self.t.read_bytes(entries_address, LOG_DEPTH * ENTRY_SIZE)

        print("homing events (%d total):" % count)
        first = max(0, count - LOG_DEPTH)
        for index in range(first, count):
            slot = index % LOG_DEPTH
            tick_ms, event = struct.unpack_from("<II", raw, slot * ENTRY_SIZE)
            name = EVENT_NAMES.get(event, "event %d" % event)
            print("  [%3d] t=%u ms  %s" % (index, tick_ms, name))


class DebugHome(HomingCommand):
    """debug-home: home the Y axis against its limit switch."""

    NAME = "debug-home"

    def invoke(self, arg, from_tty):
        self.run(CMD_HOME, "homing Y axis")


class DebugCenter(HomingCommand):
    """debug-center: move the homed Y axis to the workspace center."""

    NAME = "debug-center"

    def invoke(self, arg, from_tty):
        self.run(CMD_CENTER, "moving Y axis to workspace center")


class DebugHomingStop(HomingCommand):
    """debug-homing-stop: abort the active homing/centering operation."""

    NAME = "debug-homing-stop"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        if not self.svc.run_request(Channel.HOMING, CMD_STOP,
                                    "stopping homing"):
            print("interrupted; status = %s" % self.svc.status_name())
            return

        print("homing stopped; status = %s" % self.svc.status_name())
        self.print_events()


def homing_result_expr(result_address):
    return "(*(HomingDebugResult*)0x%08x)" % result_address
