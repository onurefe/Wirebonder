"""Contact-sensor / limit-switch pin capture for the DebugIo channel."""

from ..service import Channel
from .base import BridgeCommand

CMD_LISTEN = 1
CMD_STOP = 2
LOG_DEPTH = 64

# Must match kBridgeIoPins in robot.cpp.
PIN_NAMES = ["TIP", "YLIM", "MLEFT", "MRIGHT"]

STATE_NAMES = {0: "INACTIVE", 1: "ACTIVE"}
EVENT_NAMES = {1: "activated", 2: "deactivated"}


class DebugIoPins(BridgeCommand):
    """debug-io: record TIP/YLIM/MLEFT/MRIGHT transitions until Ctrl-C."""

    NAME = "debug-io"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        self.svc.submit(Channel.IO, CMD_LISTEN)
        print("listening - trigger the pins, Ctrl-C when done...")
        self.t.cont()

        result_address = self.svc.result_pointer(0)
        self.svc.submit(Channel.IO, CMD_STOP)
        self.svc.pump()

        if result_address == 0:
            print("no io result published - check debug-status")
            return

        self._print_states(result_address)

        event_count = self.t.eval_int(result_expr(result_address) +
                                      ".eventCount")
        if event_count == 0:
            print("no state changes recorded - check the wiring/debug-status")
            return

        pins = self._read_log(result_address, "eventPin")
        events = self._read_log(result_address, "eventState")
        print_events(event_count, pins, events)

    def _print_states(self, result_address):
        result = result_expr(result_address)
        states_address = self.t.eval_int("(unsigned long)&%s.states[0]" %
                                         result)
        states = self.t.read_bytes(states_address, len(PIN_NAMES))

        print("states: " + "  ".join(
            "%s=%s" % (name, STATE_NAMES.get(states[index], "?"))
            for index, name in enumerate(PIN_NAMES)))

    def _read_log(self, result_address, field):
        result = result_expr(result_address)
        log_address = self.t.eval_int("(unsigned long)&%s.%s[0]" %
                                      (result, field))

        return self.t.read_bytes(log_address, LOG_DEPTH)


def result_expr(result_address):
    return "(*(IoDebugResult*)0x%08x)" % result_address


def pin_name_from_index(pin_index):
    if pin_index < len(PIN_NAMES):
        return PIN_NAMES[pin_index]

    return "?(%d)" % pin_index


def print_events(event_count, pins, events):
    shown_count = min(event_count, LOG_DEPTH)
    first_sequence_number = event_count - shown_count

    if event_count > LOG_DEPTH:
        print("%d state change(s) total, showing the last %d:" %
              (event_count, shown_count))
    else:
        print("%d state change(s):" % event_count)

    for sequence_number in range(first_sequence_number, event_count):
        log_index = sequence_number % LOG_DEPTH
        event = EVENT_NAMES.get(events[log_index],
                                "?(%d)" % events[log_index])

        print("  %3d. %-6s %s" %
              (sequence_number + 1,
               pin_name_from_index(pins[log_index]),
               event))
