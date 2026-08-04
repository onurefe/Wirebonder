"""Keypad capture command for the DebugKeypad channel."""

import struct

from ..service import Channel
from .base import BridgeCommand

CMD_LISTEN = 1
CMD_STOP = 2
LOG_DEPTH = 64

BUTTON_NAMES = [
    "UP", "DOWN", "LEFT", "RIGHT",
    "PLUS", "MINUS", "SAVE", "LOAD",
    "TAIL+", "TAIL-",
    "LOOP+", "LOOP-",
    "SEARCH+", "SEARCH-",
    "STEP+", "STEP-",
    "RESET", "ENTER",
    "MANUAL", "ESC/DEL", "ADD",
    "TEST", "SETUP", "LIGHT",
    "CLAMP_OPEN", "HIGH_RESET",
]


class DebugKeys(BridgeCommand):
    """debug-keys: record keypad button presses until Ctrl-C."""

    NAME = "debug-keys"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        self.svc.submit(Channel.KEYPAD, CMD_LISTEN)
        print("listening - press keypad buttons, Ctrl-C when done...")
        self.t.cont()

        result_address = self.svc.result_pointer(0)
        self.svc.submit(Channel.KEYPAD, CMD_STOP)
        self.svc.pump()

        if result_address == 0:
            print("no keypad result published - check debug-status")
            return

        press_count = self._read_press_count(result_address)
        if press_count > 0:
            print_key_sequence(press_count, self._read_key_log(result_address))
        else:
            print("no named presses recorded")

        # Raw pin-state transitions map physical pins to buttons and surface
        # activity even when no button mask matches (i.e. keyCount stays 0).
        state_count = self._read_state_count(result_address)
        if state_count > 0:
            print_state_transitions(state_count,
                                    self._read_states(result_address))
        elif press_count == 0:
            print("no keypad activity - check the keypad wiring/debug-status")

    def _read_press_count(self, result_address):
        return self.t.eval_int(keypad_result_expr(result_address) +
                               ".keyCount")

    def _read_key_log(self, result_address):
        result = keypad_result_expr(result_address)
        log_address = self.t.eval_int("(unsigned long)&%s.keyLog[0]" %
                                      result)

        return self.t.read_bytes(log_address, LOG_DEPTH)

    def _read_state_count(self, result_address):
        return self.t.eval_int(keypad_result_expr(result_address) +
                               ".stateCount")

    def _read_states(self, result_address):
        result = keypad_result_expr(result_address)
        states_address = self.t.eval_int("(unsigned long)&%s.states[0]" %
                                         result)
        raw = self.t.read_bytes(states_address, LOG_DEPTH * 2)

        return struct.unpack("<%dH" % LOG_DEPTH, raw)


def keypad_result_expr(result_address):
    return "(*(KeypadDebugResult*)0x%08x)" % result_address


def print_key_sequence(press_count, key_log):
    shown_count = min(press_count, LOG_DEPTH)
    first_sequence_number = press_count - shown_count

    if press_count > LOG_DEPTH:
        print("%d presses total, showing the last %d:" %
              (press_count, shown_count))
    else:
        print("%d press(es):" % press_count)

    for sequence_number in range(first_sequence_number, press_count):
        log_index = sequence_number % LOG_DEPTH
        button_index = key_log[log_index]

        print("  %3d. %s" %
              (sequence_number + 1, button_name_from_index(button_index)))


def button_name_from_index(button_index):
    if button_index < len(BUTTON_NAMES):
        return BUTTON_NAMES[button_index]

    return "?(%d)" % button_index


def print_state_transitions(state_count, states):
    """Print the recorded raw keypad-expander states as per-pin rise/fall
    edges (1-based pin numbers). Baseline is all-released, so the first entry
    shows the initially-held pins as rises."""
    shown_count = min(state_count, LOG_DEPTH)
    if shown_count == 0:
        return

    first_sequence_number = state_count - shown_count

    print("raw pin-state transitions (physical pin -> button mapping):")
    previous_state = 0
    for sequence_number in range(first_sequence_number, state_count):
        state = states[sequence_number % LOG_DEPTH]
        rose = state & ~previous_state
        fell = previous_state & ~state
        previous_state = state

        edges = ["+%d" % (bit + 1) for bit in range(16) if rose & (1 << bit)]
        edges += ["-%d" % (bit + 1) for bit in range(16) if fell & (1 << bit)]
        if not edges:
            continue

        print("  %3d. state=0x%04x  pins %s" %
              (sequence_number + 1, state, ",".join(edges)))
