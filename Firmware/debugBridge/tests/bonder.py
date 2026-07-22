"""Bonder VM observation and virtual operator input.

Workflow: the protocol VM free-runs on its own internal dynamics (moves,
scans, timers, contact sensing); those events are never injected from here.
Only operator inputs (the mouse buttons) can be supplied virtually, through
the same relay path the physical pins use.

    (gdb) debug-bonder-start semi-auto
    (gdb) debug-bonder                    # free-run; Ctrl-C to inspect
    (gdb) debug-bonder-status             # where is the VM blocked?
    (gdb) debug-bonder-button right click # virtual operator trigger
    (gdb) debug-bonder                    # keep watching
    (gdb) debug-bonder-stop
"""

import csv
import datetime
import math
import os

from ..service import Channel, DONE
from .base import BridgeCommand

CMD_START_BONDER = 1
CMD_STOP_BONDER = 2
CMD_SET_BUTTON = 3

BUTTON_IDS = {"right": 0, "left": 1}

LOG_SYMBOL = "BonderDebugEnvironment::s_stepLog"
STATUS_SYMBOL = "BonderDebugEnvironment::s_status"

LOG_DEPTH = 64
CAPTURE_DIR = "captures"

# Must match BonderModule::Opcode.
OPCODE_NAMES = {
    0: "ZMOVE",
    1: "MZMOVE",
    2: "YMOVE",
    3: "TMOVE",
    4: "TIMER",
    5: "WAIT",
    6: "CLRFLAGS",
    7: "CLAMPOPEN",
    8: "CLAMPCLOSE",
    9: "SCAN",
    10: "PLL",
    11: "SETFORCE",
    12: "USREPORT",
    13: "MZDOWN",
}

# Must match BonderModule::EventFlag bit positions.
EVENT_NAMES = {
    0: "t-move-done",
    1: "y-move-done",
    2: "force-settled",
    3: "contact-connected",
    4: "contact-disconnected",
    5: "timer-expired",
    6: "scan-done",
    7: "us-done",
    8: "right-button-pressed",
    9: "right-button-released",
    10: "position-error",
    11: "force-coil-error",
    12: "us-power-error",
    13: "z-position-reached",
    14: "clamp-settled",
    15: "wait-timeout",
    16: "left-button-pressed",
}

BONDING_MODES = {
    "semi-auto": 0.0,
    "semi-automatic": 0.0,
    "manual": 1.0,
    "table-tear": 2.0,
    "lange-coupling": 3.0,
}

CLAMP_STATES = {
    0: "ENERGIZING",
    1: "DEENERGIZING",
    2: "ENERGIZED",
    3: "DEENERGIZED",
}

# Log position already shown to the user; shared by the watch/start commands
# so successive debug-bonder invocations only print new transitions.
_cursor = 0


def event_name(bit):
    return EVENT_NAMES.get(bit, "?(bit %d)" % bit)


def event_names(mask):
    names = [event_name(bit) for bit in sorted(EVENT_NAMES)
             if mask & (1 << bit)]
    extra = mask & ~sum(1 << bit for bit in EVENT_NAMES)
    if extra:
        names.append("?(0x%x)" % extra)
    return names


def print_status(target):
    """One-line-per-fact live view of the VM (target must be halted)."""

    running = target.eval_int(STATUS_SYMBOL + ".bonderRunning") != 0
    pc = target.eval_int(STATUS_SYMBOL + ".pc")
    opcode = target.eval_int(STATUS_SYMBOL + ".opcode")
    mask = target.eval_int(STATUS_SYMBOL + ".waitMask")
    flags = target.eval_int(STATUS_SYMBOL + ".eventFlags")
    clamp = target.eval_int(STATUS_SYMBOL + ".clampState")
    z = target.eval_float(STATUS_SYMBOL + ".zPosition")
    y = target.eval_float(STATUS_SYMBOL + ".yPosition")
    t = target.eval_float(STATUS_SYMBOL + ".tPosition")

    if not running:
        print("bonder: IDLE (protocol finished, aborted, or not started)")
    else:
        name = OPCODE_NAMES.get(opcode, "OP(%d)" % opcode)
        print("bonder: RUNNING at pc=%d %s" % (pc, name))
        if mask:
            pending = mask & ~flags
            print("  waiting on: %s" % ", ".join(event_names(mask)))
            if pending:
                print("  still missing: %s" % ", ".join(event_names(pending)))
            else:
                print("  mask satisfied - completing on next tick")

    latched = event_names(flags)
    print("  latched events: %s" % (", ".join(latched) if latched else "none"))

    for side in ("right", "left"):
        physical = target.eval_int(
            STATUS_SYMBOL + ".%sButtonPhysical" % side) != 0
        virtual = target.eval_int(
            STATUS_SYMBOL + ".%sButtonVirtual" % side) != 0
        state = "PRESSED" if (physical or virtual) else "released"
        source = []
        if physical:
            source.append("physical")
        if virtual:
            source.append("virtual")
        print("  %s button: %s%s" %
              (side, state, " (%s)" % "+".join(source) if source else ""))

    print("  clamp=%s z=%.4f mm y=%.4f mm t=%.4f mm" %
          (CLAMP_STATES.get(clamp, "?(%d)" % clamp), z, y, t))


class DebugBonderWatch(BridgeCommand):
    """debug-bonder: free-run and report state transitions on Ctrl-C."""

    NAME = "debug-bonder"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        print("bonder free-running - press the physical buttons or use "
              "debug-bonder-button; Ctrl-C to inspect...")
        self.t.cont()

        self._print_new_entries()
        print_status(self.t)

    def _entry_expr(self, slot):
        return LOG_SYMBOL + ".entries[%d]" % slot

    def _int(self, entry, field):
        return self.t.eval_int(entry + "." + field)

    def _float(self, entry, field):
        return self.t.eval_float(entry + "." + field)

    def _print_new_entries(self):
        global _cursor

        count = self.t.eval_int(LOG_SYMBOL + ".count")

        if count < _cursor:
            # The log restarted (new bonding run or firmware reset).
            _cursor = 0

        if count - _cursor > LOG_DEPTH:
            print("(%d transitions overwritten - showing the last %d)" %
                  (count - _cursor - LOG_DEPTH, LOG_DEPTH))
            _cursor = count - LOG_DEPTH

        if count == _cursor:
            print("no new state transitions")
            return

        print("state transitions:")
        for sequence in range(_cursor, count):
            entry = self._entry_expr(sequence % LOG_DEPTH)
            self._print_entry(sequence, entry)

        _cursor = count

    def _print_entry(self, sequence, entry):
        pc = self._int(entry, "pc")
        opcode = self._int(entry, "opcode")
        succeeded = self._int(entry, "succeeded") != 0
        mask = self._int(entry, "mask")
        flags = self._int(entry, "eventFlags")

        print("  %3d. pc=%-3d %-10s [%s]" %
              (sequence + 1, pc,
               OPCODE_NAMES.get(opcode, "OP(%d)" % opcode),
               "OK" if succeeded else "ERROR"))
        self._print_payload(entry, sequence, opcode, mask, flags)

    def _print_payload(self, entry, sequence, opcode, mask, flags):
        arg_value = self._float(entry, "argValue")
        z = self._float(entry, "zPosition")
        target = self._float(entry, "zSetpoint")
        y = self._float(entry, "yPosition")
        t = self._float(entry, "tPosition")
        clamp = self._int(entry, "clampState")
        energy = self._float(entry, "transferredEnergy")

        if opcode in (0, 1, 13):
            print("       arg=%.4f z=%.4f/%.4f mm" %
                  (arg_value, z, target))
        elif opcode == 2:
            print("       target=%.4f mm y=%.4f mm" % (arg_value, y))
        elif opcode == 3:
            print("       target=%.4f mm t=%.4f mm" % (arg_value, t))
        elif opcode == 4:
            print("       duration=%.4f s" % arg_value)
        elif opcode in (5, 6):
            requested = event_names(mask)
            print("       mask: %s" %
                  (", ".join(requested) if requested else "none"))
        elif opcode in (7, 8):
            print("       clamp=%s" %
                  CLAMP_STATES.get(clamp, "?(%d)" % clamp))
        elif opcode == 9:
            print("       target power=%.4f W" % arg_value)
        elif opcode == 10:
            print("       requested=%.6f J transferred=%.6f J" %
                  (arg_value, energy))
        elif opcode == 11:
            print("       force current=%.4f A" % arg_value)

        events = event_names(flags)
        if events:
            print("       events: %s" % ", ".join(events))

        count = self._int(entry, "scanCount")
        if count:
            self._print_impedance_scan(entry, sequence, count)

    def _print_impedance_scan(self, entry, sequence, count):
        center = self._float(entry, "centerFrequency")
        amplitude = self._float(entry, "driveAmplitude")
        address = self._int(entry, "impedances")
        print("       center=%.2f Hz drive=%.5f V peak" %
              (center, amplitude))
        if address == 0 or count == 0:
            print("       impedance curve unavailable")
            return

        flat = self.t.read_floats(address, count * 2)
        os.makedirs(CAPTURE_DIR, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(
            CAPTURE_DIR,
            "bonder_scan_%03d_%s.csv" % (sequence + 1, stamp))
        with open(path, "w", newline="") as handle:
            writer = csv.writer(handle)
            # The VM telemetry publishes the operating-point frequency but not
            # the per-bin sweep frequencies. Preserve the curve without
            # inventing a frequency axis.
            writer.writerow(["sample", "z_re", "z_im",
                             "magnitude", "phase_deg"])
            for index in range(count):
                real = flat[2 * index]
                imag = flat[2 * index + 1]
                writer.writerow([
                    index,
                    real,
                    imag,
                    math.hypot(real, imag),
                    math.degrees(math.atan2(imag, real)),
                ])
        print("       impedance=%d points -> %s" % (count, path))


class DebugBonderStatus(BridgeCommand):
    """debug-bonder-status: show where the VM is and what it waits on."""

    NAME = "debug-bonder-status"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        print_status(self.t)


class DebugBonderStart(BridgeCommand):
    """debug-bonder-start [semi-auto|manual|table-tear|lange-coupling]."""

    NAME = "debug-bonder-start"

    def invoke(self, arg, from_tty):
        global _cursor

        requested_mode = arg.strip().lower() or "semi-auto"
        mode = BONDING_MODES.get(requested_mode)
        if mode is None:
            print("usage: debug-bonder-start "
                  "[semi-auto|manual|table-tear|lange-coupling]")
            return

        if not self.svc.ensure_booted():
            return

        if not self.svc.run_request(Channel.BONDER, CMD_START_BONDER,
                                    "starting %s bonder" % requested_mode,
                                    [mode]):
            print("interrupted; status = %s" % self.svc.status_name())
            return

        if self.svc.status() != DONE:
            print("bonder start failed: status = %s, resultCode = %d" %
                  (self.svc.status_name(), self.svc.result_code()))
            return

        _cursor = 0
        print("%s bonder started - use debug-bonder to watch it run" %
              requested_mode)


class DebugBonderStop(BridgeCommand):
    """debug-bonder-stop: abort the bonder state machine."""

    NAME = "debug-bonder-stop"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        if not self.svc.run_request(Channel.BONDER, CMD_STOP_BONDER,
                                    "stopping bonder"):
            print("interrupted; status = %s" % self.svc.status_name())
            return

        print("bonder stopped; status = %s" % self.svc.status_name())


class DebugBonderButton(BridgeCommand):
    """debug-bonder-button <right|left> <press|release|click>."""

    NAME = "debug-bonder-button"

    def invoke(self, arg, from_tty):
        words = arg.lower().split()
        if len(words) != 2 or words[0] not in BUTTON_IDS or \
                words[1] not in ("press", "release", "click"):
            print("usage: debug-bonder-button <right|left> "
                  "<press|release|click>")
            return

        button, action = words

        if not self.svc.ensure_booted():
            return

        if action == "click":
            steps = [("press", 1.0), ("release", 0.0)]
        else:
            steps = [(action, 1.0 if action == "press" else 0.0)]

        for name, pressed in steps:
            if not self.svc.run_request(
                    Channel.BONDER, CMD_SET_BUTTON,
                    "%s button %s" % (button, name),
                    [BUTTON_IDS[button], pressed]):
                print("interrupted; status = %s" % self.svc.status_name())
                return

            if self.svc.status() != DONE:
                print("button input failed: status = %s, resultCode = %d" %
                      (self.svc.status_name(), self.svc.result_code()))
                return

        print("%s button %s (virtual input merged with the physical pin)" %
              (button, action))
