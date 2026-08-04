"""Motor position step command for DebugMotorPositionController."""

import struct

from ..service import Channel, DONE
from .base import BridgeCommand
from .motor_velocity import step_response_metrics, write_capture

CMD_START = 1
CMD_STALL_SCAN = 3
# Keep in sync with DCMOTOR_POSITION_MODULE_CONTROL_FREQUENCY.
SAMPLE_RATE_HZ = 1000.0
DEFAULT_DURATION_S = 4.0
SAMPLE_FORMAT = "<fff"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FORMAT)
STALL_SAMPLE_FORMAT = "<ffff"
STALL_SAMPLE_SIZE = struct.calcsize(STALL_SAMPLE_FORMAT)


class DebugMotorPosition(BridgeCommand):
    """debug-motor-position <position | velocity> [duration_s] [open].

    Closed loop (default): arg 1 is a position setpoint.
    Open loop ('open'): bypasses the position controller; arg 1 is passed
    directly as the inner velocity-loop setpoint while position is recorded."""

    NAME = "debug-motor-position"

    def invoke(self, arg, from_tty):
        target_position, duration, bypass_controller = parse_args(arg)

        if target_position is None:
            print("usage: debug-motor-position <position | velocity> "
                  "[duration_s] [open]")
            return

        if not self.svc.ensure_booted():
            return

        if not self.svc.run_request(Channel.MOTOR_POSITION, CMD_START,
                                    "capturing motor position",
                                    [target_position, duration,
                                     1.0 if bypass_controller else 0.0]):
            print("interrupted; status = %s" % self.svc.status_name())
            return

        if self.svc.status() != DONE:
            print("motor position capture failed: status = %s" %
                  self.svc.status_name())
            return

        self._report(target_position, bypass_controller)

    def _report(self, target_position, bypass_controller):
        address = self.svc.result_pointer(0)
        if address == 0:
            print("no motor position telemetry published - check debug-status")
            return

        count = self.svc.result_count()
        raw = self.t.read_bytes(address, count * SAMPLE_SIZE)
        samples = [
            struct.unpack_from(SAMPLE_FORMAT, raw, i * SAMPLE_SIZE)
            for i in range(count)
        ]
        positions = [position for position, _mag_a, _mag_b in samples]
        dt = 1.0 / SAMPLE_RATE_HZ

        rows = [[i, i * dt, position, mag_a, mag_b]
                for i, (position, mag_a, mag_b) in enumerate(samples)]
        path = write_capture("motor_position",
                             ["tick", "time_s", "position", "magA", "magB"],
                             rows)

        if bypass_controller:
            print("%d position samples -> %s (open-loop velocity %.4f)" %
                  (count, path, target_position))
            print_position_metrics(step_response_metrics(positions, dt, None))
        else:
            print("%d position samples -> %s" % (count, path))
            print_position_metrics(
                step_response_metrics(positions, dt, target_position))


def parse_args(arg):
    args = arg.split()

    if not args:
        return None, None, None

    target_position = float(args[0])
    duration = float(args[1]) if len(args) > 1 else DEFAULT_DURATION_S
    bypass_controller = (
        len(args) > 2 and args[2].lower() in ("open", "bypass", "1")
    )

    return target_position, duration, bypass_controller


def print_position_metrics(metrics):
    if metrics is None:
        print("capture too short for position step-response metrics")
        return

    print("  initial position : %.4f" % metrics["v0"])
    print("  steady position  : %.4f" % metrics["steady"])
    print("  peak position    : %.4f" % metrics["peak"])
    print("  displacement     : %.4f" %
          (metrics["steady"] - metrics["v0"]))

    if metrics["rise_time_s"] is not None:
        print("  rise time        : %.4f s" % metrics["rise_time_s"])
    if metrics["settling_time_s"] is not None:
        print("  settling time    : %.4f s" % metrics["settling_time_s"])
    if metrics["overshoot_pct"] is not None:
        print("  overshoot        : %.2f %%" % metrics["overshoot_pct"])
    if metrics.get("steady_error") is not None:
        print("  steady pos error : %.4f" % metrics["steady_error"])


class DebugMotorPositionStall(BridgeCommand):
    """debug-motor-position-stall <start_drive> <end_drive> [step] [settle_s] [relax_s].

    Applies raw signed motor drive values through the position-controller debug
    path, waits settle_s at each drive, records the stalled LVDT position, then
    relaxes with firmware-configured reverse drive before the next step. drive
    maps to bridge duty as duty = 0.5 + drive."""

    NAME = "debug-motor-position-stall"

    def invoke(self, arg, from_tty):
        try:
            start_drive, end_drive, drive_step, settle_s, relax_s = (
                parse_stall_args(arg)
            )
        except ValueError as exc:
            print("invalid stall scan arguments: %s" % exc)
            return

        if start_drive is None:
            print("usage: debug-motor-position-stall <start_drive> <end_drive> "
                  "[step] [settle_s] [relax_s]")
            return

        if not self.svc.ensure_booted():
            return

        if not self.svc.run_request(Channel.MOTOR_POSITION, CMD_STALL_SCAN,
                                    "capturing motor stall positions",
                                    [start_drive, end_drive, drive_step,
                                     settle_s, relax_s]):
            print("interrupted; status = %s" % self.svc.status_name())
            return

        if self.svc.status() != DONE:
            print("motor position stall scan failed: status = %s" %
                  self.svc.status_name())
            return

        self._report()

    def _report(self):
        address = self.svc.result_pointer(0)
        if address == 0:
            print("no motor stall telemetry published - check debug-status")
            return

        count = self.svc.result_count()
        raw = self.t.read_bytes(address, count * STALL_SAMPLE_SIZE)
        samples = [
            struct.unpack_from(STALL_SAMPLE_FORMAT, raw, i * STALL_SAMPLE_SIZE)
            for i in range(count)
        ]

        rows = [
            [i, drive, 0.5 + drive, position, mag_a, mag_b]
            for i, (drive, position, mag_a, mag_b) in enumerate(samples)
        ]
        path = write_capture(
            "motor_position_stall",
            ["index", "drive", "duty", "position", "magA", "magB"],
            rows)

        print("%d stall samples -> %s" % (count, path))
        if samples:
            print("  start drive/pos : %.4f / %.4f" %
                  (samples[0][0], samples[0][1]))
            print("  end drive/pos   : %.4f / %.4f" %
                  (samples[-1][0], samples[-1][1]))


def parse_stall_args(arg):
    args = arg.split()

    if not args:
        return None, None, None, None, None

    if len(args) < 2:
        raise ValueError("start_drive and end_drive are required")

    start_drive = float(args[0])
    end_drive = float(args[1])
    settle_s = float(args[3]) if len(args) > 3 else 0.5
    relax_s = float(args[4]) if len(args) > 4 else 0.25

    if len(args) > 2:
        drive_step = float(args[2])
    else:
        span = end_drive - start_drive
        drive_step = span / 20.0 if abs(span) > 1e-9 else 0.0

    if drive_step == 0.0:
        raise ValueError("drive step must be nonzero")

    if settle_s <= 0.0:
        raise ValueError("settle_s must be positive")

    if relax_s < 0.0:
        raise ValueError("relax_s must be nonnegative")

    if end_drive > start_drive and drive_step < 0.0:
        drive_step = -drive_step
    elif end_drive < start_drive and drive_step > 0.0:
        drive_step = -drive_step

    return start_drive, end_drive, drive_step, settle_s, relax_s
