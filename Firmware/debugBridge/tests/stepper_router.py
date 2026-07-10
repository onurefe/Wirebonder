"""Stepper router visual-inspection commands."""

from ..service import Channel, DONE
from .base import BridgeCommand

CMD_MOVE = 1
CMD_STOP = 2

AXES = {
    "y": 0.0,
    "t": 1.0,
}

AXIS_NAMES = {
    0: "Y",
    1: "T",
}


class DebugStepperRouter(BridgeCommand):
    """debug-stepper-router <y|t> <position_mm>.

    Moves one routed stepper axis to a logical position. The position origin is
    zero when the router service starts. There is no sensor feedback; verify
    motion by visual inspection."""

    NAME = "debug-stepper-router"

    def invoke(self, arg, from_tty):
        axis, position = parse_move_args(arg)

        if axis is None:
            print("usage: debug-stepper-router <y|t> <position_mm>")
            return

        if not self.svc.ensure_booted():
            return

        axis_id = AXES[axis]
        what = "moving %s axis to %.4f mm" % (axis.upper(), position)

        if not self.svc.run_request(Channel.STEPPER_ROUTER, CMD_MOVE,
                                    what, [axis_id, position]):
            print("interrupted; status = %s" % self.svc.status_name())
            print("debug-stepper-stop can stop the active move")
            return

        if self.svc.status() != DONE:
            print("stepper move failed: status = %s, resultCode = %d" %
                  (self.svc.status_name(), self.svc.result_code()))
            return

        moved_axis = AXIS_NAMES.get(self.svc.result_count(), "?")
        print("%s axis move complete" % moved_axis)


class DebugStepperRouterStop(BridgeCommand):
    """debug-stepper-stop: stop the active stepper-router debug move."""

    NAME = "debug-stepper-stop"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        if not self.svc.run_request(Channel.STEPPER_ROUTER, CMD_STOP,
                                    "stopping stepper router"):
            print("interrupted; status = %s" % self.svc.status_name())
            return

        print("stepper router stopped; status = %s" % self.svc.status_name())


def parse_move_args(arg):
    args = arg.split()

    if len(args) != 2:
        return None, None

    axis = args[0].lower()
    if axis not in AXES:
        return None, None

    return axis, float(args[1])
