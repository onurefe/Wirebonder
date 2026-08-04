"""Control-panel LED visual-inspection commands."""

from ..service import Channel, DONE
from .base import BridgeCommand

CMD_SET = 1

LEDS = {
    "test": 0.0,
    "setup": 1.0,
    "clamp-open": 2.0,
    "manual": 3.0,
    "all": 4.0,
}

STATES = {
    "off": 0.0,
    "on": 1.0,
}


class DebugLeds(BridgeCommand):
    """debug-led <test|setup|clamp-open|manual|all> <on|off>."""

    NAME = "debug-led"

    def invoke(self, arg, from_tty):
        led, state = parse_args(arg)
        if led is None:
            print("usage: debug-led <test|setup|clamp-open|manual|all> <on|off>")
            return

        if not self.svc.ensure_booted():
            return

        what = "setting %s LED %s" % (arg.split()[0], arg.split()[1])
        if not self.svc.run_request(Channel.LEDS, CMD_SET, what, [led, state]):
            print("interrupted; status = %s" % self.svc.status_name())
            return

        if self.svc.status() != DONE:
            print("LED command failed: status = %s, resultCode = %d" %
                  (self.svc.status_name(), self.svc.result_code()))
            return

        print("updated %d LED(s)" % self.svc.result_count())


def parse_args(arg):
    args = arg.lower().split()
    if len(args) != 2:
        return None, None

    led = LEDS.get(args[0])
    state = STATES.get(args[1])
    if led is None or state is None:
        return None, None

    return led, state
