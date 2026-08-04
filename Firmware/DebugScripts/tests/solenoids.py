"""Direct-solenoid visual-inspection commands."""

from ..service import Channel, DONE
from .base import BridgeCommand

CMD_SET = 1

SOLENOIDS = {
    "clamp": 0.0,
    "sol1": 1.0,
    "sol2": 2.0,
    "all": 3.0,
}

STATES = {
    "deenergize": 0.0,
    "energize": 1.0,
}


class DebugSolenoids(BridgeCommand):
    """debug-solenoid <clamp|sol1|sol2|all> <energize|deenergize>."""

    NAME = "debug-solenoid"

    def invoke(self, arg, from_tty):
        solenoid, state = parse_args(arg)
        if solenoid is None:
            print("usage: debug-solenoid <clamp|sol1|sol2|all> "
                  "<energize|deenergize>")
            return

        if not self.svc.ensure_booted():
            return

        name, requested_state = arg.lower().split()
        what = "%s %s" % (requested_state, name)
        if not self.svc.run_request(Channel.SOLENOIDS, CMD_SET, what,
                                    [solenoid, state]):
            print("interrupted; status = %s" % self.svc.status_name())
            print("use 'debug-solenoid all deenergize' to release all outputs")
            return

        if self.svc.status() != DONE:
            print("solenoid command failed: status = %s, resultCode = %d" %
                  (self.svc.status_name(), self.svc.result_code()))
            return

        print("%s %s; %d channel(s) settled" %
              (name, requested_state + "d", self.svc.result_count()))


def parse_args(arg):
    args = arg.lower().split()
    if len(args) != 2:
        return None, None

    solenoid = SOLENOIDS.get(args[0])
    state = STATES.get(args[1])
    if solenoid is None or state is None:
        return None, None

    return solenoid, state
