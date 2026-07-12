"""LCD visual-inspection commands."""

from ..service import Channel, DONE
from .base import BridgeCommand

CMD_WRITE_LINE = 1
CMD_CLEAR = 2

TEXT_BUFFER = "Robot::m_debugChannelLcd.m_text"
MAX_ROWS = 4
MAX_COLUMNS = 20

USAGE = "usage: debug-lcd <clear | row(0-%d) text...>" % (MAX_ROWS - 1)


class DebugLcd(BridgeCommand):
    """debug-lcd clear | debug-lcd <row 0-3> <text...>."""

    NAME = "debug-lcd"

    def invoke(self, arg, from_tty):
        args = arg.split(None, 1)
        if not args:
            print(USAGE)
            return

        if not self.svc.ensure_booted():
            return

        if args[0].lower() == "clear":
            self.run(CMD_CLEAR, "clearing LCD", [])
            return

        try:
            row = int(args[0])
        except ValueError:
            row = -1

        if not 0 <= row < MAX_ROWS:
            print(USAGE)
            return

        text = args[1] if len(args) > 1 else ""
        if len(text) > MAX_COLUMNS:
            print("note: text truncated to %d characters" % MAX_COLUMNS)
            text = text[:MAX_COLUMNS]

        for index, char in enumerate(text):
            self.t.set_var("%s[%d]" % (TEXT_BUFFER, index), ord(char))
        self.t.set_var("%s[%d]" % (TEXT_BUFFER, len(text)), 0)

        self.run(CMD_WRITE_LINE,
                 "writing \"%s\" to LCD line %d" % (text, row),
                 [row])

    def run(self, command, what, args):
        if not self.svc.run_request(Channel.LCD, command, what, args):
            print("interrupted; status = %s" % self.svc.status_name())
            return

        if self.svc.status() != DONE:
            print("LCD command failed: status = %s, resultCode = %d" %
                  (self.svc.status_name(), self.svc.result_code()))
            return

        print("done (%d character(s) written)" % self.svc.result_count())
