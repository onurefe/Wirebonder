"""Client for the firmware DebugService command block."""

import gdb


BLOCK = "Robot::m_debugService.m_debugServiceBlock"

IDLE = 0
BUSY = 1
DONE = 2
ERROR = 3

STATUS_NAMES = {
    IDLE: "IDLE",
    BUSY: "BUSY",
    DONE: "DONE",
    ERROR: "ERROR",
}


class Channel:
    IMPEDANCE = 1
    PLL = 2
    TONE = 3
    KEYPAD = 4
    MOTOR_VELOCITY = 5
    FORCE_COIL = 6
    MOTOR_POSITION = 7
    STEPPER_ROUTER = 8
    LEDS = 9
    LCD = 10
    IO = 11
    SOLENOIDS = 12


def command_word(channel_id, local_command):
    return ((channel_id & 0xFFFF) << 16) | (local_command & 0xFFFF)


class TransactionWatchpoint(gdb.Breakpoint):
    """Silently halts the core when the firmware writes transactionCounter.
    Completion is judged by run_request from the counter value, not from
    whether GDB attributes the stop back to this object."""

    def __init__(self, expression):
        super().__init__(
            expression,
            type=gdb.BP_WATCHPOINT,
            wp_class=gdb.WP_WRITE,
            internal=True,
        )
        self.silent = True

    def stop(self):
        return True


class DebugService:
    NUM_ARGS = 5
    NUM_RESULT_POINTERS = 4
    TICK_SYMBOL = "DebugService::executeService"

    def __init__(self, target, block=BLOCK):
        self.t = target
        self.block = block

    def ready(self):
        try:
            return self.t.eval_int("Robot::m_debugService.m_channelCount") > 0
        except gdb.error:
            return False

    def ensure_booted(self):
        if self.ready():
            return True

        print("firmware not booted yet - running to the main loop "
              "(Ctrl-C if this hangs)...")
        self.t.run_to(self.TICK_SYMBOL)

        if not self.ready():
            print("error: main loop not reached - check firmware/reset state")
            return False

        return True

    def expr(self, field):
        return "%s.%s" % (self.block, field)

    def array_expr(self, field, index):
        return "%s.%s[%d]" % (self.block, field, index)

    def status(self):
        return self.t.eval_int(self.expr("status"))

    def status_name(self):
        return STATUS_NAMES.get(self.status(), "?")

    def result_code(self):
        return self.t.eval_int(self.expr("resultCode"))

    def result_count(self):
        return self.t.eval_int(self.expr("resultCount"))

    def transaction_counter(self):
        return self.t.eval_int(self.expr("transactionCounter"))

    def arg(self, index):
        return self.t.eval_float(self.array_expr("args", index))

    def result_pointer(self, index):
        return self.t.eval_int(self.array_expr("resultPointers", index))

    def submit(self, channel_id, local_command, args=None):
        values = [0.0] * self.NUM_ARGS

        for index, value in enumerate(args or []):
            if index >= self.NUM_ARGS:
                break
            values[index] = float(value)

        for index, value in enumerate(values):
            self.t.set_var(self.array_expr("args", index), value)

        self.t.set_var(self.expr("command"),
                       command_word(channel_id, local_command))

    def pump(self):
        return self.t.run_until(self.TICK_SYMBOL)

    def run_request(self, channel_id, local_command, what, args=None):
        if what:
            print("%s..." % what)

        # The watchpoint halts the core at the firmware's transactionCounter
        # write, but we do NOT trust GDB to attribute the stop back to it:
        # some OpenOCD/probe combos report the DWT hit as a bare SIGTRAP, so
        # watchpoint.stop() never runs. The counter delta is the firmware's
        # guaranteed source of truth (incremented last, on DONE/ERROR only), so
        # decide completion from that: counter moved -> the command finished;
        # counter unchanged -> a genuine Ctrl-C / unrelated stop.
        watchpoint = TransactionWatchpoint(self.expr("transactionCounter"))

        try:
            old_counter = self.transaction_counter()
            self.submit(channel_id, local_command, args)

            while True:
                self.t.cont()

                new_counter = self.transaction_counter()
                if new_counter == old_counter:
                    return False

                status = self.status()
                if status in (DONE, ERROR):
                    return True

                old_counter = new_counter
        finally:
            if watchpoint.is_valid():
                watchpoint.delete()
