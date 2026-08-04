"""Base class for DebugBridge GDB tests."""

import gdb


class BridgeCommand(gdb.Command):
    NAME = None

    def __init__(self, target, service):
        super().__init__(self.NAME, gdb.COMMAND_USER)
        self.t = target
        self.svc = service
