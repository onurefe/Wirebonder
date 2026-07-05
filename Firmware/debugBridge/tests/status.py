"""Status command for the DebugService block."""

from .base import BridgeCommand


class DebugStatus(BridgeCommand):
    """debug-status: print the DebugService block state."""

    NAME = "debug-status"

    def invoke(self, arg, from_tty):
        if not self.svc.ensure_booted():
            return

        print("DebugService")
        print("  status             : %s" % self.svc.status_name())
        print("  resultCode         : %d" % self.svc.result_code())
        print("  resultCount        : %d" % self.svc.result_count())
        print("  transactionCounter : %d" %
              self.svc.transaction_counter())

        for index in range(self.svc.NUM_ARGS):
            print("  args[%d]            : %.6g" %
                  (index, self.svc.arg(index)))

        for index in range(self.svc.NUM_ARGS):
            print("  resultPointers[%d]  : 0x%08x" %
                  (index, self.svc.result_pointer(index)))
