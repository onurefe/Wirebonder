"""Raw GDB target access: memory, symbols, and run control."""

import struct

import gdb


class Target:
    def eval(self, expr):
        return gdb.parse_and_eval(expr)

    def eval_int(self, expr):
        return int(gdb.parse_and_eval(expr))

    def eval_float(self, expr):
        return float(gdb.parse_and_eval(expr))

    def set_var(self, lvalue, value):
        gdb.execute("set var %s = %r" % (lvalue, value), to_string=True)

    def symbol_address(self, symbol):
        return int(gdb.parse_and_eval("(unsigned long)&%s" % symbol))

    def read_bytes(self, addr, nbytes):
        return bytes(gdb.selected_inferior().read_memory(addr, nbytes))

    def read_floats(self, addr, count):
        data = self.read_bytes(addr, count * 4)
        return list(struct.unpack("<%df" % count, data))

    def read_u32(self, addr):
        return int(gdb.parse_and_eval("*(unsigned int*)0x%08x" % addr))

    def write_u32(self, addr, value):
        gdb.execute("set var *(unsigned int*)0x%08x = 0x%x" %
                    (addr, value), to_string=True)

    def cont(self):
        gdb.execute("continue")

    def run_to(self, symbol):
        bp = gdb.Breakpoint(symbol, temporary=True, internal=True)

        try:
            gdb.execute("continue")
        finally:
            if bp.is_valid():
                bp.delete()

        frame_name = gdb.selected_frame().name() or ""

        return symbol.split("::")[-1] in frame_name

    def run_until(self, symbol, what=None):
        if what:
            print("%s - waiting for the hardware (Ctrl-C to abort)..." % what)

        if not self.run_to(symbol):
            return False

        gdb.execute("finish", to_string=True)

        return True
