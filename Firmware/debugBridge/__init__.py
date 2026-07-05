"""Firmware debug tools, driven over GDB.

Layered so each concern is isolated:

    target      - raw GDB target access
    service     - the DebugService command protocol
    tests/      - GDB command front-ends for each debug channel

The package is loaded by debugBridge/debug_bridge.py, usually through
./debug_test.sh.
"""

from .service import DebugService
from .target import Target
from .tests import register_all


def register():
    target = Target()
    service = DebugService(target)
    names = register_all(target, service)

    print("debugBridge loaded: " + " / ".join(names))
