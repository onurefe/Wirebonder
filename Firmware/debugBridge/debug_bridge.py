"""GDB loader for the firmware DebugBridge tests.

Sourced by GDB, or by running ./debug_test.sh from the Firmware directory:

    arm-none-eabi-gdb build/firmware \
        -ex "target extended-remote :3333" \
        -x debugBridge/debug_bridge.py

    (gdb) debug-scan 64 56000 125
    (gdb) debug-pll 0.3 1.0 60500
    (gdb) debug-stop
    (gdb) debug-tone 0.3 60000
    (gdb) debug-tone-stop
    (gdb) debug-keys
    (gdb) debug-led all on
    (gdb) debug-led all off
    (gdb) debug-lcd 0 Hello, world!
    (gdb) debug-lcd clear
    (gdb) debug-io
    (gdb) debug-solenoid clamp energize
    (gdb) debug-solenoid clamp deenergize
    (gdb) debug-solenoid all deenergize
    (gdb) debug-motor-velocity 50 4
    (gdb) debug-motor-velocity 0.1 4 open
    (gdb) debug-motor-position 1.0 4
    (gdb) debug-motor-position 0.5 4 open
    (gdb) debug-force-coil 0.5 4
    (gdb) debug-force-coil 0.25 4 open
    (gdb) debug-stepper-router y 10
    (gdb) debug-stepper-router t 2.5
    (gdb) debug-stepper-stop
    (gdb) debug-status
"""

import os
import sys

try:
    PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    PACKAGE_DIR = os.path.join(os.getcwd(), "debugBridge")

PROJECT_DIR = os.path.dirname(PACKAGE_DIR)

if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

for name in [m for m in list(sys.modules)
             if m == "debugBridge" or m.startswith("debugBridge.")]:
    del sys.modules[name]

import debugBridge

debugBridge.register()
