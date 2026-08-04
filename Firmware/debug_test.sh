#!/usr/bin/env bash
# DebugBridge session: starts OpenOCD, attaches GDB with the debug tests,
# and cleans up
# OpenOCD when GDB exits.
#
# Usage:
#   ./debug_test.sh            # attach to the running firmware
#   ./debug_test.sh --flash    # reflash the debug image first
#
# The ELF must be a debug-environment image: set FIRMWARE_MODE to a
# FIRMWARE_MODE_DEBUG_* value in Core/Inc/configuration.h, then build:
#   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#   cmake --build build
#
# All modes share the single build/ directory; make sure the image on the
# chip matches the mode in configuration.h (use --flash after rebuilding).
#
# Environment overrides:
#   ELF=elsewhere/firmware ./debug_test.sh
#   GDB=gdb-multiarch OPENOCD=/opt/openocd/bin/openocd PORT=3334 ./debug_test.sh

set -euo pipefail
cd "$(dirname "$0")"

GDB="${GDB:-arm-none-eabi-gdb}"
OPENOCD="${OPENOCD:-openocd}"
# Deliberately NOT 3333: IDE debug extensions poll/attach to the default
# OpenOCD port in the background and steal the connection.
PORT="${PORT:-3341}"
ADAPTER_KHZ="${ADAPTER_KHZ:-1000}"
ELF="${ELF:-build/firmware}"
OPENOCD_LOG="openocd.log"

FLASH_ARGS=()
if [[ "${1:-}" == "--flash" ]]; then
    FLASH_ARGS=(-ex "load" -ex "monitor reset halt")
fi

if [[ ! -f "$ELF" ]]; then
    echo "error: $ELF not found — build the firmware first" >&2
    exit 1
fi

if nc -z localhost "$PORT" 2>/dev/null; then
    echo "error: port $PORT is already in use — another debug session is" >&2
    echo "running. Stop it first, or use PORT=<other>." >&2
    exit 1
fi

# A GDB or OpenOCD surviving from an earlier session will steal or block
# the connection ("attempted 'gdb' connection rejected" in openocd.log).
LEFTOVERS="$(pgrep -fl 'arm-none-eabi-gdb|openocd' || true)"
if [[ -n "$LEFTOVERS" ]]; then
    echo "error: leftover debug processes found — kill them first:" >&2
    echo "$LEFTOVERS" >&2
    echo "hint: pkill -f 'arm-none-eabi-gdb|openocd'" >&2
    exit 1
fi

"$OPENOCD" -f interface/stlink.cfg \
    -c "adapter speed $ADAPTER_KHZ" \
    -f target/stm32f4x.cfg \
    -c "gdb_port $PORT" >"$OPENOCD_LOG" 2>&1 &
OPENOCD_PID=$!
trap 'kill "$OPENOCD_PID" 2>/dev/null || true' EXIT

# Wait until OpenOCD reports its GDB server ready. Reading the log instead
# of probing the port avoids opening throwaway connections that OpenOCD
# can mistake for an attached client.
for _ in $(seq 1 50); do
    if grep -q "Listening on port $PORT for gdb connections" "$OPENOCD_LOG" 2>/dev/null; then
        break
    fi
    if ! kill -0 "$OPENOCD_PID" 2>/dev/null; then
        echo "error: OpenOCD exited — see $OPENOCD_LOG" >&2
        tail -5 "$OPENOCD_LOG" >&2
        exit 1
    fi
    sleep 0.1
done

exec_gdb() {
    "$GDB" "$ELF" \
        -ex "target extended-remote :$PORT" \
        -ex "monitor reset halt" \
        "${FLASH_ARGS[@]}" \
        -x DebugScripts/debug_bridge.py
}

exec_gdb
