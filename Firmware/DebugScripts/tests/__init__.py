"""Test registry for DebugBridge GDB tests."""

from .bonder import (DebugBonderWatch, DebugBonderStatus, DebugBonderStart,
                     DebugBonderStop, DebugBonderButton)
from .keypad import DebugKeys
from .lcd import DebugLcd
from .leds import DebugLeds
from .force_coil import DebugForceCoil
from .homing import DebugHome, DebugCenter, DebugHomingStop
from .motor_position import DebugMotorPosition, DebugMotorPositionStall
from .motor_velocity import DebugMotorVelocity
from .pll import DebugPll, DebugStop
from .scan import DebugScan
from .status import DebugStatus
from .solenoids import DebugSolenoids
from .stepper_router import DebugStepperRouter, DebugStepperRouterStop
from .tone import DebugTone, DebugToneStop

COMMAND_CLASSES = [
    DebugStatus,
    DebugScan,
    DebugPll,
    DebugStop,
    DebugTone,
    DebugToneStop,
    DebugKeys,
    DebugLeds,
    DebugLcd,
    DebugSolenoids,
    DebugMotorVelocity,
    DebugMotorPosition,
    DebugMotorPositionStall,
    DebugForceCoil,
    DebugStepperRouter,
    DebugStepperRouterStop,
    DebugHome,
    DebugCenter,
    DebugHomingStop,
    DebugBonderWatch,
    DebugBonderStatus,
    DebugBonderStart,
    DebugBonderStop,
    DebugBonderButton,
]


def register_all(target, service):
    names = []

    for cls in COMMAND_CLASSES:
        cls(target, service)
        names.append(cls.NAME)

    return names
