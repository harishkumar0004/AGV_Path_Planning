from enum import Enum


class SerialCommand(Enum):
    """Human-readable commands sent from Raspberry Pi to Arduino Mega."""

    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STOP = "STOP"
