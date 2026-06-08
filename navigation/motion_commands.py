from enum import Enum


class MotionCommand(Enum):
    """Motion commands produced by navigation decisions."""

    FORWARD = "FORWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STOP = "STOP"
