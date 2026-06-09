from enum import Enum


class MotionCommand(Enum):
    """Motion commands produced by navigation decisions."""

    FORWARD = "FORWARD"
    SLOW_FORWARD = "SLOW_FORWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    TURN_RIGHT = "TURN_RIGHT"
    STOP = "STOP"
