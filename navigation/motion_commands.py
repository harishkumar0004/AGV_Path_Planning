from enum import Enum


class MotionCommand(Enum):
    """Motion commands produced by navigation decisions."""

    START_CONTINUOUS_FORWARD = "START_CONTINUOUS_FORWARD"
    FORWARD = "FORWARD"
    SLOW_FORWARD = "SLOW_FORWARD"
    SLOW_MODE = "SLOW_MODE"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    TURN_RIGHT = "TURN_RIGHT"
    STOP = "STOP"
