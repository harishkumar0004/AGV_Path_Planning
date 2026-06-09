from enum import Enum


class MotionCommand(Enum):
    """Motion commands produced by navigation decisions."""

    START_FORWARD = "START_FORWARD"
    START_SLOW_FORWARD = "START_SLOW_FORWARD"
    TURN_RIGHT = "TURN_RIGHT"
    TURN_LEFT = "TURN_LEFT"
    STOP = "STOP"
