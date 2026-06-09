from enum import Enum


class RouteAction(Enum):
    """Actions produced by the fixed route manager."""

    START_FORWARD = "START_FORWARD"
    START_SLOW_FORWARD = "START_SLOW_FORWARD"
    TURN_RIGHT = "TURN_RIGHT"
    STOP = "STOP"
