from enum import Enum


class RouteAction(Enum):
    """Actions produced by the fixed route manager."""

    FORWARD = "FORWARD"
    TURN_RIGHT = "TURN_RIGHT"
    STOP = "STOP"
