from enum import Enum


class NavigationState(Enum):
    """High-level navigation states for continuous route execution."""

    WAIT_FOR_START = "WAIT_FOR_START"
    MOVING = "MOVING"
    PREPARE_TURN = "PREPARE_TURN"
    ALIGNING = "ALIGNING"
    TURNING = "TURNING"
    ROUTE_COMPLETE = "ROUTE_COMPLETE"
