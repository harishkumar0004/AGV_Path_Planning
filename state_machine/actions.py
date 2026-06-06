from enum import Enum, auto


class AGVAction(Enum):
    """Represents the action requested by the state machine."""

    # Tell the execution/control layer to move forward.
    MOVE_FORWARD = auto()

    # Tell the execution/control layer to stop.
    STOP = auto()

    # Tell the execution/control layer to turn left.
    TURN_LEFT = auto()

    # No command is needed for this event.
    NO_ACTION = auto()
