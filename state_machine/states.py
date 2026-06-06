from enum import Enum, auto


class AGVState(Enum):
    """Represents the high-level operating state of the AGV."""

    # The AGV is powered and waiting for a command.
    IDLE = auto()

    # The AGV is moving forward.
    MOVING = auto()

    # The AGV is performing a turn.
    TURNING = auto()

    # The AGV has detected a problem and should stop safely.
    ERROR = auto()
