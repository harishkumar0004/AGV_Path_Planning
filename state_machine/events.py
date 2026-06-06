from enum import Enum, auto


class AGVEvent(Enum):
    """Represents an external event received by the state machine."""

    # Command to start moving.
    START = auto()

    # Command to stop moving.
    STOP = auto()

    # Command or request to begin turning.
    TURN_REQUESTED = auto()

    # Notification that an error has been detected.
    ERROR_DETECTED = auto()

    # Notification that an AprilTag became visible.
    TAG_DETECTED = auto()

    # Notification that the visible AprilTag disappeared.
    TAG_LOST = auto()

    # Notification that the visible AprilTag ID changed.
    TAG_CHANGED = auto()
