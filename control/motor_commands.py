from enum import Enum, auto


class MotorCommand(Enum):
    """Commands that can be sent to the simulated motor controller."""

    # Move the robot forward.
    FORWARD = auto()

    # Move the robot backward.
    BACKWARD = auto()

    # Rotate the robot to the left.
    TURN_LEFT = auto()

    # Rotate the robot to the right.
    TURN_RIGHT = auto()

    # Stop all robot motion.
    STOP = auto()
