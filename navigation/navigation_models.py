from dataclasses import dataclass

from control.motor_commands import MotorCommand


@dataclass
class AlignmentResult:
    """Result of horizontal AprilTag alignment calculation."""

    # Horizontal difference between tag center and camera center.
    error_x: float

    # Suggested motor command for correcting the alignment.
    action: MotorCommand
