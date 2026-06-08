from dataclasses import dataclass

from navigation.motion_commands import MotionCommand


@dataclass
class AlignmentResult:
    """Result of horizontal AprilTag alignment calculation."""

    # Horizontal difference between tag center and camera center.
    error_x: float

    # Suggested motion command for correcting the alignment.
    action: MotionCommand
