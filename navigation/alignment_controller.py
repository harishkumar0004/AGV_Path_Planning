from control.motor_commands import MotorCommand
from navigation.navigation_models import AlignmentResult


class AlignmentController:
    """Decides how the robot should move based on AprilTag horizontal position."""

    def __init__(self, deadband: float = 20.0) -> None:
        """
        Create an alignment controller.

        Args:
            deadband: Allowed center error where the robot can move forward.
        """
        self.deadband = deadband

    def calculate(self, image_width: int, tag_center_x: float) -> AlignmentResult:
        """
        Calculate horizontal alignment error and suggested action.

        Args:
            image_width: Width of the camera image in pixels.
            tag_center_x: X coordinate of the detected AprilTag center.

        Returns:
            AlignmentResult containing error_x and suggested action.
        """
        image_center_x = image_width / 2
        error_x = tag_center_x - image_center_x

        # Tag is left of center, so rotate left.
        if error_x < -self.deadband:
            return AlignmentResult(error_x=error_x, action=MotorCommand.TURN_LEFT)

        # Tag is right of center, so rotate right.
        if error_x > self.deadband:
            return AlignmentResult(error_x=error_x, action=MotorCommand.TURN_RIGHT)

        # Tag is centered enough, so move forward.
        return AlignmentResult(error_x=error_x, action=MotorCommand.FORWARD)
