from navigation.navigation_models import AlignmentResult
from navigation.motion_commands import MotionCommand


class AlignmentController:
    """Decides how the robot should move based on AprilTag horizontal position."""

    def __init__(self, center_tolerance_px: float = 40.0) -> None:
        """
        Create an alignment controller.

        Args:
            center_tolerance_px: Allowed pixel error around the frame center.
        """
        self.center_tolerance_px = center_tolerance_px

    def decide(
        self,
        frame_center_x: float,
        tag_center_x: float | None,
    ) -> MotionCommand:
        """
        Decide the motion command from the horizontal AprilTag position.

        Args:
            frame_center_x: Horizontal center of the camera frame.
            tag_center_x: Horizontal center of the detected tag, or None.

        Returns:
            MotionCommand for the serial motor controller.
        """
        if tag_center_x is None:
            return MotionCommand.STOP

        left_limit = frame_center_x - self.center_tolerance_px
        right_limit = frame_center_x + self.center_tolerance_px

        if tag_center_x < left_limit:
            return MotionCommand.TURN_LEFT

        if tag_center_x > right_limit:
            return MotionCommand.TURN_RIGHT

        return MotionCommand.START_FORWARD

    def calculate(self, image_width: int, tag_center_x: float) -> AlignmentResult:
        """
        Calculate horizontal alignment error and suggested motion command.

        Args:
            image_width: Width of the camera image in pixels.
            tag_center_x: X coordinate of the detected AprilTag center.

        Returns:
            AlignmentResult containing error_x and suggested action.
        """
        image_center_x = image_width / 2
        error_x = tag_center_x - image_center_x
        action = self.decide(image_center_x, tag_center_x)

        return AlignmentResult(error_x=error_x, action=action)
