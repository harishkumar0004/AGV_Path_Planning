import cv2
import numpy as np

from core.application_state import ApplicationState
from control.serial_commands import SerialCommand
from control.serial_motor_controller import SerialMotorController
from navigation.alignment_controller import AlignmentController
from navigation.motion_commands import MotionCommand
from perception.perception_manager import PerceptionManager


def draw_tolerance_band(
    frame: np.ndarray,
    frame_center_x: int,
    tolerance_px: int,
) -> None:
    """
    Draw the center line and tolerance limits on the camera frame.

    Args:
        frame: Camera image to annotate.
        frame_center_x: Horizontal center of the frame.
        tolerance_px: Alignment tolerance around the center.
    """
    height = frame.shape[0]
    left_limit = frame_center_x - tolerance_px
    right_limit = frame_center_x + tolerance_px

    cv2.line(frame, (frame_center_x, 0), (frame_center_x, height), (255, 0, 0), 2)
    cv2.line(frame, (left_limit, 0), (left_limit, height), (0, 255, 255), 1)
    cv2.line(frame, (right_limit, 0), (right_limit, height), (0, 255, 255), 1)


def draw_tag(frame: np.ndarray, detections: list[dict]) -> None:
    """
    Draw the first visible AprilTag boundary and center.

    Args:
        frame: Camera image to annotate.
        detections: AprilTag detections from PerceptionManager.
    """
    if not detections:
        return

    detection = detections[0]
    center_x = int(detection["center_x"])
    center_y = int(detection["center_y"])
    corners = np.array(detection["corners"], dtype=np.int32)

    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
    cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)


def draw_command(frame: np.ndarray, command: MotionCommand) -> None:
    """
    Draw the current motion command on the camera frame.

    Args:
        frame: Camera image to annotate.
        command: Latest MotionCommand decision.
    """
    cv2.putText(
        frame,
        f"Command: {command.name}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        2,
    )


def to_serial_command(command: MotionCommand) -> SerialCommand:
    """
    Convert a navigation MotionCommand to a serial command.

    Args:
        command: Motion command from AlignmentController.

    Returns:
        SerialCommand sent to the ESP32.
    """
    if command == MotionCommand.FORWARD:
        return SerialCommand.FORWARD

    if command == MotionCommand.LEFT:
        return SerialCommand.LEFT

    if command == MotionCommand.RIGHT:
        return SerialCommand.RIGHT

    return SerialCommand.STOP


def main() -> None:
    """Run the live camera-to-serial alignment pipeline."""
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    alignment_controller = AlignmentController(center_tolerance_px=40.0)
    serial_controller = SerialMotorController()
    last_command: MotionCommand | None = None

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    try:
        while True:
            detections = perception_manager.update()
            frame = perception_manager.last_frame

            if frame is None:
                command = MotionCommand.STOP
            else:
                perception = application_state.perception
                frame_center_x = frame.shape[1] / 2
                tag_center_x = perception.center_x if perception.tag_visible else None
                command = alignment_controller.decide(frame_center_x, tag_center_x)

                draw_tolerance_band(
                    frame,
                    frame_center_x=int(frame_center_x),
                    tolerance_px=40,
                )
                draw_tag(frame, detections)
                draw_command(frame, command)
                cv2.imshow("AGV Alignment Serial Test", frame)

            if command != last_command:
                serial_controller.send_command(to_serial_command(command))
                last_command = command

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        serial_controller.send_command(SerialCommand.STOP)
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
