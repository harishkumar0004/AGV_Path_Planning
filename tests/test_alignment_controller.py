import cv2
import numpy as np

from navigation.alignment_controller import AlignmentController
from perception.apriltag_detector import AprilTagDetector
from perception.camera_manager import CameraManager


def draw_detection(frame: np.ndarray, detection: dict) -> None:
    """
    Draw AprilTag boundary and center point on the camera frame.

    Args:
        frame: Camera image where the detection should be drawn.
        detection: Detection dictionary returned by AprilTagDetector.
    """
    center_x = int(detection["center_x"])
    center_y = int(detection["center_y"])
    corners = np.array(detection["corners"], dtype=np.int32)

    # Draw the AprilTag outline.
    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

    # Draw the AprilTag center point.
    cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)


def draw_camera_center_line(frame: np.ndarray) -> None:
    """
    Draw the vertical center line of the camera image.

    Args:
        frame: Camera image where the center line should be drawn.
    """
    height, width = frame.shape[:2]
    image_center_x = width // 2
    cv2.line(
        frame,
        (image_center_x, 0),
        (image_center_x, height),
        color=(255, 0, 0),
        thickness=2,
    )


def draw_overlay(
    frame: np.ndarray,
    tag_id: int | None,
    error_x: float | None,
    action: str,
) -> None:
    """
    Draw tag ID, alignment error, and action on the frame.

    Args:
        frame: Camera image where text should be drawn.
        tag_id: Detected tag ID, or None if no tag is visible.
        error_x: Horizontal alignment error, or None if no tag is visible.
        action: Suggested alignment action.
    """
    error_text = f"{error_x:.2f}" if error_x is not None else "None"
    cv2.putText(
        frame,
        f"Tag ID: {tag_id}",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"Error X: {error_text}",
        (20, 65),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"Action: {action}",
        (20, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
    )


def main() -> None:
    """Run live AprilTag alignment testing from camera frames."""
    camera_manager = CameraManager()
    apriltag_detector = AprilTagDetector()
    alignment_controller = AlignmentController()

    if not camera_manager.initialize():
        print("Camera failed to initialize.")
        return

    try:
        while True:
            frame = camera_manager.get_frame()
            if frame is None:
                print("Frame capture failed.")
                break

            detections = apriltag_detector.detect(frame)
            draw_camera_center_line(frame)

            tag_id = None
            error_x = None
            action = "NO_TAG"

            if detections:
                # Use the first visible tag for this beginner-friendly test.
                detection = detections[0]
                image_width = frame.shape[1]
                result = alignment_controller.calculate(
                    image_width=image_width,
                    tag_center_x=detection["center_x"],
                )

                tag_id = detection["tag_id"]
                error_x = result.error_x
                action = result.action.name

                draw_detection(frame, detection)
                print(f"Tag ID: {tag_id}")
                print(f"Error X: {error_x:.2f}")
                print(f"Action: {action}")

            draw_overlay(frame, tag_id, error_x, action)
            cv2.imshow("AGV Alignment Test", frame)

            # Press q to exit the live alignment test.
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        # Always release camera and display resources before exiting.
        camera_manager.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
