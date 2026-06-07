import cv2
import numpy as np

from apriltag_detector import AprilTagDetector
from camera_manager import CameraManager


def draw_detection(frame: np.ndarray, detection: dict) -> None:
    """
    Draw one AprilTag detection on the camera frame.

    Args:
        frame: Camera image where the detection should be drawn.
        detection: Detection dictionary returned by AprilTagDetector.
    """
    tag_id = detection["tag_id"]
    center_x = int(detection["center_x"])
    center_y = int(detection["center_y"])
    corners = np.array(detection["corners"], dtype=np.int32)

    # Draw the tag boundary using the four detected corner points.
    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

    # Draw the center point of the detected tag.
    cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)

    # Display the tag ID near the tag boundary.
    cv2.putText(
        frame,
        f"ID: {tag_id}",
        (center_x + 10, center_y - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2,
    )


def main() -> None:
    """Run a live AprilTag detection test using the Raspberry Pi camera."""
    camera = CameraManager()
    detector = AprilTagDetector()

    if not camera.initialize():
        print("Camera failed to initialize.")
        return

    try:
        while True:
            # Capture one live frame from CameraManager.
            frame = camera.get_frame()
            if frame is None:
                print("Frame capture failed.")
                break

            # Detect AprilTags in the current camera frame.
            detections = detector.detect(frame)

            for detection in detections:
                tag_id = detection["tag_id"]
                center_x = int(detection["center_x"])
                center_y = int(detection["center_y"])

                # Print simple detection information to the terminal.
                print(f"Tag ID: {tag_id}")
                print(f"Center: ({center_x}, {center_y})")

                draw_detection(frame, detection)

            cv2.imshow("AGV AprilTag Test", frame)

            # Press q to exit the live test.
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        # Always release hardware and display resources before exiting.
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
