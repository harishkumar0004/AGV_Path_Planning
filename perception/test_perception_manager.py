import cv2
import numpy as np

<<<<<<< HEAD
from core.application_state import ApplicationState
from perception.perception_manager import PerceptionManager
=======
from perception_manager import PerceptionManager
>>>>>>> 19d056c (upadte tha changes)


def draw_detection(frame: np.ndarray, detection: dict) -> None:
    """
    Draw an AprilTag outline, center point, and tag ID on a frame.

    Args:
        frame: Camera image where the detection should be drawn.
        detection: Detection dictionary returned by PerceptionManager.
    """
    tag_id = detection["tag_id"]
    center_x = int(detection["center_x"])
    center_y = int(detection["center_y"])
    corners = np.array(detection["corners"], dtype=np.int32)

    # Draw the AprilTag boundary.
    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

    # Draw the tag center point.
    cv2.circle(frame, (center_x, center_y), radius=5, color=(0, 0, 255), thickness=-1)

    # Draw the tag ID beside the center point.
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
    """Run a live perception test: camera frame to AprilTag detections."""
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    try:
        while True:
            # Capture a frame and detect AprilTags through one perception entry point.
            detections = perception_manager.update()
            frame = perception_manager.last_frame

            if frame is None:
                print("No camera frame available.")
                break

            for detection in detections:
                tag_id = detection["tag_id"]
                center_x = int(detection["center_x"])
                center_y = int(detection["center_y"])

                # Print simple detection information to the terminal.
                print(f"Tag ID: {tag_id}")
                print(f"Center: ({center_x}, {center_y})")

                draw_detection(frame, detection)

            cv2.imshow("AGV Perception Test", frame)

            # Press q to exit the live perception test.
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        # Always release camera and display resources before leaving the test.
        perception_manager.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
