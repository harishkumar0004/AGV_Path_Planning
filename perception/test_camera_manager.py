import time

import cv2

from perception.camera_manager import CameraManager


def main() -> None:
    """Run a simple manual camera test with an FPS overlay."""
    camera_manager = CameraManager()

    # Initialize the camera before trying to read frames.
    if not camera_manager.initialize():
        print("Camera failed to initialize.")
        return

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            # Capture one frame from the camera.
            frame = camera_manager.get_frame()
            if frame is None:
                print("Frame capture failed.")
                break

            frame_count += 1
            elapsed_time = time.time() - start_time

            # Calculate average FPS using total frames divided by elapsed time.
            fps = frame_count / elapsed_time if elapsed_time > 0 else 0.0

            # Display the FPS value directly on the camera frame.
            cv2.putText(
                frame,
                f"FPS: {fps:.2f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
            )

            cv2.imshow("AGV Camera Test", frame)

            # Exit the display loop when the user presses q.
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        # Always release camera and window resources before exiting.
        camera_manager.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
