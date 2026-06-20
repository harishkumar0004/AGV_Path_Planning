import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agv_high_level.communication.esp32_client import ESP32Client
from agv_high_level.config.settings import ESP32_BAUDRATE, ESP32_PORT
from agv_high_level.perception.tag_pose import TagPose
from agv_high_level.perception.tag_pose_streamer import TagPoseStreamer


WINDOW_NAME = "AGV Real AprilTag TAGPOSE Stream"
POSE_PRINT_PERIOD_SEC = 0.2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream real camera TAGPOSE measurements to the ESP32."
    )
    parser.add_argument("--port", default=ESP32_PORT)
    parser.add_argument("--baudrate", type=int, default=ESP32_BAUDRATE)
    return parser.parse_args()


def print_esp32_lines(client: ESP32Client) -> None:
    for line in client.read_lines():
        print(f"ESP32 {line}")


def main() -> None:
    # Keep Raspberry Pi camera dependencies out of command-line parsing so
    # --help remains usable on development machines without camera hardware.
    args = parse_args()

    import cv2

    from perception.apriltag_detector import AprilTagDetector
    from perception.camera_manager import CameraManager
    from perception.tests.send_tag_to_esp32 import (
        add_detection_measurements,
        choose_best_detection,
        draw_debug,
    )

    client = ESP32Client(port=args.port, baudrate=args.baudrate)
    streamer = TagPoseStreamer(client)
    camera = CameraManager()
    detector = AprilTagDetector()
    last_pose_print_time = 0.0

    try:
        client.connect()
        time.sleep(2.0)
        print(f"ESP32 connected: {args.port} @ {args.baudrate}")

        if not camera.initialize():
            raise RuntimeError("Camera failed to initialize")

        while True:
            frame = camera.get_frame()
            if frame is None:
                streamer.update(TagPose.lost())
                print_esp32_lines(client)
                continue

            frame_height, frame_width = frame.shape[:2]
            image_center_x = frame_width / 2.0
            image_center_y = frame_height / 2.0
            detections = detector.detect(frame)

            for detected_tag in detections:
                add_detection_measurements(
                    detected_tag,
                    image_center_x,
                    image_center_y,
                    frame_width,
                    frame_height,
                )

            detection = choose_best_detection(detections)
            now = time.monotonic()

            if detection is not None:
                estimated_pose = detection.get("pose")
                if estimated_pose is not None and estimated_pose["valid"]:
                    pose = TagPose(
                        visible=True,
                        tag_id=detection["tag_id"],
                        x_m=estimated_pose["x_m"],
                        y_m=estimated_pose["y_m"],
                        z_m=estimated_pose["z_m"],
                        yaw_deg=estimated_pose["yaw_deg"],
                        timestamp=now,
                    )
                    streamer.update(pose)

                    if now - last_pose_print_time >= POSE_PRINT_PERIOD_SEC:
                        print(
                            f"TAGPOSE id={pose.tag_id} "
                            f"xM={pose.x_m:.4f} yM={pose.y_m:.4f} "
                            f"zM={pose.z_m:.4f} yaw={pose.yaw_deg:.2f}"
                        )
                        last_pose_print_time = now
                else:
                    streamer.update(TagPose.lost())

                draw_debug(
                    frame,
                    detection,
                    detection["x_norm"],
                    detection["y_norm"],
                    detection["theta_deg"],
                )
            else:
                streamer.update(TagPose.lost())

            print_esp32_lines(client)
            cv2.imshow(WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            if key == ord("s"):
                client.stop()
                print("Sent: STOP")
            elif key == ord("a"):
                client.align_on()
                print("Sent: ALIGN ON")
            elif key == ord("o"):
                client.align_off()
                print("Sent: ALIGN OFF")
            elif key == ord("p"):
                client.align_mode_pose()
                print("Sent: ALIGN MODE POSE")
    except KeyboardInterrupt:
        pass
    finally:
        if client.connected:
            client.stop()
            time.sleep(0.1)
            print_esp32_lines(client)
        client.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
