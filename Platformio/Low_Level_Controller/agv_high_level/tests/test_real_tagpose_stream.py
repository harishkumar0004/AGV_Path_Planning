import argparse
import sys
import time
from dataclasses import dataclass
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


@dataclass
class ESP32NavTelemetry:
    nav_state: str = "UNKNOWN"
    current_tag: int | None = None
    expected_next: int | None = None
    pose_id: int | None = None
    imu_heading_deg: float | None = None
    target_heading_deg: float | None = None
    heading_error_deg: float | None = None
    v_cmd: float | None = None
    w_cmd: float | None = None
    left_hz: float | None = None
    right_hz: float | None = None
    left_steps: int | None = None
    right_steps: int | None = None
    aligned: str = "NO"
    message: str = ""
    last_update_time: float = 0.0


class FPSCounter:
    def __init__(self):
        self.window_start = time.monotonic()
        self.frame_count = 0
        self.fps = 0.0

    def tick(self):
        self.frame_count += 1
        now = time.monotonic()
        elapsed = now - self.window_start
        if elapsed >= 1.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.window_start = now
        return self.fps


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream real camera TAGPOSE measurements to the ESP32."
    )
    parser.add_argument("--port", default=ESP32_PORT)
    parser.add_argument("--baudrate", type=int, default=ESP32_BAUDRATE)
    return parser.parse_args()


def parse_nav_fields(line: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in line.split():
        key, separator, value = token.partition("=")
        if separator:
            fields[key] = value
    return fields


def update_nav_telemetry(line: str, telemetry: ESP32NavTelemetry) -> None:
    now = time.monotonic()

    if line.startswith("NAV MOTOR state="):
        fields = parse_nav_fields(line)
        telemetry.nav_state = fields.get("state", telemetry.nav_state)
        telemetry.v_cmd = parse_float(fields.get("vCmd"), telemetry.v_cmd)
        telemetry.w_cmd = parse_float(fields.get("wCmd"), telemetry.w_cmd)
        telemetry.left_hz = parse_float(fields.get("leftHz"), telemetry.left_hz)
        telemetry.right_hz = parse_float(
            fields.get("rightHz"), telemetry.right_hz
        )
        telemetry.left_steps = parse_int(
            fields.get("leftSteps"), telemetry.left_steps
        )
        telemetry.right_steps = parse_int(
            fields.get("rightSteps"), telemetry.right_steps
        )
        telemetry.last_update_time = now
        return

    if line.startswith("NAV state="):
        fields = parse_nav_fields(line)
        telemetry.nav_state = fields.get("state", telemetry.nav_state)
        telemetry.current_tag = parse_int(
            fields.get("currentTag", fields.get("segment")),
            telemetry.current_tag,
        )
        telemetry.expected_next = parse_int(
            fields.get("expectedNext"), telemetry.expected_next
        )
        telemetry.pose_id = parse_int(fields.get("poseId"), telemetry.pose_id)
        telemetry.imu_heading_deg = parse_float(
            fields.get("imu"), telemetry.imu_heading_deg
        )
        telemetry.target_heading_deg = parse_float(
            fields.get("targetHeading"), telemetry.target_heading_deg
        )
        telemetry.heading_error_deg = parse_float(
            fields.get("headingError"), telemetry.heading_error_deg
        )
        telemetry.v_cmd = parse_float(fields.get("vCmd"), telemetry.v_cmd)
        telemetry.w_cmd = parse_float(fields.get("wCmd"), telemetry.w_cmd)
        telemetry.aligned = fields.get("aligned", telemetry.aligned)

        if telemetry.nav_state == "NAV_DONE":
            telemetry.message = "NAV DONE"
        elif telemetry.nav_state == "NAV_ERROR":
            telemetry.message = "NAV ERROR"

        telemetry.last_update_time = now
        return

    if line.startswith("NAV START ALIGN GOOD ENOUGH"):
        telemetry.message = "START ALIGN GOOD ENOUGH"
    elif line.startswith("NAV CRUISE START"):
        telemetry.message = "CRUISE STARTED"
    elif line.startswith("NAV CHECKPOINT"):
        tag_id = parse_int(parse_nav_fields(line).get("tag"), None)
        telemetry.message = f"CHECKPOINT TAG {tag_id if tag_id is not None else '?'}"
    elif line.startswith("NAV DONE"):
        telemetry.nav_state = "NAV_DONE"
        telemetry.message = "NAV DONE"
    elif line.startswith("NAV ERROR") or line.startswith("NAV START ALIGN UNSAFE"):
        telemetry.nav_state = "NAV_ERROR"
        telemetry.message = "NAV ERROR"
    elif line.startswith("NAV START ALIGN LOST TAG") or line.startswith(
        "NAV START ALIGN TIMEOUT UNSAFE"
    ):
        telemetry.nav_state = "NAV_ERROR"
        telemetry.message = "NAV ERROR"
    else:
        return

    telemetry.last_update_time = now


def parse_int(value: str | None, fallback: int | None) -> int | None:
    try:
        return int(value) if value is not None else fallback
    except ValueError:
        return fallback


def parse_float(value: str | None, fallback: float | None) -> float | None:
    try:
        return float(value) if value is not None else fallback
    except ValueError:
        return fallback


def format_value(value: float | int | None, precision: int = 2) -> str:
    if value is None:
        return "--"
    if isinstance(value, int):
        return str(value)
    return f"{value:.{precision}f}"


def print_esp32_lines(
    client: ESP32Client,
    telemetry: ESP32NavTelemetry,
) -> None:
    for line in client.read_lines():
        update_nav_telemetry(line, telemetry)
        print(f"ESP32 {line}")


def draw_fps_overlay(
    cv2,
    frame,
    camera_fps: float,
    detection_time_ms: float,
    pose: TagPose | None,
    tag_fps: float,
    visible_tag_fps: dict[int, float],
    nav_telemetry: ESP32NavTelemetry,
) -> None:
    visible = pose is not None
    left_lines = [
        f"Camera FPS: {camera_fps:.1f}",
        f"Detection Time: {detection_time_ms:.1f} ms",
        f"Tag Visible: {'YES' if visible else 'NO'}",
        f"Tag ID: {pose.tag_id if visible else -1}",
        f"Tag FPS: {tag_fps:.1f}",
    ]

    if visible:
        right_lines = [
            f"xM: {pose.x_m:+.4f}",
            f"yM: {pose.y_m:+.4f}",
            f"zM: {pose.z_m:+.4f}",
            f"yaw: {pose.yaw_deg:+.2f}",
        ]
    else:
        right_lines = ["xM: --", "yM: --", "zM: --", "yaw: --"]

    right_lines.extend(
        f"Tag {tag_id} FPS: {fps:.1f}"
        for tag_id, fps in sorted(visible_tag_fps.items())
    )

    for index, text in enumerate(left_lines):
        cv2.putText(
            frame,
            text,
            (20, 130 + index * 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )

    for index, text in enumerate(right_lines):
        cv2.putText(
            frame,
            text,
            (350, 130 + index * 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )

    nav_left_lines = [
        f"Nav State: {nav_telemetry.nav_state}",
        f"Aligned: {nav_telemetry.aligned}",
        f"Message: {nav_telemetry.message or '--'}",
        f"Current Tag: {format_value(nav_telemetry.current_tag)}",
        f"Expected Next: {format_value(nav_telemetry.expected_next)}",
    ]
    nav_right_lines = [
        f"IMU Heading: {format_value(nav_telemetry.imu_heading_deg)}",
        f"Target Heading: {format_value(nav_telemetry.target_heading_deg)}",
        f"Heading Error: {format_value(nav_telemetry.heading_error_deg)}",
        f"vCmd: {format_value(nav_telemetry.v_cmd, 4)}",
        f"wCmd: {format_value(nav_telemetry.w_cmd, 4)}",
    ]

    for index, text in enumerate(nav_left_lines):
        cv2.putText(
            frame,
            text,
            (20, 275 + index * 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 200, 255),
            1,
        )

    for index, text in enumerate(nav_right_lines):
        cv2.putText(
            frame,
            text,
            (350, 275 + index * 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 200, 255),
            1,
        )

    motor_left_lines = [
        f"Left Hz: {format_value(nav_telemetry.left_hz)}",
        f"Left Steps: {format_value(nav_telemetry.left_steps)}",
    ]
    motor_right_lines = [
        f"Right Hz: {format_value(nav_telemetry.right_hz)}",
        f"Right Steps: {format_value(nav_telemetry.right_steps)}",
    ]

    for index, text in enumerate(motor_left_lines):
        cv2.putText(
            frame,
            text,
            (20, 400 + index * 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 180, 0),
            1,
        )

    for index, text in enumerate(motor_right_lines):
        cv2.putText(
            frame,
            text,
            (350, 400 + index * 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 180, 0),
            1,
        )

    telemetry_age = time.monotonic() - nav_telemetry.last_update_time
    if nav_telemetry.last_update_time == 0.0 or telemetry_age > 1.0:
        cv2.putText(
            frame,
            "ESP32 Telemetry: STALE",
            (20, 445),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )

    key_legend = (
        "Keys: q quit | s stop | a align on | o align off | p pose | "
        "n nav start | x nav stop | r nav reset | v nav status"
    )
    frame_height, frame_width = frame.shape[:2]
    legend_scale = 0.45
    legend_width = cv2.getTextSize(
        key_legend,
        cv2.FONT_HERSHEY_SIMPLEX,
        legend_scale,
        1,
    )[0][0]
    if legend_width > frame_width - 20:
        legend_scale *= (frame_width - 20) / legend_width

    cv2.putText(
        frame,
        key_legend,
        (10, frame_height - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        legend_scale,
        (255, 255, 255),
        1,
    )


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
    nav_telemetry = ESP32NavTelemetry()
    camera_fps_counter = FPSCounter()
    tag_fps_counters: dict[int, FPSCounter] = {}
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
                print_esp32_lines(client, nav_telemetry)
                continue

            camera_fps = camera_fps_counter.tick()
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

            visible_tag_fps: dict[int, float] = {}
            for detected_tag in detections:
                estimated_pose = detected_tag.get("pose")
                if estimated_pose is None or not estimated_pose["valid"]:
                    continue

                tag_id = detected_tag["tag_id"]
                if tag_id not in tag_fps_counters:
                    tag_fps_counters[tag_id] = FPSCounter()
                visible_tag_fps[tag_id] = tag_fps_counters[tag_id].tick()

            detection = choose_best_detection(detections)
            now = time.monotonic()
            visible_pose = None
            selected_tag_fps = 0.0

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
                    visible_pose = pose
                    selected_tag_fps = visible_tag_fps.get(pose.tag_id, 0.0)
                    streamer.update(pose)

                    if now - last_pose_print_time >= POSE_PRINT_PERIOD_SEC:
                        print(
                            f"TAGPOSE id={pose.tag_id} "
                            f"xM={pose.x_m:.4f} yM={pose.y_m:.4f} "
                            f"zM={pose.z_m:.4f} yaw={pose.yaw_deg:.2f} "
                            f"cameraFPS={camera_fps:.1f} "
                            f"tagFPS={selected_tag_fps:.1f} "
                            f"imu={format_value(nav_telemetry.imu_heading_deg)} "
                            f"target={format_value(nav_telemetry.target_heading_deg)} "
                            f"headingError={format_value(nav_telemetry.heading_error_deg)} "
                            f"navState={nav_telemetry.nav_state} "
                            f"aligned={nav_telemetry.aligned}"
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

            draw_fps_overlay(
                cv2,
                frame,
                camera_fps,
                detector.detection_time_ms,
                visible_pose,
                selected_tag_fps,
                visible_tag_fps,
                nav_telemetry,
            )
            print_esp32_lines(client, nav_telemetry)
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
            elif key == ord("n"):
                client.nav_start()
                print("SENT: NAV START")
            elif key == ord("x"):
                client.nav_stop()
                print("SENT: NAV STOP")
            elif key == ord("r"):
                client.nav_reset()
                print("SENT: NAV RESET")
            elif key == ord("v"):
                client.nav_status()
                print("SENT: NAV STATUS")
            elif key == ord("h"):
                client.status()
                print("SENT: STATUS")
    except KeyboardInterrupt:
        pass
    finally:
        if client.connected:
            client.stop()
            time.sleep(0.1)
            print_esp32_lines(client, nav_telemetry)
        client.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
