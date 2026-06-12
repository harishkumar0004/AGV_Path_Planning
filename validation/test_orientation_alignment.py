from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

DEPENDENCY_ERROR: ModuleNotFoundError | None = None

try:
    import cv2

    from communication.serial_motor_controller import SerialMotorController
    from core.application_state import ApplicationState
    from perception.perception_manager import PerceptionManager
except ModuleNotFoundError as error:
    DEPENDENCY_ERROR = error


def calculate_tag_orientation_deg(detection: dict[str, Any] | None) -> float | None:
    """
    Calculate AprilTag orientation from the top edge of its corner coordinates.

    Args:
        detection: AprilTag detection dictionary, or None.

    Returns:
        Orientation angle normalized to -90..+90 degrees, or None.
    """
    if detection is None:
        return None

    corners = detection.get("corners", [])
    if len(corners) < 2:
        return None

    x0, y0 = corners[0]
    x1, y1 = corners[1]
    dx = x1 - x0
    dy = y1 - y0

    raw_angle_deg = math.degrees(math.atan2(dy, dx))
    return ((raw_angle_deg + 90.0) % 180.0) - 90.0


def choose_orientation_command(
    orientation_deg: float | None,
    tolerance_deg: float,
    pulse_ms: int,
) -> str:
    """
    Choose a correction command using only AprilTag orientation.

    Args:
        orientation_deg: Measured tag orientation in degrees.
        tolerance_deg: Deadband around zero.
        pulse_ms: Steering pulse duration.

    Returns:
        LEFT_PULSE, RIGHT_PULSE, STOP, or TAG_LOST command text.
    """
    if orientation_deg is None:
        return "TAG_LOST"

    if orientation_deg > tolerance_deg:
        return f"LEFT_PULSE {pulse_ms}"

    if orientation_deg < -tolerance_deg:
        return f"RIGHT_PULSE {pulse_ms}"

    return "STOP"


def draw_overlay(
    frame,
    detection: dict[str, Any] | None,
    orientation_deg: float | None,
    current_command: str,
    state: str,
    tolerance_deg: float,
) -> None:
    """
    Draw orientation-only correction diagnostics.

    Args:
        frame: Camera image to annotate.
        detection: First visible AprilTag detection.
        orientation_deg: Tag orientation angle.
        current_command: Current command label.
        state: Validation state text.
        tolerance_deg: Orientation tolerance.
    """
    image_height, image_width = frame.shape[:2]
    image_center_x = image_width // 2
    image_center_y = image_height // 2

    cv2.line(frame, (image_center_x, 0), (image_center_x, image_height), (0, 255, 255), 1)
    cv2.line(frame, (0, image_center_y), (image_width, image_center_y), (0, 255, 255), 1)

    tag_id_text = "None"

    if detection is not None:
        tag_id_text = str(detection["tag_id"])
        tag_center = (
            int(detection["center_x"]),
            int(detection["center_y"]),
        )
        corners = [
            (int(corner_x), int(corner_y))
            for corner_x, corner_y in detection["corners"]
        ]

        for index, corner in enumerate(corners):
            cv2.line(frame, corner, corners[(index + 1) % len(corners)], (0, 255, 0), 2)

        if len(corners) >= 2:
            cv2.line(frame, corners[0], corners[1], (255, 0, 255), 3)

        cv2.circle(frame, tag_center, 5, (0, 0, 255), -1)

    orientation_text = (
        f"{orientation_deg:+.1f} deg"
        if orientation_deg is not None
        else "None"
    )
    state_color = (0, 255, 0) if current_command == "STOP" else (0, 165, 255)

    lines = [
        f"Tag ID: {tag_id_text}",
        f"Orientation Angle: {orientation_text}",
        f"Current Command: {current_command}",
        f"State: {state}",
        f"Tolerance: +/-{tolerance_deg:g} deg",
        "A: START   S: STOP   Q: QUIT",
    ]

    y = 30
    for index, line in enumerate(lines):
        color = state_color if index in (2, 3) else (255, 255, 255)
        cv2.putText(
            frame,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )
        y += 34


def run_validation(args: argparse.Namespace) -> None:
    """
    Run validation-only orientation correction.

    This tool ignores position error and does not use navigation or route
    managers. It sends only LEFT_PULSE, RIGHT_PULSE, and STOP commands.
    """
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )

    active = False
    state = "WAITING"
    current_command = "STOP"
    last_command_time = 0.0
    last_log_time = 0.0

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    print("Orientation-only alignment validation started.")
    print("Press A to start sending correction pulses. Press S to stop.")

    try:
        while True:
            detections = perception_manager.update()
            frame = perception_manager.last_frame
            detection = detections[0] if detections else None
            orientation_deg = calculate_tag_orientation_deg(detection)
            desired_command = choose_orientation_command(
                orientation_deg,
                args.tolerance,
                args.pulse_ms,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("a"), ord("A")):
                active = True
                state = "ACTIVE"
                print("Orientation correction enabled.")

            if key in (ord("s"), ord("S")):
                active = False
                state = "STOPPED"
                current_command = "STOP"
                serial_controller.send_raw_command("STOP", force=True)
                print("Orientation correction stopped.")

            now = time.monotonic()

            if active:
                if desired_command == "TAG_LOST":
                    state = "TAG_LOST"
                    current_command = "STOP"
                elif desired_command == "STOP":
                    state = "ALIGNED"
                    current_command = "STOP"
                    if (now - last_command_time) >= args.command_interval:
                        serial_controller.send_raw_command("STOP", force=False)
                        last_command_time = now
                elif (now - last_command_time) >= args.command_interval:
                    state = "CORRECTING"
                    current_command = desired_command
                    serial_controller.send_raw_command(desired_command, force=True)
                    last_command_time = now

                if (now - last_log_time) >= args.log_interval:
                    orientation_text = (
                        f"{orientation_deg:+.1f}"
                        if orientation_deg is not None
                        else "None"
                    )
                    print(
                        f"Orientation: {orientation_text} deg | "
                        f"Command: {current_command} | "
                        f"State: {state}"
                    )
                    last_log_time = now

            if frame is not None:
                draw_overlay(
                    frame,
                    detection,
                    orientation_deg,
                    current_command,
                    state,
                    args.tolerance,
                )
                cv2.imshow("Orientation Alignment Validation", frame)

            if key in (ord("q"), ord("Q")):
                break
    except KeyboardInterrupt:
        print("Stopping orientation alignment validation.")
    finally:
        serial_controller.send_raw_command("STOP", force=True)
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse orientation alignment validation options."""
    parser = argparse.ArgumentParser(
        description="Validation-only AprilTag orientation correction.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--tolerance",
        type=float,
        default=5.0,
        help="Orientation tolerance in degrees.",
    )
    parser.add_argument(
        "--pulse-ms",
        type=int,
        default=200,
        help="Pulse duration used for LEFT_PULSE and RIGHT_PULSE.",
    )
    parser.add_argument(
        "--command-interval",
        type=float,
        default=0.25,
        help="Minimum seconds between correction commands.",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=0.25,
        help="Seconds between terminal logs.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    if any(argument in ("-h", "--help") for argument in sys.argv[1:]):
        parse_args()
        raise SystemExit(0)

    if DEPENDENCY_ERROR is not None:
        print(
            "Orientation alignment validation cannot start because a required "
            f"dependency is missing: {DEPENDENCY_ERROR.name}"
        )
        print(
            "Run this validation in the Raspberry Pi environment containing "
            "OpenCV, pyserial, Picamera2, and pupil_apriltags."
        )
        raise SystemExit(1)

    run_validation(parse_args())
