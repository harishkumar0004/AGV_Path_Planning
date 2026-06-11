from __future__ import annotations

import argparse
import sys
import time
from typing import Any


import cv2

from communication.serial_motor_controller import SerialMotorController
from core.application_state import ApplicationState
from perception.perception_manager import PerceptionManager



def choose_correction_command(error_x: float | None, deadband_px: float,) -> str:
    """
    Choose a continuous-correction command label from horizontal error.

    Args:
        error_x: Tag center X minus image center X, or None when no tag exists.
        deadband_px: Pixel range treated as centered.

    Returns:
        FORWARD, SMALL_LEFT_CORRECTION, SMALL_RIGHT_CORRECTION, or TAG_LOST.
    """
    if error_x is None:
        return "TAG_LOST"

    if abs(error_x) < deadband_px:
        return "FORWARD"

    if error_x < -deadband_px:
        return "SMALL_LEFT_CORRECTION"

    return "SMALL_RIGHT_CORRECTION"


def draw_overlay(
    frame,
    detection: dict[str, Any] | None,
    error_x: float | None,
    correction_command: str,
    correction_state: str,
    deadband_px: float,
) -> None:
    """
    Draw visual diagnostics for continuous heading correction validation.

    Args:
        frame: Camera image to annotate.
        detection: First detected AprilTag, or None.
        error_x: Horizontal center error in pixels.
        correction_command: Current correction command label.
        correction_state: Current validation state.
        deadband_px: Center deadband in pixels.
    """
    image_height, image_width = frame.shape[:2]
    image_center_x = image_width // 2
    image_center_y = image_height // 2

    cv2.line(
        frame,
        (image_center_x, 0),
        (image_center_x, image_height),
        (0, 255, 255),
        1,
    )
    cv2.line(
        frame,
        (0, image_center_y),
        (image_width, image_center_y),
        (0, 255, 255),
        1,
    )

    tag_center_text = "None"
    if detection is not None:
        tag_center_x = int(detection["center_x"])
        tag_center_y = int(detection["center_y"])
        tag_center_text = str(tag_center_x)

        corners = [
            (int(corner_x), int(corner_y))
            for corner_x, corner_y in detection["corners"]
        ]
        for index, corner in enumerate(corners):
            cv2.line(
                frame,
                corner,
                corners[(index + 1) % len(corners)],
                (0, 255, 0),
                2,
            )

        cv2.circle(frame, (tag_center_x, tag_center_y), 5, (0, 0, 255), -1)
        cv2.line(
            frame,
            (image_center_x, image_center_y),
            (tag_center_x, tag_center_y),
            (255, 0, 255),
            2,
        )

    error_text = f"{error_x:.1f} px" if error_x is not None else "None"
    state_color = (
        (0, 255, 0)
        if correction_command == "FORWARD"
        else (0, 165, 255)
    )

    lines = [
        f"Image Center X: {image_center_x}",
        f"Tag Center X: {tag_center_text}",
        f"Error X: {error_text}",
        f"Current Correction Command: {correction_command}",
        f"Correction State: {correction_state}",
        f"Deadband: +/-{deadband_px:g} px",
        "F: START_FORWARD   S: STOP   Q: QUIT",
    ]

    y = 30
    for index, line in enumerate(lines):
        color = state_color if index in (3, 4) else (255, 255, 255)
        cv2.putText(
            frame,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
        )
        y += 30


def log_correction(
    start_time: float,
    error_x: float | None,
    correction_command: str,
) -> None:
    """
    Print one correction decision with timestamp.

    Args:
        start_time: Monotonic start time for relative timestamp.
        error_x: Horizontal error in pixels, or None.
        correction_command: Current command label.
    """
    elapsed = time.monotonic() - start_time
    error_text = f"{error_x:.1f}" if error_x is not None else "None"
    print(
        f"Time: {elapsed:.2f}s | "
        f"Error X: {error_text} | "
        f"Correction Command: {correction_command}"
    )


def maybe_send_correction(
    serial_controller: SerialMotorController,
    correction_command: str,
    transmit_corrections: bool,
) -> None:
    """
    Send validation correction command when explicitly enabled.

    Args:
        serial_controller: Serial link to ESP32.
        correction_command: Current correction label.
        transmit_corrections: True to transmit experimental correction labels.
    """
    if correction_command == "FORWARD":
        serial_controller.send_raw_command("START_FORWARD", force=False)
        return

    if correction_command == "TAG_LOST":
        return

    if transmit_corrections:
        serial_controller.send_raw_command(correction_command, force=True)


def run_continuous_correction(args: argparse.Namespace) -> None:
    """
    Run isolated continuous correction validation using one AprilTag.

    This program does not use RouteManager or NavigationStateManager. By default
    it transmits only START_FORWARD and STOP, while logging the experimental
    correction labels needed for a future steering-capable motor interface.
    """
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )
    start_time = time.monotonic()
    correction_active = False
    correction_state = "STOPPED"
    current_command = "STOP"
    previous_command: str | None = None
    stable_frames = 0
    last_log_time = 0.0
    last_send_time = 0.0

    if args.transmit_corrections:
        print(
            "WARNING: --transmit-corrections sends experimental command labels "
            "that require ESP32 support."
        )

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    print("Continuous correction validation started.")
    print("Place one AprilTag in view. Press F to begin forward motion.")

    try:
        while True:
            detections = perception_manager.update()
            frame = perception_manager.last_frame
            detection = detections[0] if detections else None
            tag_center_x = (
                float(detection["center_x"])
                if detection is not None
                else None
            )
            image_center_x = (
                frame.shape[1] / 2
                if frame is not None
                else 320.0
            )
            error_x = (
                tag_center_x - image_center_x
                if tag_center_x is not None
                else None
            )
            desired_command = choose_correction_command(
                error_x,
                args.deadband,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("f"), ord("F")):
                correction_active = True
                correction_state = "MOVING"
                current_command = "FORWARD"
                serial_controller.send_raw_command("START_FORWARD", force=True)
                print("START_FORWARD requested.")

            if key in (ord("s"), ord("S")):
                correction_active = False
                correction_state = "STOPPED"
                current_command = "STOP"
                serial_controller.send_raw_command("STOP", force=True)
                print("STOP requested.")

            if correction_active:
                if desired_command == previous_command:
                    stable_frames += 1
                else:
                    previous_command = desired_command
                    stable_frames = 1

                if stable_frames >= args.stable_frames:
                    current_command = desired_command
                    if desired_command == "TAG_LOST":
                        correction_state = "TAG_LOST"
                    elif desired_command == "FORWARD":
                        correction_state = "CENTERED"
                    else:
                        correction_state = "CORRECTING"

                    now = time.monotonic()
                    if (now - last_send_time) >= args.command_interval:
                        maybe_send_correction(
                            serial_controller,
                            current_command,
                            args.transmit_corrections,
                        )
                        last_send_time = now

                    if (now - last_log_time) >= args.log_interval:
                        log_correction(start_time, error_x, current_command)
                        last_log_time = now

            if frame is not None:
                draw_overlay(
                    frame,
                    detection,
                    error_x,
                    current_command,
                    correction_state,
                    args.deadband,
                )
                cv2.imshow("Continuous Correction Validation", frame)

            if key in (ord("q"), ord("Q")):
                break
    except KeyboardInterrupt:
        print("Stopping continuous correction validation.")
    finally:
        serial_controller.send_raw_command("STOP", force=True)
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse continuous correction validation options."""
    parser = argparse.ArgumentParser(
        description="Validate continuous AprilTag heading correction behavior.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--deadband",
        type=float,
        default=15.0,
        help="Pixel error band treated as centered.",
    )
    parser.add_argument(
        "--stable-frames",
        type=int,
        default=3,
        help="Matching decision frames required before changing command.",
    )
    parser.add_argument(
        "--command-interval",
        type=float,
        default=0.25,
        help="Minimum seconds between transmitted validation commands.",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=0.25,
        help="Minimum seconds between correction log lines.",
    )
    parser.add_argument(
        "--transmit-corrections",
        action="store_true",
        help="Transmit experimental correction labels to ESP32.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    if any(argument in ("-h", "--help") for argument in sys.argv[1:]):
        parse_args()
        raise SystemExit(0)

    print(
            "Run this validation in the Raspberry Pi environment containing "
            "OpenCV, pyserial, Picamera2, and pupil_apriltags."
        )
    raise SystemExit(1)

    run_continuous_correction(parse_args())
