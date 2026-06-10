import argparse
import time
from typing import Any

import cv2

from communication.serial_motor_controller import SerialMotorController
from core.application_state import ApplicationState
from navigation.alignment_controller import AlignmentController
from navigation.motion_commands import MotionCommand
from perception.perception_manager import PerceptionManager


def correction_name(action: MotionCommand) -> str:
    """
    Convert an AlignmentController action into a validation command name.

    Args:
        action: Action returned by AlignmentController.

    Returns:
        LEFT, RIGHT, or STOP.
    """
    if action == MotionCommand.TURN_LEFT:
        return "LEFT"

    if action == MotionCommand.TURN_RIGHT:
        return "RIGHT"

    return "STOP"


def draw_validation_overlay(
    frame,
    detection: dict[str, Any] | None,
    error_x: float | None,
    command_name: str,
    alignment_state: str,
    tolerance_px: float,
) -> None:
    """
    Draw camera guides and alignment validation information.

    Args:
        frame: Camera image to annotate.
        detection: First visible AprilTag detection, or None.
        error_x: Horizontal tag alignment error, or None.
        command_name: Current LEFT, RIGHT, or STOP command.
        alignment_state: Current validation state text.
        tolerance_px: Allowed horizontal alignment error.
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
    cv2.drawMarker(
        frame,
        (image_center_x, image_center_y),
        (0, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=20,
        thickness=2,
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

    status_color = (
        (0, 255, 0)
        if alignment_state == "ALIGNED"
        else (0, 165, 255)
    )
    error_text = f"{error_x:.1f} px" if error_x is not None else "None"

    lines = [
        f"Image Center X: {image_center_x}",
        f"Tag Center X: {tag_center_text}",
        f"Alignment Error: {error_text}",
        f"Current Command: {command_name}",
        f"Alignment State: {alignment_state}",
        f"Tolerance: +/-{tolerance_px:g} px",
        "A: ALIGN   S: STOP   Q: QUIT",
    ]

    y = 30
    for index, line in enumerate(lines):
        color = status_color if index == 4 else (255, 255, 255)
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


def log_alignment(
    tag_center_x: float | None,
    image_center_x: float,
    error_x: float | None,
    command_name: str,
    alignment_state: str,
) -> None:
    """Print one alignment decision to the terminal."""
    tag_text = f"{tag_center_x:.1f}" if tag_center_x is not None else "None"
    error_text = f"{error_x:.1f}" if error_x is not None else "None"

    print(f"Tag Center X: {tag_text}")
    print(f"Image Center X: {image_center_x:.1f}")
    print(f"Error X: {error_text}")
    print(f"Correction Command: {command_name}")
    print(f"Alignment State: {alignment_state}")
    print()


def run_alignment_validation(args: argparse.Namespace) -> None:
    """
    Run isolated single-AprilTag AlignmentController validation.

    Press A to start alignment. The AGV is stopped first, then finite LEFT or
    RIGHT correction commands are sent only after the ESP32 reports idle.
    """
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    alignment_controller = AlignmentController(
        center_tolerance_px=args.tolerance,
    )
    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )

    alignment_active = False
    alignment_state = "WAITING_FOR_ALIGN"
    current_command = "STOP"
    previous_decision: str | None = None
    stable_decision_frames = 0
    last_status_query_time = 0.0
    motion_running: bool | None = None

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    print("Standalone AlignmentController validation started.")
    print("Place one AprilTag on the floor and press A to align.")

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
            error_x: float | None = None
            desired_command = "STOP"

            if tag_center_x is not None and frame is not None:
                result = alignment_controller.calculate(
                    image_width=frame.shape[1],
                    tag_center_x=tag_center_x,
                )
                error_x = result.error_x
                desired_command = correction_name(result.action)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("a"), ord("A")):
                alignment_active = True
                alignment_state = "STOPPING"
                current_command = "STOP"
                previous_decision = None
                stable_decision_frames = 0
                serial_controller.send_raw_command("STOP", force=True)
                motion_running = True
                print("ALIGN requested. Stopping before correction.")

            if key in (ord("s"), ord("S")):
                alignment_active = False
                alignment_state = "STOPPED"
                current_command = "STOP"
                serial_controller.send_raw_command("STOP", force=True)

            if alignment_active:
                now = time.monotonic()
                if (now - last_status_query_time) >= args.status_query_interval:
                    motion_running = serial_controller.is_motion_running()
                    last_status_query_time = now

                if motion_running is True:
                    alignment_state = "CORRECTING"
                elif motion_running is False:
                    if detection is None or error_x is None:
                        alignment_state = "TAG_LOST"
                        current_command = "STOP"
                        previous_decision = None
                        stable_decision_frames = 0
                    elif desired_command == "STOP":
                        alignment_active = False
                        alignment_state = "ALIGNED"
                        current_command = "STOP"
                        serial_controller.send_raw_command("STOP", force=True)
                        log_alignment(
                            tag_center_x,
                            image_center_x,
                            error_x,
                            current_command,
                            alignment_state,
                        )
                    else:
                        alignment_state = "ALIGNING"

                        if desired_command == previous_decision:
                            stable_decision_frames += 1
                        else:
                            previous_decision = desired_command
                            stable_decision_frames = 1

                        if stable_decision_frames >= args.stable_frames:
                            current_command = desired_command
                            serial_controller.send_raw_command(
                                f"{desired_command} {args.correction_value:g}",
                                force=True,
                            )
                            motion_running = True
                            log_alignment(
                                tag_center_x,
                                image_center_x,
                                error_x,
                                current_command,
                                alignment_state,
                            )
                            stable_decision_frames = 0

            if frame is not None:
                draw_validation_overlay(
                    frame,
                    detection,
                    error_x,
                    current_command,
                    alignment_state,
                    args.tolerance,
                )
                cv2.imshow("AlignmentController Validation", frame)

            if key in (ord("q"), ord("Q")):
                break
    except KeyboardInterrupt:
        print("Stopping alignment validation.")
    finally:
        serial_controller.send_raw_command("STOP", force=True)
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse standalone alignment validation options."""
    parser = argparse.ArgumentParser(
        description="Validate AlignmentController using one AprilTag.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--tolerance",
        type=float,
        default=10.0,
        help="Alignment tolerance in pixels.",
    )
    parser.add_argument(
        "--correction-value",
        type=float,
        default=2.0,
        help="Finite LEFT/RIGHT correction value sent to ESP32.",
    )
    parser.add_argument(
        "--stable-frames",
        type=int,
        default=3,
        help="Matching decision frames required before a correction.",
    )
    parser.add_argument(
        "--status-query-interval",
        type=float,
        default=0.1,
        help="Seconds between ESP32 motion STATUS queries.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_alignment_validation(parse_args())
