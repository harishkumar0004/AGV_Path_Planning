from __future__ import annotations

import argparse
import csv
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

    from communication.imu_serial_reader import ImuSerialReader
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


def wrap_angle_deg(angle_deg: float) -> float:
    """
    Wrap an angle to -180..180 degrees.

    Args:
        angle_deg: Input angle in degrees.

    Returns:
        Wrapped angle in degrees.
    """
    while angle_deg > 180.0:
        angle_deg -= 360.0

    while angle_deg < -180.0:
        angle_deg += 360.0

    return angle_deg


class HeadingLockLogger:
    """Writes heading-lock validation samples to CSV."""

    def __init__(self, csv_path: Path) -> None:
        """
        Create a CSV logger.

        Args:
            csv_path: Output CSV path.
        """
        self.csv_path = csv_path
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self.csv_path.open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "timestamp_sec",
                    "reference_heading_deg",
                    "current_heading_deg",
                    "heading_error_deg",
                    "command",
                    "state",
                ]
            )

    def write(
        self,
        timestamp_sec: float,
        reference_heading_deg: float | None,
        current_heading_deg: float | None,
        heading_error_deg: float | None,
        command: str,
        state: str,
    ) -> None:
        """
        Append one heading-lock sample.

        Args:
            timestamp_sec: Seconds since forward motion started.
            reference_heading_deg: Stored heading reference.
            current_heading_deg: Latest IMU heading.
            heading_error_deg: Current heading error.
            command: Current correction command.
            state: Current heading-lock state.
        """
        with self.csv_path.open("a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    f"{timestamp_sec:.2f}",
                    format_optional(reference_heading_deg),
                    format_optional(current_heading_deg),
                    format_optional(heading_error_deg),
                    command,
                    state,
                ]
            )


def format_optional(value: float | None) -> str:
    """Format optional float values for display and CSV."""
    if value is None:
        return ""

    return f"{value:.2f}"


def choose_orientation_state(
    orientation_deg: float | None,
    start_threshold_deg: float,
    stop_tolerance_deg: float,
    current_state: str,
) -> str:
    """
    Choose orientation alignment state before heading lock starts.

    Args:
        orientation_deg: Measured AprilTag orientation.
        start_threshold_deg: Threshold to start continuous correction.
        stop_tolerance_deg: Tolerance to stop orientation correction.
        current_state: Current orientation alignment state.

    Returns:
        WAITING, ALIGNING_LEFT, ALIGNING_RIGHT, ALIGNED, or TAG_LOST.
    """
    if orientation_deg is None:
        return "TAG_LOST"

    if abs(orientation_deg) <= stop_tolerance_deg:
        return "ALIGNED"

    if current_state == "ALIGNING_LEFT" and orientation_deg > stop_tolerance_deg:
        return "ALIGNING_LEFT"

    if current_state == "ALIGNING_RIGHT" and orientation_deg < -stop_tolerance_deg:
        return "ALIGNING_RIGHT"

    if orientation_deg > start_threshold_deg:
        return "ALIGNING_LEFT"

    if orientation_deg < -start_threshold_deg:
        return "ALIGNING_RIGHT"

    return current_state


def command_for_orientation_state(state: str) -> str:
    """
    Convert orientation alignment state to ESP32 validation command.

    Args:
        state: Orientation alignment state.

    Returns:
        Serial command text or NONE.
    """
    if state == "ALIGNING_LEFT":
        return "START_LEFT_CORRECTION"

    if state == "ALIGNING_RIGHT":
        return "START_RIGHT_CORRECTION"

    if state == "ALIGNED":
        return "STOP_CORRECTION"

    return "NONE"


def choose_heading_lock_state(
    heading_error_deg: float | None,
    start_threshold_deg: float,
    stop_tolerance_deg: float,
    current_state: str,
) -> str:
    """
    Choose heading lock correction state using hysteresis.

    Args:
        heading_error_deg: Current heading error.
        start_threshold_deg: Error threshold that starts correction.
        stop_tolerance_deg: Error threshold that stops correction.
        current_state: Current heading-lock state.

    Returns:
        HEADING_LOCKED, CORRECTING_LEFT, or CORRECTING_RIGHT.
    """
    if heading_error_deg is None:
        return current_state

    if abs(heading_error_deg) <= stop_tolerance_deg:
        return "HEADING_LOCKED"

    if current_state == "CORRECTING_LEFT" and heading_error_deg > stop_tolerance_deg:
        return "CORRECTING_LEFT"

    if current_state == "CORRECTING_RIGHT" and heading_error_deg < -stop_tolerance_deg:
        return "CORRECTING_RIGHT"

    if heading_error_deg > start_threshold_deg:
        return "CORRECTING_LEFT"

    if heading_error_deg < -start_threshold_deg:
        return "CORRECTING_RIGHT"

    return current_state


def command_for_heading_state(state: str) -> str:
    """
    Convert heading-lock state to ESP32 correction command.

    Args:
        state: Heading-lock state.

    Returns:
        Serial command text.
    """
    if state == "CORRECTING_LEFT":
        return "START_LEFT_CORRECTION"

    if state == "CORRECTING_RIGHT":
        return "START_RIGHT_CORRECTION"

    return "STOP_CORRECTION"


def send_if_changed(
    serial_controller: SerialMotorController,
    command: str,
    last_sent_command: str,
) -> str:
    """
    Send a raw serial command only when it changes.

    Args:
        serial_controller: ESP32 serial controller.
        command: Desired command.
        last_sent_command: Last command transmitted.

    Returns:
        Updated last sent command.
    """
    if command == "NONE" or command == last_sent_command:
        return last_sent_command

    serial_controller.send_raw_command(command, force=True)
    return command


def draw_overlay(
    frame,
    orientation_deg: float | None,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    current_command: str,
    state: str,
    moving_time_sec: float,
) -> None:
    """
    Draw heading-lock validation values on the camera image.

    Args:
        frame: Camera frame.
        orientation_deg: AprilTag orientation during setup.
        reference_heading_deg: Stored heading reference.
        current_heading_deg: Latest IMU heading.
        heading_error_deg: Current heading error.
        current_command: Current serial command.
        state: Current validation state.
        moving_time_sec: Seconds since START_FORWARD.
    """
    orientation_text = format_display(orientation_deg, suffix=" deg", signed=True)
    reference_text = format_display(reference_heading_deg, suffix=" deg")
    current_text = format_display(current_heading_deg, suffix=" deg")
    error_text = format_display(heading_error_deg, suffix=" deg", signed=True)

    lines = [
        f"Orientation: {orientation_text}",
        f"Reference Heading: {reference_text}",
        f"Current Heading: {current_text}",
        f"Heading Error: {error_text}",
        f"Current Correction Command: {current_command}",
        f"Current State: {state}",
        f"Distance Time Moving: {moving_time_sec:.2f} s",
        "A: START   S: STOP   Q: QUIT",
    ]

    y = 32
    for line in lines:
        cv2.putText(
            frame,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (255, 255, 255),
            2,
        )
        y += 34


def format_display(
    value: float | None,
    suffix: str = "",
    signed: bool = False,
) -> str:
    """Format optional float for overlay display."""
    if value is None:
        return "None"

    if signed:
        return f"{value:+.2f}{suffix}"

    return f"{value:.2f}{suffix}"


def run_validation(args: argparse.Namespace) -> None:
    """Run validation-only IMU heading lock while moving forward."""
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )
    imu_reader = ImuSerialReader(
        port=args.port,
        baudrate=args.baudrate,
        timeout=0.0,
    )
    logger = HeadingLockLogger(Path(args.csv).expanduser().resolve())

    phase = "WAITING"
    heading_state = "HEADING_LOCKED"
    orientation_state = "WAITING"
    current_command = "NONE"
    last_sent_command = "NONE"
    reference_heading_deg: float | None = None
    moving_start_time: float | None = None
    last_log_time = 0.0
    last_csv_time = 0.0
    calibration_sent = False

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    print("Heading lock validation started.")
    print("Press A to align orientation, recalibrate IMU, and start forward motion.")

    try:
        while True:
            detections = perception_manager.update()
            frame = perception_manager.last_frame
            detection = detections[0] if detections else None
            orientation_deg = calculate_tag_orientation_deg(detection)

            imu_reader.update_from_connection(serial_controller.serial_connection)
            current_heading_deg = imu_reader.get_heading()
            heading_error_deg: float | None = None

            now = time.monotonic()
            moving_time_sec = (
                now - moving_start_time
                if moving_start_time is not None
                else 0.0
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("a"), ord("A")):
                phase = "ORIENTATION_ALIGNING"
                orientation_state = "WAITING"
                heading_state = "HEADING_LOCKED"
                reference_heading_deg = None
                moving_start_time = None
                calibration_sent = False
                current_command = "NONE"
                last_sent_command = "NONE"
                print("Heading lock sequence started.")

            if key in (ord("s"), ord("S")):
                phase = "STOPPED"
                heading_state = "HEADING_LOCKED"
                orientation_state = "WAITING"
                current_command = "STOP"
                last_sent_command = "STOP"
                serial_controller.send_raw_command("STOP_CORRECTION", force=True)
                serial_controller.send_raw_command("STOP", force=True)
                print("Heading lock validation stopped.")

            if phase == "ORIENTATION_ALIGNING":
                next_orientation_state = choose_orientation_state(
                    orientation_deg,
                    args.orientation_start_threshold,
                    args.orientation_tolerance,
                    orientation_state,
                )

                if next_orientation_state == "TAG_LOST":
                    current_command = "STOP_CORRECTION"
                    last_sent_command = send_if_changed(
                        serial_controller,
                        current_command,
                        last_sent_command,
                    )
                else:
                    orientation_state = next_orientation_state
                    orientation_command = command_for_orientation_state(
                        orientation_state,
                    )
                    last_sent_command = send_if_changed(
                        serial_controller,
                        orientation_command,
                        last_sent_command,
                    )
                    if orientation_command != "NONE":
                        current_command = orientation_command

                    if orientation_state == "ALIGNED":
                        phase = "CALIBRATING_IMU"
                        current_command = "CALIBRATE_IMU"
                        serial_controller.send_raw_command(
                            "CALIBRATE_IMU",
                            force=True,
                        )
                        last_sent_command = "CALIBRATE_IMU"
                        calibration_sent = True

            elif phase == "CALIBRATING_IMU":
                if (
                    calibration_sent
                    and imu_reader.latest_status == "CONNECTED"
                    and current_heading_deg is not None
                ):
                    reference_heading_deg = current_heading_deg
                    phase = "MOVING"
                    heading_state = "HEADING_LOCKED"
                    current_command = "START_FORWARD"
                    serial_controller.send_raw_command("START_FORWARD", force=True)
                    last_sent_command = "START_FORWARD"
                    moving_start_time = now
                    logger.write(
                        0.0,
                        reference_heading_deg,
                        current_heading_deg,
                        0.0,
                        current_command,
                        heading_state,
                    )
                    print(
                        "Heading reference captured: "
                        f"{reference_heading_deg:.2f} deg"
                    )

            elif phase == "MOVING":
                if reference_heading_deg is not None and current_heading_deg is not None:
                    heading_error_deg = wrap_angle_deg(
                        current_heading_deg - reference_heading_deg,
                    )
                    next_heading_state = choose_heading_lock_state(
                        heading_error_deg,
                        args.heading_start_threshold,
                        args.heading_stop_tolerance,
                        heading_state,
                    )
                    heading_state = next_heading_state
                    current_command = command_for_heading_state(heading_state)
                    last_sent_command = send_if_changed(
                        serial_controller,
                        current_command,
                        last_sent_command,
                    )

                if (now - last_csv_time) >= args.csv_interval:
                    logger.write(
                        moving_time_sec,
                        reference_heading_deg,
                        current_heading_deg,
                        heading_error_deg,
                        current_command,
                        heading_state,
                    )
                    last_csv_time = now

            if (now - last_log_time) >= args.log_interval:
                print("Ref Heading:", format_display(reference_heading_deg, " deg"))
                print("Current Heading:", format_display(current_heading_deg, " deg"))
                print("Heading Error:", format_display(heading_error_deg, " deg", signed=True))
                print("Command:", current_command)
                print("State:", heading_state if phase == "MOVING" else phase)
                print()
                last_log_time = now

            if frame is not None:
                draw_overlay(
                    frame,
                    orientation_deg,
                    reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    current_command,
                    heading_state if phase == "MOVING" else phase,
                    moving_time_sec,
                )
                cv2.imshow("Heading Lock Validation", frame)

            if key in (ord("q"), ord("Q")):
                break
    except KeyboardInterrupt:
        print("Stopping heading lock validation.")
    finally:
        serial_controller.send_raw_command("STOP_CORRECTION", force=True)
        serial_controller.send_raw_command("STOP", force=True)
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse heading lock validation options."""
    parser = argparse.ArgumentParser(
        description="Validation-only IMU heading lock while moving forward.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--orientation-start-threshold",
        type=float,
        default=5.0,
        help="Start orientation correction above this tag angle.",
    )
    parser.add_argument(
        "--orientation-tolerance",
        type=float,
        default=2.0,
        help="Finish orientation alignment within this angle.",
    )
    parser.add_argument(
        "--heading-start-threshold",
        type=float,
        default=2.0,
        help="Start heading correction above this heading error.",
    )
    parser.add_argument(
        "--heading-stop-tolerance",
        type=float,
        default=1.0,
        help="Stop heading correction within this heading error.",
    )
    parser.add_argument(
        "--csv",
        default="validation/heading_lock_log.csv",
        help="CSV file for heading lock samples.",
    )
    parser.add_argument(
        "--csv-interval",
        type=float,
        default=0.1,
        help="Seconds between CSV samples while moving.",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=0.5,
        help="Seconds between terminal logs.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    if any(argument in ("-h", "--help") for argument in sys.argv[1:]):
        parse_args()
        raise SystemExit(0)

    if DEPENDENCY_ERROR is not None:
        print(
            "Heading lock validation cannot start because a required "
            f"dependency is missing: {DEPENDENCY_ERROR.name}"
        )
        print(
            "Run this validation in the Raspberry Pi environment containing "
            "OpenCV, pyserial, Picamera2, and pupil_apriltags."
        )
        raise SystemExit(1)

    run_validation(parse_args())
