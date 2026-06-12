from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
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


@dataclass
class TagMeasurement:
    """Stores camera-side AprilTag measurements for display and logging."""

    tag_id: int | None
    tag_center_x: float | None
    tag_center_y: float | None
    image_center_x: float | None
    image_center_y: float | None
    position_error_x: float | None
    orientation_deg: float | None


@dataclass
class StartGateStatus:
    """Stores validation-only start gate condition status."""

    orientation_pass: bool
    position_pass: bool
    heading_pass: bool
    stable_time_sec: float
    status_text: str


class HeadingDriftLogger:
    """Logs natural heading drift while the AGV moves forward."""

    def __init__(self, csv_path: Path) -> None:
        """
        Create a fresh heading drift CSV log.

        Args:
            csv_path: Output CSV path.
        """
        self.csv_path = csv_path
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.heading_errors: list[float] = []

        with self.csv_path.open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "timestamp_sec",
                    "reference_heading_deg",
                    "current_heading_deg",
                    "heading_error_deg",
                    "tag_orientation_deg",
                    "tag_center_x",
                    "tag_center_y",
                    "position_error_x",
                ]
            )

    def write(
        self,
        timestamp_sec: float,
        reference_heading_deg: float | None,
        current_heading_deg: float | None,
        heading_error_deg: float | None,
        measurement: TagMeasurement,
    ) -> None:
        """
        Append one drift measurement row.

        Args:
            timestamp_sec: Moving time in seconds.
            reference_heading_deg: Stored IMU reference heading.
            current_heading_deg: Latest IMU heading.
            heading_error_deg: Current heading drift.
            measurement: Latest AprilTag camera measurement.
        """
        if heading_error_deg is not None:
            self.heading_errors.append(heading_error_deg)

        with self.csv_path.open("a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    format_csv(timestamp_sec),
                    format_csv(reference_heading_deg),
                    format_csv(current_heading_deg),
                    format_csv(heading_error_deg),
                    format_csv(measurement.orientation_deg),
                    format_csv(measurement.tag_center_x),
                    format_csv(measurement.tag_center_y),
                    format_csv(measurement.position_error_x),
                ]
            )

    def print_summary(self) -> None:
        """Print drift statistics for the completed run."""
        print("Heading Drift Summary")
        if not self.heading_errors:
            print("No heading error samples recorded.")
            return

        absolute_errors = [abs(error) for error in self.heading_errors]
        maximum_error = max(absolute_errors)
        average_error = sum(absolute_errors) / len(absolute_errors)
        final_error = self.heading_errors[-1]

        print(f"Maximum Heading Error: {maximum_error:.2f} deg")
        print(f"Average Heading Error: {average_error:.2f} deg")
        print(f"Final Heading Error: {final_error:+.2f} deg")


def calculate_tag_orientation_deg(detection: dict[str, Any] | None) -> float | None:
    """
    Calculate AprilTag orientation from corner[0] to corner[1].

    Args:
        detection: AprilTag detection dictionary, or None.

    Returns:
        Tag orientation normalized to -90..+90 degrees.
    """
    if detection is None:
        return None

    corners = detection.get("corners", [])
    if len(corners) < 2:
        return None

    x0, y0 = corners[0]
    x1, y1 = corners[1]
    raw_angle_deg = math.degrees(math.atan2(y1 - y0, x1 - x0))

    return ((raw_angle_deg + 90.0) % 180.0) - 90.0


def measure_tag(frame, detection: dict[str, Any] | None) -> TagMeasurement:
    """
    Build one tag measurement from the current frame and detection.

    Args:
        frame: Latest camera frame.
        detection: First visible AprilTag detection.

    Returns:
        TagMeasurement for overlay and CSV.
    """
    if frame is None:
        return TagMeasurement(None, None, None, None, None, None, None)

    image_height, image_width = frame.shape[:2]
    image_center_x = image_width / 2
    image_center_y = image_height / 2

    if detection is None:
        return TagMeasurement(
            tag_id=None,
            tag_center_x=None,
            tag_center_y=None,
            image_center_x=image_center_x,
            image_center_y=image_center_y,
            position_error_x=None,
            orientation_deg=None,
        )

    tag_center_x = float(detection["center_x"])
    tag_center_y = float(detection["center_y"])

    return TagMeasurement(
        tag_id=int(detection["tag_id"]),
        tag_center_x=tag_center_x,
        tag_center_y=tag_center_y,
        image_center_x=image_center_x,
        image_center_y=image_center_y,
        position_error_x=tag_center_x - image_center_x,
        orientation_deg=calculate_tag_orientation_deg(detection),
    )


def wrap_angle_deg(angle_deg: float) -> float:
    """
    Wrap an angle to -180..180 degrees.

    Args:
        angle_deg: Input angle.

    Returns:
        Wrapped angle.
    """
    while angle_deg > 180.0:
        angle_deg -= 360.0

    while angle_deg < -180.0:
        angle_deg += 360.0

    return angle_deg


def choose_orientation_state(
    orientation_deg: float | None,
    start_threshold_deg: float,
    stop_tolerance_deg: float,
    current_state: str,
) -> str:
    """
    Choose orientation-alignment state before drift measurement starts.

    Args:
        orientation_deg: Current AprilTag orientation.
        start_threshold_deg: Angle that starts continuous correction.
        stop_tolerance_deg: Angle that completes alignment.
        current_state: Current orientation state.

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
    Convert orientation-alignment state to validation serial command.

    Args:
        state: Orientation-alignment state.

    Returns:
        START_LEFT_CORRECTION, START_RIGHT_CORRECTION, STOP_CORRECTION, or NONE.
    """
    if state == "ALIGNING_LEFT":
        return "START_LEFT_CORRECTION"

    if state == "ALIGNING_RIGHT":
        return "START_RIGHT_CORRECTION"

    if state == "ALIGNED":
        return "STOP_CORRECTION"

    return "NONE"


def send_if_changed(
    serial_controller: SerialMotorController,
    command: str,
    last_sent_command: str,
) -> str:
    """
    Send a raw command only when it changes.

    Args:
        serial_controller: ESP32 serial controller.
        command: Desired command.
        last_sent_command: Last transmitted command.

    Returns:
        Updated last transmitted command.
    """
    if command == "NONE" or command == last_sent_command:
        return last_sent_command

    serial_controller.send_raw_command(command, force=True)
    return command


def evaluate_start_gate(
    measurement: TagMeasurement,
    heading_error_deg: float | None,
    stable_since: float | None,
    now: float,
    stable_duration_sec: float,
) -> tuple[StartGateStatus, float | None]:
    """
    Check whether all start conditions are stable before forward motion.

    Args:
        measurement: Latest AprilTag camera measurement.
        heading_error_deg: Current IMU heading error from the reference.
        stable_since: Time when all gate conditions first became true.
        now: Current monotonic time.
        stable_duration_sec: Required continuous stable duration.

    Returns:
        Start gate status and updated stable start time.
    """
    orientation_pass = (
        measurement.orientation_deg is not None
        and abs(measurement.orientation_deg) <= 2.0
    )
    position_pass = (
        measurement.position_error_x is not None
        and abs(measurement.position_error_x) <= 10.0
    )
    heading_pass = heading_error_deg is not None and abs(heading_error_deg) <= 0.5

    all_conditions_pass = orientation_pass and position_pass and heading_pass

    if all_conditions_pass:
        if stable_since is None:
            stable_since = now

        stable_time_sec = now - stable_since
        if stable_time_sec >= stable_duration_sec:
            status_text = "READY_TO_MOVE"
        else:
            stable_ms = int(stable_time_sec * 1000)
            status_text = f"STABLE_{stable_ms}ms"
    else:
        stable_since = None
        stable_time_sec = 0.0
        status_text = "WAITING_FOR_STABLE_ALIGNMENT"

    return (
        StartGateStatus(
            orientation_pass=orientation_pass,
            position_pass=position_pass,
            heading_pass=heading_pass,
            stable_time_sec=stable_time_sec,
            status_text=status_text,
        ),
        stable_since,
    )


def draw_visualization(
    frame,
    detection: dict[str, Any] | None,
    measurement: TagMeasurement,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    moving_time_sec: float,
    phase: str,
    gate_status: StartGateStatus | None,
) -> None:
    """
    Draw full AprilTag and heading drift diagnostics.

    Args:
        frame: Camera frame.
        detection: First visible AprilTag detection.
        measurement: Latest tag measurements.
        reference_heading_deg: Stored IMU reference heading.
        current_heading_deg: Latest IMU heading.
        heading_error_deg: Current IMU heading error.
        moving_time_sec: Time spent moving forward.
        phase: Current validation phase.
        gate_status: Latest start gate condition status.
    """
    image_height, image_width = frame.shape[:2]
    image_center_x = int(image_width / 2)
    image_center_y = int(image_height / 2)

    cv2.line(frame, (image_center_x, 0), (image_center_x, image_height), (0, 255, 255), 1)
    cv2.line(frame, (0, image_center_y), (image_width, image_center_y), (0, 255, 255), 1)
    cv2.drawMarker(
        frame,
        (image_center_x, image_center_y),
        (0, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=22,
        thickness=2,
    )

    if detection is not None:
        corners = [
            (int(corner_x), int(corner_y))
            for corner_x, corner_y in detection["corners"]
        ]

        for index, corner in enumerate(corners):
            cv2.line(frame, corner, corners[(index + 1) % len(corners)], (0, 255, 0), 2)

        if len(corners) >= 2:
            cv2.line(frame, corners[0], corners[1], (255, 0, 255), 3)

        if measurement.tag_center_x is not None and measurement.tag_center_y is not None:
            tag_center = (
                int(measurement.tag_center_x),
                int(measurement.tag_center_y),
            )
            cv2.circle(frame, tag_center, 6, (0, 0, 255), -1)

    orientation_color = get_orientation_color(measurement.orientation_deg)

    text_lines = [
        ("Tag ID", format_display(measurement.tag_id)),
        ("Tag Center X", format_display(measurement.tag_center_x, decimals=0)),
        ("Tag Center Y", format_display(measurement.tag_center_y, decimals=0)),
        ("Image Center X", format_display(measurement.image_center_x, decimals=0)),
        ("Image Center Y", format_display(measurement.image_center_y, decimals=0)),
        ("Position Error X", format_display(measurement.position_error_x, decimals=0, signed=True)),
        ("Tag Orientation", format_display(measurement.orientation_deg, suffix=" deg", signed=True)),
        ("Reference Heading", format_display(reference_heading_deg, suffix=" deg", signed=True)),
        ("Current Heading", format_display(current_heading_deg, suffix=" deg", signed=True)),
        ("Heading Error", format_display(heading_error_deg, suffix=" deg", signed=True)),
        ("Moving Time", f"{moving_time_sec:.2f} s"),
        ("Phase", phase),
    ]

    y = 28
    for label, value in text_lines:
        color = orientation_color if label == "Tag Orientation" else (255, 255, 255)
        cv2.putText(
            frame,
            f"{label}: {value}",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            color,
            2,
        )
        y += 28

    if gate_status is not None:
        gate_lines = [
            ("Orientation", pass_fail_text(gate_status.orientation_pass), pass_fail_color(gate_status.orientation_pass)),
            ("Position", pass_fail_text(gate_status.position_pass), pass_fail_color(gate_status.position_pass)),
            ("Heading", pass_fail_text(gate_status.heading_pass), pass_fail_color(gate_status.heading_pass)),
            ("Stable Time", f"{gate_status.stable_time_sec:.2f} s", (255, 255, 255)),
            ("Gate Status", gate_status.status_text, pass_fail_color(gate_status.status_text == "READY_TO_MOVE")),
        ]

        for label, value, color in gate_lines:
            cv2.putText(
                frame,
                f"{label}: {value}",
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                color,
                2,
            )
            y += 28

    cv2.putText(
        frame,
        "Keys: A: START   S: STOP   Q: QUIT",
        (20, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        2,
    )


def pass_fail_text(condition_passed: bool) -> str:
    """Return PASS or FAIL for a boolean gate condition."""
    return "PASS" if condition_passed else "FAIL"


def pass_fail_color(condition_passed: bool) -> tuple[int, int, int]:
    """Return green for PASS and red for FAIL."""
    return (0, 255, 0) if condition_passed else (0, 0, 255)


def get_orientation_color(orientation_deg: float | None) -> tuple[int, int, int]:
    """
    Return overlay color for orientation severity.

    Args:
        orientation_deg: AprilTag orientation angle.

    Returns:
        BGR color tuple.
    """
    if orientation_deg is None:
        return (255, 255, 255)

    abs_orientation = abs(orientation_deg)
    if abs_orientation <= 2.0:
        return (0, 255, 0)

    if abs_orientation <= 5.0:
        return (0, 255, 255)

    return (0, 0, 255)


def format_display(
    value: float | int | None,
    suffix: str = "",
    decimals: int = 2,
    signed: bool = False,
) -> str:
    """
    Format optional values for display.

    Args:
        value: Numeric value or None.
        suffix: Text appended after the number.
        decimals: Decimal places.
        signed: True to include + sign for positive numbers.

    Returns:
        Display text.
    """
    if value is None:
        return "None"

    if isinstance(value, int):
        return str(value)

    sign = "+" if signed else ""
    return f"{value:{sign}.{decimals}f}{suffix}"


def format_csv(value: float | None) -> str:
    """Format optional float for CSV output."""
    if value is None:
        return ""

    return f"{value:.2f}"


def log_terminal(
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    measurement: TagMeasurement,
    moving_time_sec: float,
    phase: str,
    gate_status: StartGateStatus | None,
) -> None:
    """Print one terminal diagnostic block."""
    print("Reference Heading:", format_display(reference_heading_deg, " deg"))
    print("Current Heading:", format_display(current_heading_deg, " deg"))
    print("Heading Error:", format_display(heading_error_deg, " deg", signed=True))
    print("Tag Orientation:", format_display(measurement.orientation_deg, " deg", signed=True))
    print("Position Error X:", format_display(measurement.position_error_x, " px", decimals=0, signed=True))
    if gate_status is not None:
        print(f"Stable Time: {gate_status.stable_time_sec:.2f} s")
        print("Gate Status:")
        print(gate_status.status_text)
    print(f"Moving Time: {moving_time_sec:.2f} s")
    print("Phase:", phase)
    print()


def run_validation(args: argparse.Namespace) -> None:
    """Run heading drift measurement with full AprilTag visualization."""
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
    logger = HeadingDriftLogger(Path(args.csv).expanduser().resolve())

    phase = "WAITING"
    orientation_state = "WAITING"
    last_sent_command = "NONE"
    reference_heading_deg: float | None = None
    moving_start_time: float | None = None
    last_log_time = 0.0
    last_csv_time = 0.0
    calibration_sent = False
    summary_printed = False
    gate_stable_since: float | None = None
    gate_status: StartGateStatus | None = None

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    print("Heading drift measurement started.")
    print("Press A to align, recalibrate IMU, and start forward measurement.")

    try:
        while True:
            detections = perception_manager.update()
            frame = perception_manager.last_frame
            detection = detections[0] if detections else None
            measurement = measure_tag(frame, detection)

            imu_reader.update_from_connection(serial_controller.serial_connection)
            current_heading_deg = imu_reader.get_heading()

            now = time.monotonic()
            moving_time_sec = (
                now - moving_start_time
                if moving_start_time is not None
                else 0.0
            )
            heading_error_deg = (
                wrap_angle_deg(current_heading_deg - reference_heading_deg)
                if current_heading_deg is not None and reference_heading_deg is not None
                else None
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("a"), ord("A")):
                phase = "ORIENTATION_ALIGNING"
                orientation_state = "WAITING"
                reference_heading_deg = None
                moving_start_time = None
                calibration_sent = False
                last_sent_command = "NONE"
                imu_reader.latest_heading_deg = None
                imu_reader.latest_status = "IMU_CALIBRATING"
                last_csv_time = 0.0
                gate_stable_since = None
                gate_status = None
                summary_printed = False
                print("Measurement sequence started.")

            if key in (ord("s"), ord("S")):
                serial_controller.send_raw_command("STOP", force=True)
                phase = "STOPPED"
                print("Forward measurement stopped.")
                if not summary_printed:
                    logger.print_summary()
                    summary_printed = True

            if phase == "ORIENTATION_ALIGNING":
                next_orientation_state = choose_orientation_state(
                    measurement.orientation_deg,
                    args.orientation_start_threshold,
                    args.orientation_tolerance,
                    orientation_state,
                )

                if next_orientation_state == "TAG_LOST":
                    command = "STOP_CORRECTION"
                    last_sent_command = send_if_changed(
                        serial_controller,
                        command,
                        last_sent_command,
                    )
                else:
                    orientation_state = next_orientation_state
                    command = command_for_orientation_state(orientation_state)
                    last_sent_command = send_if_changed(
                        serial_controller,
                        command,
                        last_sent_command,
                    )

                    if orientation_state == "ALIGNED":
                        phase = "CALIBRATING_IMU"
                        serial_controller.send_raw_command("CALIBRATE_IMU", force=True)
                        last_sent_command = "CALIBRATE_IMU"
                        calibration_sent = True
                        imu_reader.latest_heading_deg = None
                        imu_reader.latest_status = "IMU_CALIBRATING"

            elif phase == "CALIBRATING_IMU":
                if calibration_sent and current_heading_deg is not None:
                    reference_heading_deg = current_heading_deg
                    phase = "START_GATE"
                    gate_stable_since = None
                    gate_status = None
                    print(
                        "Reference heading captured: "
                        f"{reference_heading_deg:+.2f} deg"
                    )
                    print("Entering START_GATE.")

            elif phase == "START_GATE":
                gate_status, gate_stable_since = evaluate_start_gate(
                    measurement,
                    heading_error_deg,
                    gate_stable_since,
                    now,
                    args.start_gate_duration,
                )

                if gate_status.status_text == "READY_TO_MOVE":
                    print("START GATE PASSED")
                    print("Orientation Error:", format_display(measurement.orientation_deg, " deg", signed=True))
                    print("Position Error:", format_display(measurement.position_error_x, " px", decimals=0, signed=True))
                    print("Heading Error:", format_display(heading_error_deg, " deg", signed=True))
                    print(f"Stable Time: {gate_status.stable_time_sec:.2f} s")
                    print("START_FORWARD")

                    phase = "MOVING_MEASUREMENT"
                    serial_controller.send_raw_command("START_FORWARD", force=True)
                    last_sent_command = "START_FORWARD"
                    moving_start_time = now
                    logger.write(
                        0.0,
                        reference_heading_deg,
                        current_heading_deg,
                        heading_error_deg,
                        measurement,
                    )

            elif phase == "MOVING_MEASUREMENT":
                if (now - last_csv_time) >= args.csv_interval:
                    logger.write(
                        moving_time_sec,
                        reference_heading_deg,
                        current_heading_deg,
                        heading_error_deg,
                        measurement,
                    )
                    last_csv_time = now

            if (now - last_log_time) >= args.log_interval:
                log_terminal(
                    reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    measurement,
                    moving_time_sec,
                    phase,
                    gate_status,
                )
                last_log_time = now

            if frame is not None:
                draw_visualization(
                    frame,
                    detection,
                    measurement,
                    reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    moving_time_sec,
                    phase,
                    gate_status,
                )
                cv2.imshow("Heading Drift Measurement", frame)

            if key in (ord("q"), ord("Q")):
                break
    except KeyboardInterrupt:
        print("Stopping heading drift measurement.")
    finally:
        serial_controller.send_raw_command("STOP", force=True)
        serial_controller.disconnect()
        perception_manager.release()
        if not summary_printed:
            logger.print_summary()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse heading drift measurement options."""
    parser = argparse.ArgumentParser(
        description="Validation-only natural heading drift measurement.",
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
        "--csv",
        default="validation/heading_drift_measurement.csv",
        help="CSV file for heading drift measurement samples.",
    )
    parser.add_argument(
        "--csv-interval",
        type=float,
        default=0.1,
        help="Seconds between CSV samples while moving.",
    )
    parser.add_argument(
        "--start-gate-duration",
        type=float,
        default=0.5,
        help="Seconds that all start gate conditions must remain stable.",
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
            "Heading drift measurement cannot start because a required "
            f"dependency is missing: {DEPENDENCY_ERROR.name}"
        )
        print(
            "Run this validation in the Raspberry Pi environment containing "
            "OpenCV, pyserial, Picamera2, and pupil_apriltags."
        )
        raise SystemExit(1)

    run_validation(parse_args())
