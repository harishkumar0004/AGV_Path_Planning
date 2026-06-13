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


@dataclass
class AcquisitionStatus:
    """Stores continuous tag acquisition status while moving."""

    orientation_pass: bool
    position_pass: bool
    stable_time_sec: float
    status_text: str


class ContinuousNavigationLogger:
    """Logs continuous tag-to-tag heading-hold validation samples."""

    def __init__(self, csv_path: Path) -> None:
        """
        Create a fresh CSV log.

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
                    "state",
                    "tag_id",
                    "reference_heading_deg",
                    "current_heading_deg",
                    "heading_error_deg",
                    "tag_orientation_deg",
                    "position_error_x",
                    "tag_visible",
                    "tag_acquired",
                    "command",
                ]
            )

    def write(
        self,
        timestamp_sec: float,
        state: str,
        measurement: TagMeasurement,
        reference_heading_deg: float | None,
        current_heading_deg: float | None,
        heading_error_deg: float | None,
        tag_visible: bool,
        tag_acquired: bool,
        command: str,
    ) -> None:
        """
        Append one validation row.

        Args:
            timestamp_sec: Time since validation start.
            state: Current validation state.
            measurement: Latest AprilTag camera measurement.
            reference_heading_deg: Stored IMU reference heading.
            current_heading_deg: Latest IMU heading.
            heading_error_deg: Current heading error.
            tag_visible: True when a tag is visible.
            tag_acquired: True after stable vision lock on current tag.
            command: Current serial correction command.
        """
        with self.csv_path.open("a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    format_csv(timestamp_sec),
                    state,
                    measurement.tag_id if measurement.tag_id is not None else "",
                    format_csv(reference_heading_deg),
                    format_csv(current_heading_deg),
                    format_csv(heading_error_deg),
                    format_csv(measurement.orientation_deg),
                    format_csv(measurement.position_error_x),
                    "YES" if tag_visible else "NO",
                    "YES" if tag_acquired else "NO",
                    command,
                ]
            )


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


def choose_initial_orientation_state(
    orientation_deg: float | None,
    tolerance_deg: float,
) -> str:
    """
    Choose the initial stationary Tag1 orientation alignment command state.

    Args:
        orientation_deg: Current AprilTag orientation.
        tolerance_deg: Alignment tolerance in degrees.

    Returns:
        ALIGNING_LEFT, ALIGNING_RIGHT, ALIGNED, or TAG_LOST.
    """
    if orientation_deg is None:
        return "TAG_LOST"

    if abs(orientation_deg) <= tolerance_deg:
        return "ALIGNED"

    if orientation_deg > tolerance_deg:
        return "ALIGNING_RIGHT"

    return "ALIGNING_LEFT"


def command_for_alignment_state(state: str) -> str:
    """
    Convert alignment state to ESP32 validation command.

    Args:
        state: Alignment state.

    Returns:
        Serial command string or NONE.
    """
    if state == "ALIGNING_LEFT":
        return "START_LEFT_CORRECTION"

    if state == "ALIGNING_RIGHT":
        return "START_RIGHT_CORRECTION"

    if state == "ALIGNED":
        return "STOP_CORRECTION"

    return "NONE"


def command_for_heading_hold(heading_error_deg: float | None, deadband_deg: float) -> str:
    """
    Choose correction command from IMU heading error.

    Args:
        heading_error_deg: Current heading error.
        deadband_deg: Heading deadband.

    Returns:
        START_LEFT_CORRECTION, START_RIGHT_CORRECTION, STOP_CORRECTION, or NONE.
    """
    if heading_error_deg is None:
        return "NONE"

    if heading_error_deg > deadband_deg:
        return "START_LEFT_CORRECTION"

    if heading_error_deg < -deadband_deg:
        return "START_RIGHT_CORRECTION"

    return "STOP_CORRECTION"


def command_for_vision_tracking(
    measurement: TagMeasurement,
    orientation_deadband_deg: float,
) -> str:
    """
    Choose correction command from live AprilTag orientation.

    Orientation is the primary control input. Position error is displayed and
    logged, but not used for command generation in this validation phase.

    Args:
        measurement: Latest tag measurement.
        orientation_deadband_deg: Vision orientation deadband.

    Returns:
        START_LEFT_CORRECTION, START_RIGHT_CORRECTION, STOP_CORRECTION, or NONE.
    """
    if measurement.orientation_deg is None:
        return "NONE"

    if measurement.orientation_deg > orientation_deadband_deg:
        return "START_LEFT_CORRECTION"

    if measurement.orientation_deg < -orientation_deadband_deg:
        return "START_RIGHT_CORRECTION"

    return "STOP_CORRECTION"


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
    heading_pass = True

    if orientation_pass and position_pass and heading_pass:
        if stable_since is None:
            stable_since = now

        stable_time_sec = now - stable_since
        status_text = (
            "READY_TO_MOVE"
            if stable_time_sec >= stable_duration_sec
            else f"STABLE_{int(stable_time_sec * 1000)}ms"
        )
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


def evaluate_tag_acquisition(
    measurement: TagMeasurement,
    stable_since: float | None,
    now: float,
    stable_duration_sec: float,
) -> tuple[AcquisitionStatus, float | None]:
    """
    Check whether a visible tag has been stably acquired while moving.

    Args:
        measurement: Latest tag measurement.
        stable_since: Time when all acquisition conditions became true.
        now: Current monotonic time.
        stable_duration_sec: Required stable duration.

    Returns:
        Acquisition status and updated stable start time.
    """
    orientation_pass = (
        measurement.orientation_deg is not None
        and abs(measurement.orientation_deg) <= 2.0
    )
    position_pass = (
        measurement.position_error_x is not None
        and abs(measurement.position_error_x) <= 10.0
    )

    if orientation_pass and position_pass:
        if stable_since is None:
            stable_since = now

        stable_time_sec = now - stable_since
        status_text = (
            "TAG_READY"
            if stable_time_sec >= stable_duration_sec
            else f"TAG_STABLE_{int(stable_time_sec * 1000)}ms"
        )
    else:
        stable_since = None
        stable_time_sec = 0.0
        status_text = "TRACKING_TAG"

    return (
        AcquisitionStatus(
            orientation_pass=orientation_pass,
            position_pass=position_pass,
            stable_time_sec=stable_time_sec,
            status_text=status_text,
        ),
        stable_since,
    )


def log_transition(state: str) -> None:
    """
    Print a clear state transition line.

    Args:
        state: New validation state.
    """
    print(state)


def draw_visualization(
    frame,
    detection: dict[str, Any] | None,
    measurement: TagMeasurement,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    elapsed_time_sec: float,
    state: str,
    current_command: str,
    tag_visible: bool,
    tag_acquired: bool,
    gate_status: StartGateStatus | None,
    acquisition_status: AcquisitionStatus | None,
) -> None:
    """
    Draw continuous navigation diagnostics on the camera frame.

    Args:
        frame: Camera frame.
        detection: First visible AprilTag detection.
        measurement: Latest tag measurements.
        reference_heading_deg: Stored IMU reference heading.
        current_heading_deg: Latest IMU heading.
        heading_error_deg: Current IMU heading error.
        elapsed_time_sec: Time since validation start.
        state: Current validation state.
        current_command: Last command sent to ESP32.
        tag_visible: True when a tag is visible.
        tag_acquired: True after stable tag acquisition.
        gate_status: Latest start gate status.
        acquisition_status: Latest moving tag acquisition status.
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
            cv2.circle(
                frame,
                (int(measurement.tag_center_x), int(measurement.tag_center_y)),
                6,
                (0, 0, 255),
                -1,
            )

    text_lines = [
        ("State", state),
        ("Reference Heading", format_display(reference_heading_deg, suffix=" deg", signed=True)),
        ("Current Heading", format_display(current_heading_deg, suffix=" deg", signed=True)),
        ("Heading Error", format_display(heading_error_deg, suffix=" deg", signed=True)),
        ("Tag Orientation", format_display(measurement.orientation_deg, suffix=" deg", signed=True)),
        ("Position Error X", format_display(measurement.position_error_x, decimals=0, signed=True)),
        ("Tag Visible", "YES" if tag_visible else "NO"),
        ("Tag Acquired", "YES" if tag_acquired else "NO"),
        ("Current Command", current_command),
        ("Elapsed Time", f"{elapsed_time_sec:.2f} s"),
    ]

    y = 28
    for label, value in text_lines:
        color = get_orientation_color(measurement.orientation_deg) if label == "Tag Orientation" else (255, 255, 255)
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
            ("Gate Orientation", pass_fail_text(gate_status.orientation_pass), pass_fail_color(gate_status.orientation_pass)),
            ("Gate Position", pass_fail_text(gate_status.position_pass), pass_fail_color(gate_status.position_pass)),
            ("Gate Heading", pass_fail_text(gate_status.heading_pass), pass_fail_color(gate_status.heading_pass)),
            ("Gate Stable", f"{gate_status.stable_time_sec:.2f} s", (255, 255, 255)),
            ("Gate Status", gate_status.status_text, pass_fail_color(gate_status.status_text == "READY_TO_MOVE")),
        ]
        y = draw_status_lines(frame, gate_lines, y)

    if acquisition_status is not None:
        acquisition_lines = [
            ("Vision Orientation", pass_fail_text(acquisition_status.orientation_pass), pass_fail_color(acquisition_status.orientation_pass)),
            ("Vision Position", pass_fail_text(acquisition_status.position_pass), pass_fail_color(acquisition_status.position_pass)),
            ("Vision Stable", f"{acquisition_status.stable_time_sec:.2f} s", (255, 255, 255)),
            ("Vision Status", acquisition_status.status_text, pass_fail_color(acquisition_status.status_text == "TAG_READY")),
        ]
        y = draw_status_lines(frame, acquisition_lines, y)

    cv2.putText(
        frame,
        "Keys: S: STOP   Q: QUIT",
        (20, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        2,
    )


def draw_status_lines(
    frame,
    lines: list[tuple[str, str, tuple[int, int, int]]],
    y: int,
) -> int:
    """
    Draw colored status lines.

    Args:
        frame: Camera frame.
        lines: Label, value, color tuples.
        y: Starting y position.

    Returns:
        Next y position.
    """
    for label, value, color in lines:
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

    return y


def pass_fail_text(condition_passed: bool) -> str:
    """Return PASS or FAIL for a boolean condition."""
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
    state: str,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    measurement: TagMeasurement,
    tag_visible: bool,
    tag_acquired: bool,
    current_command: str,
) -> None:
    """Print one terminal diagnostic block."""
    print("State:", state)
    print("Reference Heading:", format_display(reference_heading_deg, " deg"))
    print("Current Heading:", format_display(current_heading_deg, " deg"))
    print("Heading Error:", format_display(heading_error_deg, " deg", signed=True))
    print("Tag Orientation:", format_display(measurement.orientation_deg, " deg", signed=True))
    print("Position Error:", format_display(measurement.position_error_x, " px", decimals=0, signed=True))
    print("Tag Visible:", "YES" if tag_visible else "NO")
    print("Tag Acquired:", "YES" if tag_acquired else "NO")
    print("Current Command:", current_command)
    print()


def run_validation(args: argparse.Namespace) -> None:
    """Run continuous tag-to-tag navigation with IMU heading hold."""
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
    logger = ContinuousNavigationLogger(Path(args.csv).expanduser().resolve())

    state = "WAIT_FOR_TAG1"
    current_command = "NONE"
    reference_heading_deg: float | None = None
    gate_stable_since: float | None = None
    gate_status: StartGateStatus | None = None
    acquisition_stable_since: float | None = None
    acquisition_status: AcquisitionStatus | None = None
    calibration_sent = False
    last_log_time = 0.0
    last_csv_time = 0.0
    start_time = time.monotonic()
    moving_started = False
    tag_acquired = False
    active_tag_id: int | None = None
    last_visible_tag_id: int | None = None

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    log_transition("WAIT_FOR_TAG1")

    try:
        while True:
            detections = perception_manager.update()
            frame = perception_manager.last_frame
            detection = detections[0] if detections else None
            measurement = measure_tag(frame, detection)
            tag_visible = measurement.tag_id is not None

            imu_reader.update_from_connection(serial_controller.serial_connection)
            current_heading_deg = imu_reader.get_heading()

            now = time.monotonic()
            elapsed_time_sec = now - start_time
            heading_error_deg = (
                wrap_angle_deg(current_heading_deg - reference_heading_deg)
                if current_heading_deg is not None and reference_heading_deg is not None
                else None
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("s"), ord("S")):
                current_command = "STOP"
                serial_controller.send_raw_command(current_command, force=True)
                state = "STOPPED"
                log_transition("STOPPED")

            if state == "WAIT_FOR_TAG1" and measurement.tag_id == 1:
                state = "ALIGN_TAG1"
                log_transition("ALIGN_TAG1")

            elif state == "ALIGN_TAG1":
                alignment_state = choose_initial_orientation_state(
                    measurement.orientation_deg,
                    args.orientation_tolerance,
                )
                desired_command = command_for_alignment_state(alignment_state)
                current_command = send_if_changed(
                    serial_controller,
                    desired_command,
                    current_command,
                )

                if alignment_state == "ALIGNED":
                    state = "CALIBRATE_IMU_TAG1"
                    current_command = "CALIBRATE_IMU"
                    serial_controller.send_raw_command(current_command, force=True)
                    imu_reader.latest_heading_deg = None
                    imu_reader.latest_status = "IMU_CALIBRATING"
                    reference_heading_deg = None
                    calibration_sent = True
                    gate_stable_since = None
                    gate_status = None
                    log_transition("CALIBRATE_IMU_TAG1")

            elif state == "CALIBRATE_IMU_TAG1":
                if calibration_sent and current_heading_deg is not None:
                    reference_heading_deg = None
                    state = "START_GATE_TAG1"
                    log_transition("START_GATE_TAG1")

            elif state == "START_GATE_TAG1":
                gate_status, gate_stable_since = evaluate_start_gate(
                    measurement,
                    heading_error_deg,
                    gate_stable_since,
                    now,
                    args.start_gate_duration,
                )

                if gate_status.status_text == "READY_TO_MOVE":
                    print("Gate Orientation:", pass_fail_text(gate_status.orientation_pass))
                    print("Gate Position:", pass_fail_text(gate_status.position_pass))
                    print("START GATE PASSED")
                    print("START_FORWARD")
                    current_command = "START_FORWARD"
                    serial_controller.send_raw_command(current_command, force=True)
                    reference_heading_deg = current_heading_deg
                    heading_error_deg = 0.0 if reference_heading_deg is not None else None
                    moving_started = True
                    tag_acquired = True
                    active_tag_id = 1
                    last_visible_tag_id = 1
                    state = "VISION_TRACKING"
                    log_transition("VISION_TRACKING")

            elif moving_started:
                if tag_visible:
                    if state != "VISION_TRACKING":
                        state = "VISION_TRACKING"
                        tag_acquired = False
                        acquisition_stable_since = None
                        acquisition_status = None
                        active_tag_id = measurement.tag_id
                        log_transition("VISION_TRACKING")
                    elif measurement.tag_id != active_tag_id:
                        tag_acquired = False
                        acquisition_stable_since = None
                        acquisition_status = None
                        active_tag_id = measurement.tag_id

                    last_visible_tag_id = measurement.tag_id
                    desired_command = command_for_vision_tracking(
                        measurement,
                        args.vision_orientation_deadband,
                    )
                    current_command = send_if_changed(
                        serial_controller,
                        desired_command,
                        current_command,
                    )

                    if not tag_acquired:
                        acquisition_status, acquisition_stable_since = evaluate_tag_acquisition(
                            measurement,
                            acquisition_stable_since,
                            now,
                            args.tag_acquisition_duration,
                        )

                        if (
                            acquisition_status.status_text == "TAG_READY"
                            and current_heading_deg is not None
                        ):
                            tag_acquired = True
                            reference_heading_deg = current_heading_deg
                            heading_error_deg = 0.0
                            print("TAG_ACQUIRED")
                            print(
                                "NEW_REFERENCE_HEADING:",
                                format_display(reference_heading_deg, " deg", signed=True),
                            )
                            log_transition("TAG_ACQUIRED")
                    else:
                        acquisition_status = None

                else:
                    acquisition_status = None
                    acquisition_stable_since = None
                    if state == "VISION_TRACKING":
                        print("TAG_LOST")
                        if tag_acquired:
                            print("HEADING_HOLD_RESTORED")
                        state = "HEADING_HOLD"
                        log_transition("HEADING_HOLD")

                    desired_command = command_for_heading_hold(
                        heading_error_deg,
                        args.heading_deadband,
                    )
                    current_command = send_if_changed(
                        serial_controller,
                        desired_command,
                        current_command,
                    )

            if (now - last_csv_time) >= args.csv_interval:
                logger.write(
                    elapsed_time_sec,
                    state,
                    measurement,
                    reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    tag_visible,
                    tag_acquired,
                    current_command,
                )
                last_csv_time = now

            if (now - last_log_time) >= args.log_interval:
                log_terminal(
                    state,
                    reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    measurement,
                    tag_visible,
                    tag_acquired,
                    current_command,
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
                    elapsed_time_sec,
                    state,
                    current_command,
                    tag_visible,
                    tag_acquired,
                    gate_status,
                    acquisition_status,
                )
                cv2.imshow("Continuous Heading Hold Validation", frame)

            if key in (ord("q"), ord("Q")):
                break
    except KeyboardInterrupt:
        print("Stopping continuous heading-hold validation.")
    finally:
        serial_controller.send_raw_command("STOP", force=True)
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse continuous heading-hold validation options."""
    parser = argparse.ArgumentParser(
        description="Validation-only continuous tag-to-tag navigation with heading hold.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--orientation-tolerance",
        type=float,
        default=2.0,
        help="Initial Tag1 orientation alignment tolerance.",
    )
    parser.add_argument(
        "--heading-deadband",
        type=float,
        default=0.5,
        help="IMU heading-hold deadband in degrees.",
    )
    parser.add_argument(
        "--vision-orientation-deadband",
        type=float,
        default=2.0,
        help="AprilTag orientation deadband in degrees.",
    )
    parser.add_argument(
        "--tag-acquisition-duration",
        type=float,
        default=0.5,
        help="Seconds a visible tag must be stable before updating heading reference.",
    )
    parser.add_argument(
        "--csv",
        default="validation/continuous_heading_hold_log.csv",
        help="CSV file for continuous heading-hold validation samples.",
    )
    parser.add_argument(
        "--csv-interval",
        type=float,
        default=0.1,
        help="Seconds between CSV samples.",
    )
    parser.add_argument(
        "--start-gate-duration",
        type=float,
        default=0.5,
        help="Seconds that initial start gate conditions must remain stable.",
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
            "Continuous heading-hold validation cannot start because a required "
            f"dependency is missing: {DEPENDENCY_ERROR.name}"
        )
        print(
            "Run this validation in the Raspberry Pi environment containing "
            "OpenCV, pyserial, Picamera2, and pupil_apriltags."
        )
        raise SystemExit(1)

    run_validation(parse_args())
