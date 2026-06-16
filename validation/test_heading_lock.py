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


KP_HEADING = 90.0
KI_HEADING = 10.0
KD_HEADING = 0.0

HEADING_DEADBAND_DEG = 0.2
MAX_INTEGRAL = 50.0
MAX_CORRECTION = 1000.0

BASE_FREQUENCY_HZ = 9000.0
MIN_FREQUENCY_HZ = 7000.0
MAX_FREQUENCY_HZ = 10000.0
PID_UPDATE_INTERVAL_SEC = 0.05

TRAVEL_DISTANCE_CM = 150.0

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


@dataclass
class PIDResult:
    """Stores one heading PID calculation for logging and serial output."""

    heading_error_deg: float
    kp: float
    ki: float
    kd: float
    p_term: float
    i_term: float
    d_term: float
    pid_output: float
    left_frequency_hz: float
    right_frequency_hz: float


class HeadingPIDController:
    """Converts IMU heading error into differential-drive wheel frequencies."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        max_integral: float,
        max_correction: float,
        base_frequency_hz: float,
        min_frequency_hz: float,
        max_frequency_hz: float,
    ) -> None:
        """
        Store PID gains and frequency limits.

        Args:
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            max_integral: Integral anti-windup clamp.
            max_correction: Maximum PID output magnitude.
            base_frequency_hz: Forward drive base frequency.
            min_frequency_hz: Minimum allowed wheel frequency.
            max_frequency_hz: Maximum allowed wheel frequency.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.max_correction = max_correction
        self.base_frequency_hz = base_frequency_hz
        self.min_frequency_hz = min_frequency_hz
        self.max_frequency_hz = max_frequency_hz
        self.integral = 0.0
        self.previous_error_deg: float | None = None
        self.previous_update_time: float | None = None

    def reset(self) -> None:
        """Clear stored PID state when a new heading reference is captured."""
        self.integral = 0.0
        self.previous_error_deg = None
        self.previous_update_time = None

    def update(
        self,
        reference_heading_deg: float,
        current_heading_deg: float,
        now: float,
    ) -> PIDResult:
        """
        Calculate one PID update and convert it into left/right frequencies.

        Args:
            reference_heading_deg: Desired IMU heading.
            current_heading_deg: Current IMU heading.
            now: Current monotonic timestamp.

        Returns:
            PIDResult containing terms, output, and wheel frequencies.
        """
        heading_error_deg = wrap_angle_deg(reference_heading_deg - current_heading_deg)
        if abs(heading_error_deg) < HEADING_DEADBAND_DEG:
            heading_error_deg = 0.0

        if self.previous_update_time is None:
            delta_time_sec = PID_UPDATE_INTERVAL_SEC
        else:
            delta_time_sec = max(now - self.previous_update_time, 1e-6)

        self.previous_update_time = now
        self.integral += heading_error_deg * delta_time_sec
        self.integral = clamp_float(
            self.integral,
            -self.max_integral,
            self.max_integral,
        )

        if self.previous_error_deg is None:
            derivative = 0.0
        else:
            derivative = (heading_error_deg - self.previous_error_deg) / delta_time_sec

        self.previous_error_deg = heading_error_deg

        p_term = self.kp * heading_error_deg
        i_term = self.ki * self.integral
        d_term = self.kd * derivative
        pid_output = clamp_float(
            p_term + i_term + d_term,
            -self.max_correction,
            self.max_correction,
        )

        left_frequency_hz = clamp_float(
            self.base_frequency_hz + pid_output,
            self.min_frequency_hz,
            self.max_frequency_hz,
        )
        right_frequency_hz = clamp_float(
            self.base_frequency_hz - pid_output,
            self.min_frequency_hz,
            self.max_frequency_hz,
        )

        return PIDResult(
            heading_error_deg=heading_error_deg,
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            p_term=p_term,
            i_term=i_term,
            d_term=d_term,
            pid_output=pid_output,
            left_frequency_hz=left_frequency_hz,
            right_frequency_hz=right_frequency_hz,
        )


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
                    "timestamp",
                    "distance_travelled_cm",
                    "state",
                    "reference_heading_deg",
                    "current_heading_deg",
                    "heading_error_deg",
                    "kp",
                    "ki",
                    "kd",
                    "p_term",
                    "i_term",
                    "d_term",
                    "pid_output",
                    "left_frequency_hz",
                    "right_frequency_hz",
                    "tag_visible",
                    "tag_orientation_deg",
                    "position_error_px",
                ]
            )

    def write(
        self,
        timestamp_sec: float,
        distance_travelled_cm: float,
        state: str,
        measurement: TagMeasurement,
        reference_heading_deg: float | None,
        current_heading_deg: float | None,
        heading_error_deg: float | None,
        tag_visible: bool,
        tag_acquired: bool,
        pid_result: PIDResult | None,
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
            pid_result: Latest PID result, or None before PID starts.
        """
        with self.csv_path.open("a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    format_csv(timestamp_sec),
                    format_csv(distance_travelled_cm),
                    state,
                    format_csv(reference_heading_deg),
                    format_csv(current_heading_deg),
                    format_csv(heading_error_deg),
                    format_csv(pid_result.kp if pid_result is not None else None),
                    format_csv(pid_result.ki if pid_result is not None else None),
                    format_csv(pid_result.kd if pid_result is not None else None),
                    format_csv(pid_result.p_term if pid_result is not None else None),
                    format_csv(pid_result.i_term if pid_result is not None else None),
                    format_csv(pid_result.d_term if pid_result is not None else None),
                    format_csv(pid_result.pid_output if pid_result is not None else None),
                    format_csv(pid_result.left_frequency_hz if pid_result is not None else None),
                    format_csv(pid_result.right_frequency_hz if pid_result is not None else None),
                    "YES" if tag_visible else "NO",
                    format_csv(measurement.orientation_deg),
                    format_csv(measurement.position_error_x),
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


def clamp_float(value: float, minimum: float, maximum: float) -> float:
    """
    Clamp a float into a safe range.

    Args:
        value: Input value.
        minimum: Lowest allowed value.
        maximum: Highest allowed value.

    Returns:
        Clamped value.
    """
    return max(minimum, min(maximum, value))


def format_set_drive_command(left_frequency_hz: float, right_frequency_hz: float) -> str:
    """
    Build a human-readable ESP32 SET_DRIVE command.

    Args:
        left_frequency_hz: Left motor frequency.
        right_frequency_hz: Right motor frequency.

    Returns:
        Serial command string.
    """
    return f"SET_DRIVE {left_frequency_hz:.0f} {right_frequency_hz:.0f}"


def update_pid_drive_command(
    serial_controller: SerialMotorController,
    pid_controller: HeadingPIDController,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    current_command: str,
    latest_pid_result: PIDResult | None,
    last_pid_update_time: float,
    now: float,
    program_start_time: float,
    movement_start_time: float | None,
    tag_orientation_deg: float | None,
) -> tuple[str, PIDResult | None, float]:
    """
    Send a 20 Hz SET_DRIVE command generated by the heading PID controller.

    Args:
        serial_controller: ESP32 serial controller.
        pid_controller: PID controller instance.
        reference_heading_deg: Desired heading.
        current_heading_deg: Latest IMU heading.
        current_command: Last command sent.
        latest_pid_result: Most recent PID result.
        last_pid_update_time: Last PID update timestamp.
        now: Current monotonic timestamp.
        program_start_time: Validation program start time.
        movement_start_time: Time when START_FORWARD was sent.
        tag_orientation_deg: Optional tag orientation for logging.

    Returns:
        Updated command, latest PID result, and PID timestamp.
    """
    if reference_heading_deg is None or current_heading_deg is None:
        return current_command, latest_pid_result, last_pid_update_time

    if (now - last_pid_update_time) < PID_UPDATE_INTERVAL_SEC:
        return current_command, latest_pid_result, last_pid_update_time

    pid_result = pid_controller.update(reference_heading_deg, current_heading_deg, now)
    command = format_set_drive_command(
        pid_result.left_frequency_hz,
        pid_result.right_frequency_hz,
    )
    current_command = send_if_changed(
        serial_controller,
        command,
        current_command,
        program_start_time,
        movement_start_time,
        pid_result.heading_error_deg,
        tag_orientation_deg,
    )

    return current_command, pid_result, now


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
        return "ALIGNING_LEFT"

    return "ALIGNING_RIGHT"


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


def send_if_changed(
    serial_controller: SerialMotorController,
    command: str,
    last_sent_command: str,
    program_start_time: float,
    movement_start_time: float | None,
    heading_error_deg: float | None = None,
    tag_orientation_deg: float | None = None,
) -> str:
    """
    Send a raw command only when it changes.

    Args:
        serial_controller: ESP32 serial controller.
        command: Desired command.
        last_sent_command: Last transmitted command.
        program_start_time: Validation program start time.
        movement_start_time: Time when START_FORWARD was sent.
        heading_error_deg: Optional heading error for transmission logs.
        tag_orientation_deg: Optional tag orientation for transmission logs.

    Returns:
        Updated last transmitted command.
    """
    if command == "NONE" or command == last_sent_command:
        return last_sent_command

    transmit_command(
        serial_controller,
        command,
        program_start_time,
        movement_start_time,
        heading_error_deg,
        tag_orientation_deg,
    )
    return command


def transmit_command(
    serial_controller: SerialMotorController,
    command: str,
    program_start_time: float,
    movement_start_time: float | None,
    heading_error_deg: float | None = None,
    tag_orientation_deg: float | None = None,
) -> bool:
    """
    Send one serial command with timestamped validation logging.

    Args:
        serial_controller: ESP32 serial controller.
        command: Raw serial command.
        program_start_time: Validation program start time.
        movement_start_time: Time when START_FORWARD was sent.
        heading_error_deg: Optional heading error for transmission logs.
        tag_orientation_deg: Optional tag orientation for transmission logs.

    Returns:
        True if the serial write succeeds.
    """
    now = time.monotonic()
    elapsed_time_sec = now - program_start_time
    print("Heading Error:", format_display(heading_error_deg, " deg", signed=True))
    print("Tag Orientation:", format_display(tag_orientation_deg, " deg", signed=True))
    print("Command Sent:", command)
    print(f"TX >>> {command}")
    print(f"[{elapsed_time_sec:.3f}s] TX >>> {command}")

    if movement_start_time is not None:
        print(f"Elapsed since START_FORWARD: {now - movement_start_time:.3f} s")
    elif command in {
        "START_LEFT_CORRECTION",
        "START_RIGHT_CORRECTION",
        "STOP_CORRECTION",
    }:
        print(f"WARNING: {command} is being sent before START_FORWARD")

    return serial_controller.send_raw_command(command, force=True)


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
    print(f"STATE TRANSITION >>> {state}")
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
    cv2.rectangle(
        frame,
        (image_center_x - 10, image_center_y - 80),
        (image_center_x + 10, image_center_y + 80),
        (255, 255, 0),
        2,
    )
    cv2.putText(
        frame,
        "ALIGN BOX",
        (image_center_x - 58, image_center_y - 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 0),
        2,
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
            ("Vision Orientation", pass_fail_text(gate_status.orientation_pass), pass_fail_color(gate_status.orientation_pass)),
            ("Vision Position", pass_fail_text(gate_status.position_pass), pass_fail_color(gate_status.position_pass)),
            ("Vision Stable", f"{gate_status.stable_time_sec:.2f} s", (255, 255, 255)),
            ("Vision Gate Status", gate_status.status_text, pass_fail_color(gate_status.status_text == "READY_TO_MOVE")),
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


def format_pid_value(pid_result: PIDResult | None, attribute: str, suffix: str = "") -> str:
    """
    Format one optional PID field for terminal diagnostics.

    Args:
        pid_result: Latest PID result.
        attribute: PIDResult field name.
        suffix: Optional unit suffix.

    Returns:
        Formatted value or None.
    """
    if pid_result is None:
        return "None"

    return format_display(getattr(pid_result, attribute), suffix=suffix, signed=True)


def log_terminal(
    state: str,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    measurement: TagMeasurement,
    tag_visible: bool,
    tag_acquired: bool,
    current_command: str,
    gate_status: StartGateStatus | None,
    pid_result: PIDResult | None,
) -> None:
    """Print one terminal diagnostic block."""
    print("Current State:", state)
    print("Reference Heading:", format_display(reference_heading_deg, " deg"))
    print("Current Heading:", format_display(current_heading_deg, " deg"))
    print("Heading Error:", format_display(heading_error_deg, " deg", signed=True))
    print("KP:", KP_HEADING)
    print("KI:", KI_HEADING)
    print("KD:", KD_HEADING)
    print("P Term:", format_pid_value(pid_result, "p_term"))
    print("I Term:", format_pid_value(pid_result, "i_term"))
    print("D Term:", format_pid_value(pid_result, "d_term"))
    print("PID Output:", format_pid_value(pid_result, "pid_output"))
    print("Left Frequency:", format_pid_value(pid_result, "left_frequency_hz", " Hz"))
    print("Right Frequency:", format_pid_value(pid_result, "right_frequency_hz", " Hz"))
    print("Tag Orientation:", format_display(measurement.orientation_deg, " deg", signed=True))
    print("Position Error:", format_display(measurement.position_error_x, " px", decimals=0, signed=True))
    print(
        "Vision Gate Status:",
        gate_status.status_text if gate_status is not None else "None",
    )
    print("Tag Visible:", "YES" if tag_visible else "NO")
    print("Tag Acquired:", "YES" if tag_acquired else "NO")
    print("Command Sent:", current_command)
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
    heading_pid = HeadingPIDController(
        kp=KP_HEADING,
        ki=KI_HEADING,
        kd=KD_HEADING,
        max_integral=MAX_INTEGRAL,
        max_correction=MAX_CORRECTION,
        base_frequency_hz=BASE_FREQUENCY_HZ,
        min_frequency_hz=MIN_FREQUENCY_HZ,
        max_frequency_hz=MAX_FREQUENCY_HZ,
    )

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
    last_pid_update_time = 0.0
    latest_pid_result: PIDResult | None = None
    start_time = time.monotonic()
    movement_start_time: float | None = None
    moving_started = False
    start_forward_sent = False
    tag_acquired = False
    active_tag_id: int | None = None
    last_visible_tag_id: int | None = None
    ignored_tag_id_logged: int | None = None

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
                wrap_angle_deg(reference_heading_deg - current_heading_deg)
                if current_heading_deg is not None and reference_heading_deg is not None
                else None
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("s"), ord("S")):
                current_command = "STOP"
                transmit_command(
                    serial_controller,
                    current_command,
                    start_time,
                    movement_start_time,
                )
                moving_started = False
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
                    start_time,
                    movement_start_time,
                    heading_error_deg,
                    measurement.orientation_deg,
                )

                if alignment_state == "ALIGNED":
                    gate_stable_since = None
                    gate_status = None
                    state = "VISION_START_GATE"
                    log_transition("VISION_START_GATE")

            elif state == "VISION_START_GATE":
                gate_status, gate_stable_since = evaluate_start_gate(
                    measurement,
                    None,
                    gate_stable_since,
                    now,
                    args.start_gate_duration,
                )
                print("Current State:", state)
                print("Position Error X:", format_display(measurement.position_error_x, " px", decimals=0, signed=True))
                print("Tag Orientation:", format_display(measurement.orientation_deg, " deg", signed=True))
                print("Vision Gate Status:", gate_status.status_text)

                if gate_status.status_text == "READY_TO_MOVE":
                    print("## VISION START GATE PASSED")
                    print("VISION START GATE PASSED")
                    current_command = "CALIBRATE_IMU"
                    transmit_command(
                        serial_controller,
                        current_command,
                        start_time,
                        movement_start_time,
                    )
                    imu_reader.latest_heading_deg = None
                    imu_reader.latest_status = "IMU_CALIBRATING"
                    reference_heading_deg = None
                    calibration_sent = True
                    state = "CALIBRATE_IMU"
                    log_transition("CALIBRATE_IMU")

            elif state == "CALIBRATE_IMU":
                if (
                    calibration_sent
                    and current_heading_deg is not None
                    and not start_forward_sent
                ):
                    print("## IMU_READY RECEIVED")
                    print("IMU_READY RECEIVED")
                    log_transition("CAPTURE_REFERENCE_HEADING")
                    reference_heading_deg = current_heading_deg
                    heading_error_deg = 0.0
                    heading_pid.reset()
                    latest_pid_result = None
                    last_pid_update_time = 0.0
                    print("## REFERENCE HEADING CAPTURED")
                    print(
                        "REFERENCE HEADING CAPTURED:",
                        format_display(reference_heading_deg, " deg", signed=True),
                    )
                    current_command = "START_FORWARD"
                    print("## SENDING START_FORWARD")
                    print("START_FORWARD")
                    transmit_command(
                        serial_controller,
                        current_command,
                        start_time,
                        movement_start_time,
                    )
                    movement_start_time = time.monotonic()
                    print("TX: START_FORWARD")
                    print("START_FORWARD SENT")
                    moving_started = True
                    start_forward_sent = True
                    tag_acquired = True
                    active_tag_id = 1
                    last_visible_tag_id = 1
                    state = "FORWARD_START_DELAY"
                    log_transition("FORWARD_START_DELAY")

            elif moving_started:
                if state == "FORWARD_START_DELAY":
                    print("FORWARD START DELAY ACTIVE")
                    if (
                        movement_start_time is not None
                        and (now - movement_start_time) >= 1.0
                    ):
                        print("FORWARD START DELAY COMPLETE")
                        print("## ENTERING HEADING_HOLD")
                        print("ENTERING HEADING_HOLD")
                        state = "HEADING_HOLD"
                        log_transition("HEADING_HOLD")

                elif (
                    tag_visible
                    and measurement.tag_id == active_tag_id
                    and state != "VISION_TRACKING"
                ):
                    if ignored_tag_id_logged != measurement.tag_id:
                        print(f"Ignoring Current Tag: {measurement.tag_id}")
                        ignored_tag_id_logged = measurement.tag_id

                    (
                        current_command,
                        latest_pid_result,
                        last_pid_update_time,
                    ) = update_pid_drive_command(
                        serial_controller,
                        heading_pid,
                        reference_heading_deg,
                        current_heading_deg,
                        current_command,
                        latest_pid_result,
                        last_pid_update_time,
                        now,
                        start_time,
                        movement_start_time,
                        measurement.orientation_deg,
                    )
                    if latest_pid_result is not None:
                        heading_error_deg = latest_pid_result.heading_error_deg

                elif tag_visible:
                    if measurement.tag_id != active_tag_id:
                        print(f"Next Tag Detected: {measurement.tag_id}")
                        if state == "HEADING_HOLD":
                            print("HEADING_HOLD -> VISION_TRACKING")
                        state = "VISION_TRACKING"
                        tag_acquired = False
                        acquisition_stable_since = None
                        acquisition_status = None
                        active_tag_id = measurement.tag_id
                        ignored_tag_id_logged = None
                        log_transition("VISION_TRACKING")

                    last_visible_tag_id = measurement.tag_id
                    (
                        current_command,
                        latest_pid_result,
                        last_pid_update_time,
                    ) = update_pid_drive_command(
                        serial_controller,
                        heading_pid,
                        reference_heading_deg,
                        current_heading_deg,
                        current_command,
                        latest_pid_result,
                        last_pid_update_time,
                        now,
                        start_time,
                        movement_start_time,
                        measurement.orientation_deg,
                    )
                    if latest_pid_result is not None:
                        heading_error_deg = latest_pid_result.heading_error_deg

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
                            print("TAG_ACQUIRED")
                            print(
                                "REFERENCE_HEADING_RETAINED:",
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

                    (
                        current_command,
                        latest_pid_result,
                        last_pid_update_time,
                    ) = update_pid_drive_command(
                        serial_controller,
                        heading_pid,
                        reference_heading_deg,
                        current_heading_deg,
                        current_command,
                        latest_pid_result,
                        last_pid_update_time,
                        now,
                        start_time,
                        movement_start_time,
                        measurement.orientation_deg,
                    )
                    if latest_pid_result is not None:
                        heading_error_deg = latest_pid_result.heading_error_deg

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
                    latest_pid_result,
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
                    gate_status,
                    latest_pid_result,
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
        transmit_command(
            serial_controller,
            "STOP",
            start_time,
            movement_start_time,
        )
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
        default=1.0,
        help="Compatibility option; PID heading deadband is HEADING_DEADBAND_DEG.",
    )
    parser.add_argument(
        "--vision-orientation-deadband",
        type=float,
        default=2.0,
        help="Compatibility option; moving vision tracking now keeps PID heading hold.",
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
