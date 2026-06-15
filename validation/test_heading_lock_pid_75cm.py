from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

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
    from validation.performance_monitor import PerformanceMonitor
    from validation.test_heading_lock import (
        PIDResult,
        StartGateStatus,
        TagMeasurement,
        HeadingPIDController,
        choose_initial_orientation_state,
        command_for_alignment_state,
        evaluate_start_gate,
        format_csv,
        format_display,
        format_set_drive_command,
        get_orientation_color,
        measure_tag,
        transmit_command,
        wrap_angle_deg,
    )
except ModuleNotFoundError as error:
    DEPENDENCY_ERROR = error


KP_HEADING = 100.0
KI_HEADING = 0.0
KD_HEADING = 0.0

HEADING_DEADBAND_DEG = 0.2
MAX_INTEGRAL = 50.0
MAX_CORRECTION = 1000.0

BASE_FREQUENCY_HZ = 9000.0
MIN_FREQUENCY_HZ = 8000.0
MAX_FREQUENCY_HZ = 10000.0
PID_UPDATE_INTERVAL_SEC = 0.05

DEFAULT_TRAVEL_DISTANCE_CM = 75.0
DEFAULT_WHEEL_DIAMETER_MM = 117.0
DEFAULT_PULSES_PER_REVOLUTION = 20000


@dataclass
class DistanceEstimate:
    """Stores distance travelled using commanded wheel frequencies."""

    travelled_cm: float = 0.0
    last_update_time: float | None = None

    def reset(self) -> None:
        """Clear the travelled distance estimate."""
        self.travelled_cm = 0.0
        self.last_update_time = None

    def update(
        self,
        now: float,
        pid_result: PIDResult | None,
        wheel_diameter_mm: float,
        pulses_per_revolution: int,
    ) -> None:
        """
        Integrate commanded wheel frequency into approximate travel distance.

        Args:
            now: Current monotonic timestamp.
            pid_result: Latest PID result containing left/right frequency.
            wheel_diameter_mm: Wheel diameter used by the AGV.
            pulses_per_revolution: T60 driver pulse setting.
        """
        if pid_result is None:
            self.last_update_time = now
            return

        if self.last_update_time is None:
            self.last_update_time = now
            return

        delta_time_sec = max(now - self.last_update_time, 0.0)
        self.last_update_time = now

        average_frequency_hz = (
            pid_result.left_frequency_hz + pid_result.right_frequency_hz
        ) / 2.0
        wheel_circumference_mm = 3.141592653589793 * wheel_diameter_mm
        revolutions = average_frequency_hz * delta_time_sec / pulses_per_revolution
        self.travelled_cm += revolutions * wheel_circumference_mm / 10.0


class PIDDistanceLogger:
    """Writes PID heading-control samples for the 75 cm validation run."""

    def __init__(self, csv_path: Path) -> None:
        """
        Create a fresh PID validation CSV file.

        Args:
            csv_path: Output CSV path.
        """
        self.csv_path = csv_path
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.csv_path.open("w", newline="")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(
            [
                "event",
                "timestamp",
                "distance_travelled_cm",
                "reference_heading_deg",
                "current_heading_deg",
                "heading_error_deg",
                "navigation_authority",
                "fps",
                "detection_time_ms",
                "tag_visible",
                "tag_id",
                "tag_orientation_deg",
                "position_error_px",
                "kp",
                "ki",
                "kd",
                "p_term",
                "i_term",
                "d_term",
                "pid_output",
                "left_frequency_hz",
                "right_frequency_hz",
                "target_distance_cm",
                "actual_distance_cm",
                "max_heading_error_deg",
                "min_heading_error_deg",
                "avg_heading_error_deg",
                "final_heading_error_deg",
                "max_pid_output",
                "min_pid_output",
                "avg_pid_output",
                "correction_count",
                "runtime_sec",
            ]
        )

    def write(
        self,
        timestamp_sec: float,
        distance_travelled_cm: float,
        reference_heading_deg: float | None,
        current_heading_deg: float | None,
        heading_error_deg: float | None,
        navigation_authority: str,
        fps: float,
        detection_time_ms: float,
        tag_visible: bool,
        tag_id: int | None,
        tag_orientation_deg: float | None,
        position_error_px: float | None,
        pid_result: PIDResult | None,
    ) -> None:
        """
        Append one PID validation sample.

        Args:
            timestamp_sec: Time since program start.
            distance_travelled_cm: Estimated distance travelled.
            reference_heading_deg: Captured heading reference.
            current_heading_deg: Latest IMU heading.
            heading_error_deg: Current heading error.
            navigation_authority: VISION when a tag controls steering, otherwise IMU.
            fps: Camera FPS from PerformanceMonitor.
            detection_time_ms: Latest detection update time.
            tag_visible: True when a tag is visible.
            tag_id: Latest visible tag ID.
            tag_orientation_deg: Latest tag orientation.
            position_error_px: Latest horizontal position error.
            pid_result: Latest PID calculation.
        """
        self._writer.writerow(
            [
                "SAMPLE",
                format_csv(timestamp_sec),
                format_csv(distance_travelled_cm),
                format_csv(reference_heading_deg),
                format_csv(current_heading_deg),
                format_csv(heading_error_deg),
                navigation_authority,
                format_csv(fps),
                format_csv(detection_time_ms),
                "YES" if tag_visible else "NO",
                tag_id if tag_id is not None else "",
                format_csv(tag_orientation_deg),
                format_csv(position_error_px),
                format_csv(pid_result.kp if pid_result is not None else None),
                format_csv(pid_result.ki if pid_result is not None else None),
                format_csv(pid_result.kd if pid_result is not None else None),
                format_csv(pid_result.p_term if pid_result is not None else None),
                format_csv(pid_result.i_term if pid_result is not None else None),
                format_csv(pid_result.d_term if pid_result is not None else None),
                format_csv(pid_result.pid_output if pid_result is not None else None),
                format_csv(pid_result.left_frequency_hz if pid_result is not None else None),
                format_csv(pid_result.right_frequency_hz if pid_result is not None else None),
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
            ]
        )

    def write_summary(
        self,
        summary: dict[str, float | int | None],
    ) -> None:
        """
        Append one final RUN_COMPLETE summary row.

        Args:
            summary: Calculated run summary values.
        """
        self._writer.writerow(
            [
                "RUN_COMPLETE",
                format_csv(summary.get("runtime_sec")),
                format_csv(summary.get("actual_distance_cm")),
                format_csv(summary.get("reference_heading_deg")),
                format_csv(summary.get("final_current_heading_deg")),
                format_csv(summary.get("final_heading_error_deg")),
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                format_csv(summary.get("target_distance_cm")),
                format_csv(summary.get("actual_distance_cm")),
                format_csv(summary.get("max_heading_error_deg")),
                format_csv(summary.get("min_heading_error_deg")),
                format_csv(summary.get("avg_heading_error_deg")),
                format_csv(summary.get("final_heading_error_deg")),
                format_csv(summary.get("max_pid_output")),
                format_csv(summary.get("min_pid_output")),
                format_csv(summary.get("avg_pid_output")),
                summary.get("correction_count", ""),
                format_csv(summary.get("runtime_sec")),
            ]
        )
        self._csv_file.flush()

    def write_event(
        self,
        event_name: str,
        timestamp_sec: float,
        distance_travelled_cm: float,
        tag_id: int | None,
        tag_orientation_deg: float | None,
        position_error_px: float | None,
        navigation_authority: str,
    ) -> None:
        """
        Append a one-time navigation event row.

        Args:
            event_name: Event label.
            timestamp_sec: Time since program start.
            distance_travelled_cm: Estimated distance travelled.
            tag_id: Visible tag ID, when available.
            tag_orientation_deg: Visible tag orientation, when available.
            position_error_px: Visible tag position error, when available.
            navigation_authority: Current navigation authority.
        """
        self._writer.writerow(
            [
                event_name,
                format_csv(timestamp_sec),
                format_csv(distance_travelled_cm),
                "",
                "",
                "",
                navigation_authority,
                "",
                "",
                "YES" if tag_id is not None else "NO",
                tag_id if tag_id is not None else "",
                format_csv(tag_orientation_deg),
                format_csv(position_error_px),
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
                "",
            ]
        )
        self._csv_file.flush()

    def close(self) -> None:
        """Flush and close the CSV log file."""
        self._csv_file.flush()
        self._csv_file.close()


def create_pid_controller() -> HeadingPIDController:
    """
    Create the heading PID controller with phase-one tuning values.

    Returns:
        Configured HeadingPIDController.
    """
    return HeadingPIDController(
        kp=KP_HEADING,
        ki=KI_HEADING,
        kd=KD_HEADING,
        max_integral=MAX_INTEGRAL,
        max_correction=MAX_CORRECTION,
        base_frequency_hz=BASE_FREQUENCY_HZ,
        min_frequency_hz=MIN_FREQUENCY_HZ,
        max_frequency_hz=MAX_FREQUENCY_HZ,
    )


def send_set_drive(
    serial_controller: SerialMotorController,
    pid_result: PIDResult,
    program_start_time: float,
    movement_start_time: float | None,
) -> str:
    """
    Send one SET_DRIVE command from the PID result.

    Args:
        serial_controller: ESP32 serial connection.
        pid_result: Current PID result.
        program_start_time: Program start time for logging.
        movement_start_time: Movement start time for logging.

    Returns:
        Command text that was transmitted.
    """
    command = format_set_drive_command(
        pid_result.left_frequency_hz,
        pid_result.right_frequency_hz,
    )
    transmit_command(
        serial_controller,
        command,
        program_start_time,
        movement_start_time,
        pid_result.heading_error_deg,
        None,
    )
    return command


def draw_overlay(
    frame,
    detection: dict | None,
    measurement: TagMeasurement,
    state: str,
    distance_travelled_cm: float,
    target_distance_cm: float,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    navigation_authority: str,
    fps: float,
    detection_time_ms: float,
    tag_visible: bool,
    pid_result: PIDResult | None,
    gate_status: StartGateStatus | None,
    current_command: str,
) -> None:
    """
    Draw startup vision and PID validation information on the camera frame.

    Args:
        frame: Latest camera frame.
        detection: First visible AprilTag detection.
        measurement: Current AprilTag measurement.
        state: Current validation state.
        distance_travelled_cm: Estimated distance.
        target_distance_cm: Stop distance.
        reference_heading_deg: Captured heading reference.
        current_heading_deg: Latest IMU heading.
        heading_error_deg: Current heading error.
        navigation_authority: Current steering authority.
        fps: Camera FPS from PerformanceMonitor.
        detection_time_ms: Latest detection update time.
        tag_visible: True when a tag is visible.
        pid_result: Latest PID result.
        gate_status: Current vision gate status.
        current_command: Last transmitted command.
    """
    image_height, image_width = frame.shape[:2]
    image_center_x = int(image_width / 2)
    image_center_y = int(image_height / 2)

    cv2.line(frame, (image_center_x, 0), (image_center_x, image_height), (0, 255, 255), 1)
    cv2.line(frame, (0, image_center_y), (image_width, image_center_y), (0, 255, 255), 1)
    cv2.rectangle(
        frame,
        (image_center_x - 10, image_center_y - 80),
        (image_center_x + 10, image_center_y + 80),
        (255, 255, 0),
        2,
    )

    if detection is not None:
        corners = detection["corners"]
        corner_count = len(corners)
        for index in range(corner_count):
            corner_x, corner_y = corners[index]
            next_corner_x, next_corner_y = corners[(index + 1) % corner_count]
            cv2.line(
                frame,
                (int(corner_x), int(corner_y)),
                (int(next_corner_x), int(next_corner_y)),
                (0, 255, 0),
                2,
            )
        if len(corners) >= 2:
            first_corner_x, first_corner_y = corners[0]
            second_corner_x, second_corner_y = corners[1]
            cv2.line(
                frame,
                (int(first_corner_x), int(first_corner_y)),
                (int(second_corner_x), int(second_corner_y)),
                (255, 0, 255),
                3,
            )
        if measurement.tag_center_x is not None and measurement.tag_center_y is not None:
            cv2.circle(
                frame,
                (int(measurement.tag_center_x), int(measurement.tag_center_y)),
                6,
                (0, 0, 255),
                -1,
            )

    pid_text = "None"
    left_frequency = "None"
    right_frequency = "None"
    if pid_result is not None:
        pid_text = f"{pid_result.pid_output:+.2f}"
        left_frequency = f"{pid_result.left_frequency_hz:.0f} Hz"
        right_frequency = f"{pid_result.right_frequency_hz:.0f} Hz"

    lines = [
        ("State", state),
        ("Distance", f"{distance_travelled_cm:.1f} / {target_distance_cm:.1f} cm"),
        ("Reference Heading", format_display(reference_heading_deg, " deg", signed=True)),
        ("Current Heading", format_display(current_heading_deg, " deg", signed=True)),
        ("Heading Error", format_display(heading_error_deg, " deg", signed=True)),
        ("Authority", navigation_authority),
        ("FPS", f"{fps:.1f}"),
        ("Detection Time", f"{detection_time_ms:.1f} ms"),
        ("Tag Visible", "YES" if tag_visible else "NO"),
        ("Tag ID", str(measurement.tag_id) if measurement.tag_id is not None else "None"),
        ("P Term", format_display(pid_result.p_term if pid_result else None, signed=True)),
        ("I Term", format_display(pid_result.i_term if pid_result else None, signed=True)),
        ("D Term", format_display(pid_result.d_term if pid_result else None, signed=True)),
        ("PID Output", pid_text),
        ("Left Frequency", left_frequency),
        ("Right Frequency", right_frequency),
        ("Tag Orientation", format_display(measurement.orientation_deg, " deg", signed=True)),
        ("Position Error X", format_display(measurement.position_error_x, " px", decimals=0, signed=True)),
        ("Vision Gate", gate_status.status_text if gate_status else "None"),
        ("Command", current_command),
    ]

    y = 28
    for label, value in lines:
        color = get_orientation_color(measurement.orientation_deg) if label == "Tag Orientation" else (255, 255, 255)
        cv2.putText(
            frame,
            f"{label}: {value}",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            color,
            2,
        )
        y += 26

    cv2.putText(
        frame,
        "Keys: S STOP | Q QUIT",
        (20, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        2,
    )


def log_terminal(
    distance_travelled_cm: float,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    navigation_authority: str,
    fps: float,
    detection_time_ms: float,
    tag_visible: bool,
    tag_id: int | None,
    pid_result: PIDResult | None,
    current_command: str,
) -> None:
    """
    Print PID heading-control diagnostics to the terminal.

    Args:
        distance_travelled_cm: Estimated travel distance.
        reference_heading_deg: Captured heading reference.
        current_heading_deg: Latest IMU heading.
        heading_error_deg: Current heading error.
        navigation_authority: Current steering authority.
        fps: Camera FPS from PerformanceMonitor.
        detection_time_ms: Latest detection update time.
        tag_visible: True when a tag is visible.
        tag_id: Latest visible tag ID.
        pid_result: Latest PID result.
        current_command: Last transmitted command.
    """
    print("Distance Travelled:", f"{distance_travelled_cm:.2f} cm")
    print("Reference Heading:", format_display(reference_heading_deg, " deg", signed=True))
    print("Current Heading:", format_display(current_heading_deg, " deg", signed=True))
    print("Heading Error:", format_display(heading_error_deg, " deg", signed=True))
    print("Navigation Authority:", navigation_authority)
    print("FPS:", f"{fps:.1f}")
    print("Detection Time:", f"{detection_time_ms:.1f} ms")
    print("Tag Visible:", "YES" if tag_visible else "NO")
    print("Tag ID:", tag_id if tag_id is not None else "None")
    print("P Term:", format_display(pid_result.p_term if pid_result else None, signed=True))
    print("I Term:", format_display(pid_result.i_term if pid_result else None, signed=True))
    print("D Term:", format_display(pid_result.d_term if pid_result else None, signed=True))
    print("PID Output:", format_display(pid_result.pid_output if pid_result else None, signed=True))
    print("Left Frequency:", format_display(pid_result.left_frequency_hz if pid_result else None, " Hz"))
    print("Right Frequency:", format_display(pid_result.right_frequency_hz if pid_result else None, " Hz"))
    print("Command:", current_command)
    print()


def print_processing_startup_summary(
    perception_manager: PerceptionManager,
    frame,
    fps: float,
    detection_time_ms: float,
) -> None:
    """
    Print measured camera and detection resolution details.

    Args:
        perception_manager: Active perception manager.
        frame: Latest camera frame.
        fps: Measured camera loop FPS.
        detection_time_ms: Latest detection update time.
    """
    if frame is None:
        return

    frame_height, frame_width = frame.shape[:2]
    diagnostics = perception_manager.camera_manager.diagnostics

    configured_resolution = "Unknown"
    target_fps = 0.0
    pixel_format = "Unknown"
    if diagnostics is not None:
        configured_resolution = (
            f"{diagnostics.configured_width}x{diagnostics.configured_height}"
        )
        target_fps = diagnostics.expected_fps
        pixel_format = diagnostics.pixel_format

    print("Processing startup summary:")
    print(f"Camera Resolution: {configured_resolution}")
    print(f"Detection Resolution: {frame_width}x{frame_height}")
    print(f"Pixel Format: {pixel_format}")
    print(f"Target FPS: {target_fps:.1f}")
    print(f"Actual FPS: {fps:.1f}")
    print(f"Detection Time: {detection_time_ms:.1f} ms")
    print()


def calculate_run_summary(
    heading_error_samples: list[float],
    pid_output_samples: list[float],
    left_frequency_samples: list[float],
    right_frequency_samples: list[float],
    current_heading_samples: list[float],
    reference_heading_deg: float | None,
    target_distance_cm: float,
    actual_distance_cm: float,
    runtime_sec: float,
) -> dict[str, float | int | None]:
    """
    Calculate heading-control summary values for later PID tuning.

    Args:
        heading_error_samples: Heading error samples from the run.
        pid_output_samples: PID output samples from the run.
        left_frequency_samples: Left frequency samples from the run.
        right_frequency_samples: Right frequency samples from the run.
        current_heading_samples: Current heading samples from the run.
        reference_heading_deg: Stored navigation heading reference.
        target_distance_cm: Commanded travel distance.
        actual_distance_cm: Estimated actual travel distance.
        runtime_sec: Movement runtime.

    Returns:
        Summary values.
    """
    if not heading_error_samples:
        return {
            "target_distance_cm": target_distance_cm,
            "actual_distance_cm": actual_distance_cm,
            "reference_heading_deg": reference_heading_deg,
            "final_current_heading_deg": current_heading_samples[-1]
            if current_heading_samples
            else None,
            "max_heading_error_deg": None,
            "min_heading_error_deg": None,
            "avg_heading_error_deg": None,
            "final_heading_error_deg": None,
            "max_pid_output": None,
            "min_pid_output": None,
            "avg_pid_output": None,
            "max_left_frequency_hz": None,
            "min_left_frequency_hz": None,
            "max_right_frequency_hz": None,
            "min_right_frequency_hz": None,
            "correction_count": 0,
            "runtime_sec": runtime_sec,
        }

    absolute_errors = [abs(error) for error in heading_error_samples]

    return {
        "target_distance_cm": target_distance_cm,
        "actual_distance_cm": actual_distance_cm,
        "reference_heading_deg": reference_heading_deg,
        "final_current_heading_deg": current_heading_samples[-1]
        if current_heading_samples
        else None,
        "max_heading_error_deg": max(heading_error_samples),
        "min_heading_error_deg": min(heading_error_samples),
        "avg_heading_error_deg": sum(absolute_errors) / len(absolute_errors),
        "final_heading_error_deg": heading_error_samples[-1],
        "max_pid_output": max(pid_output_samples) if pid_output_samples else None,
        "min_pid_output": min(pid_output_samples) if pid_output_samples else None,
        "avg_pid_output": sum(pid_output_samples) / len(pid_output_samples)
        if pid_output_samples
        else None,
        "max_left_frequency_hz": max(left_frequency_samples)
        if left_frequency_samples
        else None,
        "min_left_frequency_hz": min(left_frequency_samples)
        if left_frequency_samples
        else None,
        "max_right_frequency_hz": max(right_frequency_samples)
        if right_frequency_samples
        else None,
        "min_right_frequency_hz": min(right_frequency_samples)
        if right_frequency_samples
        else None,
        "correction_count": sum(
            1 for pid_output in pid_output_samples if abs(pid_output) > 0.0
        ),
        "runtime_sec": runtime_sec,
    }


def print_run_summary(summary: dict[str, float | int | None]) -> None:
    """
    Print the final heading-control run summary.

    Args:
        summary: Calculated run summary values.
    """
    if summary.get("max_heading_error_deg") is None:
        print("PID run summary unavailable: no heading samples were recorded.")
        return

    print("==================================================")
    print("RUN SUMMARY")
    print("==================================================")
    print()
    print("Target Distance:", f"{summary['target_distance_cm']:.2f} cm")
    print("Actual Distance:", f"{summary['actual_distance_cm']:.2f} cm")
    print()
    print("Reference Heading:", format_display(summary["reference_heading_deg"], " deg"))
    print("Final Heading:", format_display(summary["final_current_heading_deg"], " deg"))
    print()
    print("Maximum Heading Error:", format_display(summary["max_heading_error_deg"], " deg", signed=True))
    print("Minimum Heading Error:", format_display(summary["min_heading_error_deg"], " deg", signed=True))
    print("Average Heading Error:", format_display(summary["avg_heading_error_deg"], " deg"))
    print()
    print("Maximum PID Output:", format_display(summary["max_pid_output"], signed=True))
    print("Minimum PID Output:", format_display(summary["min_pid_output"], signed=True))
    print("Average PID Output:", format_display(summary["avg_pid_output"], signed=True))
    print()
    print("Maximum Left Frequency:", format_display(summary["max_left_frequency_hz"], " Hz", decimals=0))
    print("Minimum Left Frequency:", format_display(summary["min_left_frequency_hz"], " Hz", decimals=0))
    print()
    print("Maximum Right Frequency:", format_display(summary["max_right_frequency_hz"], " Hz", decimals=0))
    print("Minimum Right Frequency:", format_display(summary["min_right_frequency_hz"], " Hz", decimals=0))
    print()
    print("PID Corrections Applied:", summary["correction_count"])
    print()
    print("Total Runtime:", f"{summary['runtime_sec']:.2f} sec")
    print()
    print("==================================================")
    print()


def finalize_run_summary(
    logger: PIDDistanceLogger,
    heading_error_samples: list[float],
    pid_output_samples: list[float],
    left_frequency_samples: list[float],
    right_frequency_samples: list[float],
    current_heading_samples: list[float],
    reference_heading_deg: float | None,
    target_distance_cm: float,
    actual_distance_cm: float,
    movement_start_time: float | None,
    now: float,
    summary_printed: bool,
) -> bool:
    """
    Print and export the run summary once.

    Args:
        logger: CSV logger.
        heading_error_samples: Heading error samples.
        pid_output_samples: PID output samples.
        left_frequency_samples: Left wheel frequency samples.
        right_frequency_samples: Right wheel frequency samples.
        current_heading_samples: Current heading samples.
        reference_heading_deg: Stored navigation heading reference.
        target_distance_cm: Commanded travel distance.
        actual_distance_cm: Estimated distance travelled.
        movement_start_time: Time when the first SET_DRIVE command was sent.
        now: Current monotonic timestamp.
        summary_printed: True when the summary was already emitted.

    Returns:
        True after the summary has been emitted.
    """
    if summary_printed:
        return True

    runtime_sec = 0.0
    if movement_start_time is not None:
        runtime_sec = max(now - movement_start_time, 0.0)

    summary = calculate_run_summary(
        heading_error_samples,
        pid_output_samples,
        left_frequency_samples,
        right_frequency_samples,
        current_heading_samples,
        reference_heading_deg,
        target_distance_cm,
        actual_distance_cm,
        runtime_sec,
    )
    print_run_summary(summary)
    logger.write_summary(summary)
    return True


def log_navigation_event(
    logger: PIDDistanceLogger,
    event_name: str,
    tag_id: int | None,
    distance_cm: float,
    timestamp_sec: float,
    tag_orientation_deg: float | None,
    position_error_px: float | None,
    navigation_authority: str,
) -> None:
    """
    Print and write a one-time navigation event.

    Args:
        logger: CSV logger.
        event_name: Event label.
        tag_id: Visible tag ID.
        distance_cm: Estimated distance travelled.
        timestamp_sec: Time since program start.
        tag_orientation_deg: Tag orientation, when available.
        position_error_px: Tag position error, when available.
        navigation_authority: Current navigation authority.
    """
    print(event_name)
    print("tag_id:", tag_id if tag_id is not None else "None")
    print("distance_cm:", f"{distance_cm:.2f}")
    print("timestamp:", f"{timestamp_sec:.2f}")
    print()
    logger.write_event(
        event_name,
        timestamp_sec,
        distance_cm,
        tag_id,
        tag_orientation_deg,
        position_error_px,
        navigation_authority,
    )


def run_validation(args: argparse.Namespace) -> None:
    """Run the 75 cm PID heading-control validation test."""
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
    pid_controller = create_pid_controller()
    distance_estimate = DistanceEstimate()
    monitor = PerformanceMonitor()
    logger = PIDDistanceLogger(Path(args.csv).expanduser().resolve())

    state = "WAIT_FOR_TAG1"
    current_command = "NONE"
    gate_status: StartGateStatus | None = None
    gate_stable_since: float | None = None
    navigation_reference_heading_deg: float | None = None
    latest_pid_result: PIDResult | None = None
    last_pid_update_time = 0.0
    last_log_time = 0.0
    last_csv_time = 0.0
    start_time = time.monotonic()
    movement_start_time: float | None = None
    navigation_authority = "NONE"
    last_logged_tag_id: int | None = None
    vision_authority_tag_id: int | None = None
    summary_printed = False
    processing_summary_printed = False
    heading_error_samples: list[float] = []
    pid_output_samples: list[float] = []
    left_frequency_samples: list[float] = []
    right_frequency_samples: list[float] = []
    current_heading_samples: list[float] = []

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        logger.close()
        return

    if not serial_controller.connect():
        logger.close()
        perception_manager.release()
        return

    print("STATE TRANSITION >>> WAIT_FOR_TAG1")

    try:
        while True:
            detections = monitor.measure_detection_cycle(perception_manager.update)
            monitor.record_camera_frame()
            frame = perception_manager.last_frame
            detection = detections[0] if detections else None
            measurement = measure_tag(frame, detection)
            tag_visible = measurement.tag_id is not None
            monitor.record_tag_detection(measurement.tag_id)
            metrics = monitor.metrics

            if (
                not processing_summary_printed
                and frame is not None
                and metrics.camera_fps > 0.0
            ):
                print_processing_startup_summary(
                    perception_manager,
                    frame,
                    metrics.camera_fps,
                    metrics.detection_time_ms,
                )
                processing_summary_printed = True

            imu_reader.update_from_connection(serial_controller.serial_connection)
            current_heading_deg = imu_reader.get_heading()

            now = time.monotonic()
            elapsed_time_sec = now - start_time
            heading_error_deg = (
                wrap_angle_deg(navigation_reference_heading_deg - current_heading_deg)
                if (
                    navigation_reference_heading_deg is not None
                    and current_heading_deg is not None
                )
                else None
            )

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("s"), ord("S")):
                current_command = "STOP"
                transmit_command(serial_controller, current_command, start_time, movement_start_time)
                summary_printed = finalize_run_summary(
                    logger,
                    heading_error_samples,
                    pid_output_samples,
                    left_frequency_samples,
                    right_frequency_samples,
                    current_heading_samples,
                    navigation_reference_heading_deg,
                    args.travel_distance_cm,
                    distance_estimate.travelled_cm,
                    movement_start_time,
                    now,
                    summary_printed,
                )
                state = "STOPPED"
                print("STATE TRANSITION >>> STOPPED")

            if state == "WAIT_FOR_TAG1" and measurement.tag_id == 1:
                state = "ALIGN_TAG1"
                print("STATE TRANSITION >>> ALIGN_TAG1")

            elif state == "ALIGN_TAG1":
                alignment_state = choose_initial_orientation_state(
                    measurement.orientation_deg,
                    args.orientation_tolerance,
                )
                desired_command = command_for_alignment_state(alignment_state)
                if desired_command != "NONE" and desired_command != current_command:
                    current_command = desired_command
                    transmit_command(
                        serial_controller,
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
                    print("STATE TRANSITION >>> VISION_START_GATE")

            elif state == "VISION_START_GATE":
                gate_status, gate_stable_since = evaluate_start_gate(
                    measurement,
                    None,
                    gate_stable_since,
                    now,
                    args.start_gate_duration,
                )

                if gate_status.status_text == "READY_TO_MOVE":
                    print("VISION START GATE PASSED")
                    current_command = "CALIBRATE_IMU"
                    transmit_command(serial_controller, current_command, start_time, movement_start_time)
                    imu_reader.latest_heading_deg = None
                    imu_reader.latest_status = "IMU_CALIBRATING"
                    navigation_reference_heading_deg = None
                    state = "CALIBRATE_IMU"
                    print("STATE TRANSITION >>> CALIBRATE_IMU")

            elif state == "CALIBRATE_IMU":
                if current_heading_deg is not None:
                    print("IMU_READY RECEIVED")
                    navigation_reference_heading_deg = current_heading_deg
                    heading_error_deg = 0.0
                    pid_controller.reset()
                    distance_estimate.reset()
                    latest_pid_result = None
                    last_pid_update_time = 0.0
                    movement_start_time = None
                    print(
                        "NAVIGATION REFERENCE HEADING CAPTURED:",
                        format_display(
                            navigation_reference_heading_deg,
                            " deg",
                            signed=True,
                        ),
                    )
                    state = "HEADING_HOLD_75CM"
                    print("STATE TRANSITION >>> HEADING_HOLD_75CM")

            elif state == "HEADING_HOLD_75CM":
                distance_estimate.update(
                    now,
                    latest_pid_result,
                    args.wheel_diameter_mm,
                    args.pulses_per_revolution,
                )

                if tag_visible and measurement.tag_id != last_logged_tag_id:
                    log_navigation_event(
                        logger,
                        "TAG_DETECTED",
                        measurement.tag_id,
                        distance_estimate.travelled_cm,
                        elapsed_time_sec,
                        measurement.orientation_deg,
                        measurement.position_error_x,
                        navigation_authority,
                    )
                    last_logged_tag_id = measurement.tag_id

                if not tag_visible:
                    last_logged_tag_id = None

                tag_has_valid_orientation = (
                    tag_visible and measurement.orientation_deg is not None
                )

                if tag_has_valid_orientation:
                    selected_authority = "VISION"
                elif (
                    navigation_reference_heading_deg is not None
                    and current_heading_deg is not None
                ):
                    selected_authority = "IMU"
                else:
                    selected_authority = "NONE"

                if selected_authority != navigation_authority:
                    if selected_authority == "VISION":
                        log_navigation_event(
                            logger,
                            "VISION_AUTHORITY_ENTERED",
                            measurement.tag_id,
                            distance_estimate.travelled_cm,
                            elapsed_time_sec,
                            measurement.orientation_deg,
                            measurement.position_error_x,
                            selected_authority,
                        )
                        vision_authority_tag_id = measurement.tag_id

                    if (
                        navigation_authority == "VISION"
                        and selected_authority != "VISION"
                    ):
                        log_navigation_event(
                            logger,
                            "VISION_AUTHORITY_EXITED",
                            vision_authority_tag_id,
                            distance_estimate.travelled_cm,
                            elapsed_time_sec,
                            measurement.orientation_deg,
                            measurement.position_error_x,
                            navigation_authority,
                        )
                        vision_authority_tag_id = None

                    navigation_authority = selected_authority
                    if navigation_authority != "NONE":
                        pid_controller.reset()
                    print("NAVIGATION AUTHORITY:", navigation_authority)

                if distance_estimate.travelled_cm >= args.travel_distance_cm:
                    current_command = "STOP"
                    transmit_command(serial_controller, current_command, start_time, movement_start_time)
                    state = "DISTANCE_COMPLETE"
                    print("TRAVEL DISTANCE COMPLETE")
                    print("STATE TRANSITION >>> DISTANCE_COMPLETE")
                    summary_printed = finalize_run_summary(
                        logger,
                        heading_error_samples,
                        pid_output_samples,
                        left_frequency_samples,
                        right_frequency_samples,
                        current_heading_samples,
                        navigation_reference_heading_deg,
                        args.travel_distance_cm,
                        distance_estimate.travelled_cm,
                        movement_start_time,
                        now,
                        summary_printed,
                    )
                elif (
                    navigation_reference_heading_deg is not None
                    and (now - last_pid_update_time) >= PID_UPDATE_INTERVAL_SEC
                ):
                    if navigation_authority == "VISION":
                        selected_error_deg = measurement.orientation_deg
                    elif (
                        navigation_authority == "IMU"
                        and current_heading_deg is not None
                    ):
                        selected_error_deg = wrap_angle_deg(
                            navigation_reference_heading_deg - current_heading_deg
                        )
                    else:
                        selected_error_deg = None

                    if selected_error_deg is not None:
                        latest_pid_result = pid_controller.update(
                            selected_error_deg,
                            0.0,
                            now,
                        )
                        heading_error_deg = latest_pid_result.heading_error_deg
                        current_command = send_set_drive(
                            serial_controller,
                            latest_pid_result,
                            start_time,
                            movement_start_time,
                        )
                        heading_error_samples.append(latest_pid_result.heading_error_deg)
                        pid_output_samples.append(latest_pid_result.pid_output)
                        left_frequency_samples.append(latest_pid_result.left_frequency_hz)
                        right_frequency_samples.append(latest_pid_result.right_frequency_hz)
                        if current_heading_deg is not None:
                            current_heading_samples.append(current_heading_deg)
                        if movement_start_time is None:
                            movement_start_time = now
                            distance_estimate.last_update_time = now
                        last_pid_update_time = now

            elif state == "DISTANCE_COMPLETE" and not summary_printed:
                summary_printed = finalize_run_summary(
                    logger,
                    heading_error_samples,
                    pid_output_samples,
                    left_frequency_samples,
                    right_frequency_samples,
                    current_heading_samples,
                    navigation_reference_heading_deg,
                    args.travel_distance_cm,
                    distance_estimate.travelled_cm,
                    movement_start_time,
                    now,
                    summary_printed,
                )

            if (now - last_csv_time) >= args.csv_interval:
                logger.write(
                    elapsed_time_sec,
                    distance_estimate.travelled_cm,
                    navigation_reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    navigation_authority,
                    metrics.camera_fps,
                    metrics.detection_time_ms,
                    tag_visible,
                    measurement.tag_id,
                    measurement.orientation_deg,
                    measurement.position_error_x,
                    latest_pid_result,
                )
                last_csv_time = now

            if (now - last_log_time) >= args.log_interval:
                log_terminal(
                    distance_estimate.travelled_cm,
                    navigation_reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    navigation_authority,
                    metrics.camera_fps,
                    metrics.detection_time_ms,
                    tag_visible,
                    measurement.tag_id,
                    latest_pid_result,
                    current_command,
                )
                last_log_time = now

            if frame is not None:
                draw_overlay(
                    frame,
                    detection,
                    measurement,
                    state,
                    distance_estimate.travelled_cm,
                    args.travel_distance_cm,
                    navigation_reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    navigation_authority,
                    metrics.camera_fps,
                    metrics.detection_time_ms,
                    tag_visible,
                    latest_pid_result,
                    gate_status,
                    current_command,
                )
                cv2.imshow("PID Heading Lock 75 cm Validation", frame)

            if key in (ord("q"), ord("Q")):
                break
    except KeyboardInterrupt:
        print("Stopping PID heading validation.")
    finally:
        transmit_command(serial_controller, "STOP", start_time, movement_start_time)
        logger.close()
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse 75 cm PID validation options."""
    parser = argparse.ArgumentParser(
        description="Validation-only PID heading hold over a fixed 75 cm distance.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--orientation-tolerance", type=float, default=2.0)
    parser.add_argument("--start-gate-duration", type=float, default=0.5)
    parser.add_argument("--travel-distance-cm", type=float, default=DEFAULT_TRAVEL_DISTANCE_CM)
    parser.add_argument("--wheel-diameter-mm", type=float, default=DEFAULT_WHEEL_DIAMETER_MM)
    parser.add_argument("--pulses-per-revolution", type=int, default=DEFAULT_PULSES_PER_REVOLUTION)
    parser.add_argument("--csv", default="validation/heading_lock_pid_75cm_log.csv")
    parser.add_argument("--csv-interval", type=float, default=0.1)
    parser.add_argument("--log-interval", type=float, default=0.25)
    return parser.parse_args()


if __name__ == "__main__":
    if any(argument in ("-h", "--help") for argument in sys.argv[1:]):
        parse_args()
        raise SystemExit(0)

    if DEPENDENCY_ERROR is not None:
        print(
            "PID 75 cm validation cannot start because a required dependency "
            f"is missing: {DEPENDENCY_ERROR.name}"
        )
        print(
            "Run this validation on the Raspberry Pi environment containing "
            "OpenCV, pyserial, Picamera2, and pupil_apriltags."
        )
        raise SystemExit(1)

    run_validation(parse_args())
