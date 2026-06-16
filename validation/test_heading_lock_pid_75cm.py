from __future__ import annotations

import argparse
import csv
import math
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
    from core.perception_state import PerceptionState
    from perception.perception_manager import PerceptionManager
    from validation.test_heading_lock import (
        PIDResult,
        StartGateStatus,
        HeadingPIDController,
        choose_initial_orientation_state,
        command_for_alignment_state,
        evaluate_start_gate,
        format_csv,
        format_display,
        format_set_drive_command,
        get_orientation_color,
        transmit_command,
        wrap_angle_deg,
    )
except ModuleNotFoundError as error:
    DEPENDENCY_ERROR = error


KP_HEADING = 90.0
KI_HEADING = 10.0
KD_HEADING = 0.0

HEADING_DEADBAND_DEG = 0.2
MAX_INTEGRAL = 50.0
MAX_CORRECTION = 1000.0

BASE_FREQUENCY_HZ = 9000.0
MIN_FREQUENCY_HZ = 8000.0
MAX_FREQUENCY_HZ = 10000.0
PID_UPDATE_INTERVAL_SEC = 0.05
VISION_LOST_FRAME_THRESHOLD = 5
TAG_ACQUISITION_FRAME_THRESHOLD = 3
POSITION_GAIN = 0.01
VISION_POSITION_GAIN_DEG_PER_PX = POSITION_GAIN
VISION_POSITION_COMPONENT_LIMIT_DEG = 3.0

DEFAULT_TRAVEL_DISTANCE_CM = 150.0
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


@dataclass
class TagVisibilityStats:
    """Stores diagnostic visibility statistics for one AprilTag."""

    tag_id: int
    first_seen_timestamp: float | None = None
    last_seen_timestamp: float | None = None
    first_seen_distance_cm: float | None = None
    last_seen_distance_cm: float | None = None
    total_frames_visible: int = 0
    current_visible_streak: int = 0
    longest_consecutive_visible_frames: int = 0
    currently_visible: bool = False
    acquired: bool = False
    tag_pixel_width: float | None = None
    tag_pixel_height: float | None = None
    tag_center_x: float | None = None
    tag_center_y: float | None = None
    tag_orientation_deg: float | None = None
    detection_time_ms: float = 0.0
    fps: float = 0.0


class TagVisibilityLogger:
    """Writes AprilTag visibility events and summaries."""

    def __init__(self, csv_path: Path) -> None:
        """
        Create a fresh tag visibility diagnostics CSV file.

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
                "tag_id",
                "timestamp_sec",
                "distance_cm",
                "visible_frame_count",
                "longest_visible_streak",
                "tag_center_x",
                "tag_center_y",
                "tag_orientation_deg",
                "tag_pixel_width",
                "tag_pixel_height",
                "detection_time_ms",
                "fps",
                "first_seen_timestamp",
                "last_seen_timestamp",
                "first_detection_distance_cm",
                "last_detection_distance_cm",
                "total_detections",
            ]
        )

    def write_event(
        self,
        event_name: str,
        stats: TagVisibilityStats,
        timestamp_sec: float,
        distance_cm: float,
    ) -> None:
        """
        Write one tag visibility event.

        Args:
            event_name: Event label.
            stats: Current tag statistics.
            timestamp_sec: Current run timestamp.
            distance_cm: Current distance estimate.
        """
        print(event_name)
        print("tag_id:", stats.tag_id)
        print("distance_cm:", f"{distance_cm:.2f}")
        print("timestamp:", f"{timestamp_sec:.2f}")
        print("visible_frame_count:", stats.total_frames_visible)
        print("longest_visible_streak:", stats.longest_consecutive_visible_frames)
        print("tag_center_x:", format_display(stats.tag_center_x))
        print("tag_center_y:", format_display(stats.tag_center_y))
        print("tag_orientation_deg:", format_display(stats.tag_orientation_deg, " deg", signed=True))
        print()
        self._writer.writerow(
            [
                event_name,
                stats.tag_id,
                format_csv(timestamp_sec),
                format_csv(distance_cm),
                stats.total_frames_visible,
                stats.longest_consecutive_visible_frames,
                format_csv(stats.tag_center_x),
                format_csv(stats.tag_center_y),
                format_csv(stats.tag_orientation_deg),
                format_csv(stats.tag_pixel_width),
                format_csv(stats.tag_pixel_height),
                format_csv(stats.detection_time_ms),
                format_csv(stats.fps),
                format_csv(stats.first_seen_timestamp),
                format_csv(stats.last_seen_timestamp),
                format_csv(stats.first_seen_distance_cm),
                format_csv(stats.last_seen_distance_cm),
                stats.total_frames_visible,
            ]
        )
        self._csv_file.flush()

    def write_summary(self, stats_by_tag: dict[int, TagVisibilityStats]) -> None:
        """
        Write final tag visibility summary rows.

        Args:
            stats_by_tag: All collected tag visibility statistics.
        """
        print("==================================================")
        print("TAG VISIBILITY SUMMARY")
        print("==================================================")
        for tag_id in sorted(stats_by_tag):
            stats = stats_by_tag[tag_id]
            print(f"Tag {tag_id}:")
            print("  first seen distance:", format_display(stats.first_seen_distance_cm, " cm"))
            print("  last seen distance:", format_display(stats.last_seen_distance_cm, " cm"))
            print("  visible frames:", stats.total_frames_visible)
            print("  longest streak:", stats.longest_consecutive_visible_frames)
            print()
            self._writer.writerow(
                [
                    "TAG_SUMMARY",
                    stats.tag_id,
                    "",
                    "",
                    stats.total_frames_visible,
                    stats.longest_consecutive_visible_frames,
                    format_csv(stats.tag_center_x),
                    format_csv(stats.tag_center_y),
                    format_csv(stats.tag_orientation_deg),
                    format_csv(stats.tag_pixel_width),
                    format_csv(stats.tag_pixel_height),
                    format_csv(stats.detection_time_ms),
                    format_csv(stats.fps),
                    format_csv(stats.first_seen_timestamp),
                    format_csv(stats.last_seen_timestamp),
                    format_csv(stats.first_seen_distance_cm),
                    format_csv(stats.last_seen_distance_cm),
                    stats.total_frames_visible,
                ]
            )
        self._csv_file.flush()

    def close(self) -> None:
        """Flush and close the tag visibility CSV log."""
        self._csv_file.flush()
        self._csv_file.close()


class TagVisibilityTracker:
    """Tracks per-tag visibility frame counts and acquisition diagnostics."""

    def __init__(
        self,
        logger: TagVisibilityLogger,
        acquisition_frame_threshold: int = TAG_ACQUISITION_FRAME_THRESHOLD,
    ) -> None:
        """
        Create a tag visibility tracker.

        Args:
            logger: Tag visibility CSV logger.
            acquisition_frame_threshold: Consecutive frames required for TAG_ACQUIRED.
        """
        self.logger = logger
        self.acquisition_frame_threshold = acquisition_frame_threshold
        self.stats_by_tag: dict[int, TagVisibilityStats] = {}
        self._summary_written = False

    def update(
        self,
        detections: list[dict],
        timestamp_sec: float,
        distance_cm: float,
        detection_time_ms: float,
        fps: float,
    ) -> None:
        """
        Update visibility statistics for the current frame.

        Args:
            detections: Raw AprilTag detections for the current frame.
            timestamp_sec: Time since run start.
            distance_cm: Current distance estimate.
            detection_time_ms: Latest AprilTag processing time.
            fps: Latest measured FPS.
        """
        visible_tag_ids: set[int] = set()

        for detection in detections:
            tag_id = int(detection["tag_id"])
            visible_tag_ids.add(tag_id)
            stats = self.stats_by_tag.setdefault(
                tag_id,
                TagVisibilityStats(tag_id=tag_id),
            )
            was_visible = stats.currently_visible
            self._update_visible_stats(
                stats,
                detection,
                timestamp_sec,
                distance_cm,
                detection_time_ms,
                fps,
            )

            if stats.total_frames_visible == 1:
                self.logger.write_event(
                    "TAG_FIRST_SEEN",
                    stats,
                    timestamp_sec,
                    distance_cm,
                )

            if (
                not stats.acquired
                and stats.current_visible_streak >= self.acquisition_frame_threshold
            ):
                stats.acquired = True
                self.logger.write_event(
                    "TAG_ACQUIRED",
                    stats,
                    timestamp_sec,
                    distance_cm,
                )

            if not was_visible:
                stats.currently_visible = True

        for tag_id, stats in self.stats_by_tag.items():
            if tag_id not in visible_tag_ids and stats.currently_visible:
                stats.currently_visible = False
                stats.current_visible_streak = 0
                self.logger.write_event(
                    "TAG_LOST",
                    stats,
                    timestamp_sec,
                    distance_cm,
                )

    def write_summary_once(self) -> None:
        """Write final tag visibility summary once."""
        if self._summary_written:
            return

        self.logger.write_summary(self.stats_by_tag)
        self._summary_written = True

    def _update_visible_stats(
        self,
        stats: TagVisibilityStats,
        detection: dict,
        timestamp_sec: float,
        distance_cm: float,
        detection_time_ms: float,
        fps: float,
    ) -> None:
        """
        Update statistics for one visible tag.

        Args:
            stats: Mutable tag statistics.
            detection: Raw detection dictionary.
            timestamp_sec: Current run timestamp.
            distance_cm: Current distance estimate.
            detection_time_ms: Latest detector timing.
            fps: Latest measured FPS.
        """
        if stats.first_seen_timestamp is None:
            stats.first_seen_timestamp = timestamp_sec
            stats.first_seen_distance_cm = distance_cm

        stats.last_seen_timestamp = timestamp_sec
        stats.last_seen_distance_cm = distance_cm
        stats.total_frames_visible += 1
        stats.current_visible_streak += 1
        stats.longest_consecutive_visible_frames = max(
            stats.longest_consecutive_visible_frames,
            stats.current_visible_streak,
        )
        stats.tag_center_x = float(detection["center_x"])
        stats.tag_center_y = float(detection["center_y"])
        stats.tag_orientation_deg = detection.get(
            "orientation_deg",
            calculate_detection_orientation_deg(detection),
        )
        stats.detection_time_ms = detection_time_ms
        stats.fps = fps
        stats.tag_pixel_width, stats.tag_pixel_height = calculate_tag_pixel_size(
            detection,
        )


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
                "vision_lost_count",
                "fps",
                "detection_time_ms",
                "tag_visible",
                "tag_id",
                "tag_orientation_deg",
                "position_error_px",
                "position_component_deg",
                "vision_error_deg",
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
        vision_lost_count: int,
        fps: float,
        detection_time_ms: float,
        tag_visible: bool,
        tag_id: int | None,
        tag_orientation_deg: float | None,
        position_error_px: float | None,
        position_component_deg: float | None,
        vision_error_deg: float | None,
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
            vision_lost_count: Consecutive missed frames while holding VISION.
            fps: Camera FPS from PerceptionManager.
            detection_time_ms: Latest detection update time.
            tag_visible: True when a tag is visible.
            tag_id: Latest visible tag ID.
            tag_orientation_deg: Latest tag orientation.
            position_error_px: Latest horizontal position error.
            position_component_deg: Shadow position correction component.
            vision_error_deg: Shadow combined vision error.
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
                vision_lost_count,
                format_csv(fps),
                format_csv(detection_time_ms),
                "YES" if tag_visible else "NO",
                tag_id if tag_id is not None else "",
                format_csv(tag_orientation_deg),
                format_csv(position_error_px),
                format_csv(position_component_deg),
                format_csv(vision_error_deg),
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


def calculate_detection_orientation_deg(detection: dict | None) -> float | None:
    """
    Calculate AprilTag orientation for old PerceptionManager compatibility.

    Args:
        detection: Raw AprilTag detection dictionary.

    Returns:
        Orientation normalized to -90..+90 degrees.
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


def calculate_tag_pixel_size(detection: dict | None) -> tuple[float | None, float | None]:
    """
    Calculate a detected tag's bounding-box size in image pixels.

    Args:
        detection: Raw AprilTag detection dictionary.

    Returns:
        Pixel width and height, or None values when unavailable.
    """
    if detection is None:
        return None, None

    corners = detection.get("corners", [])
    if not corners:
        return None, None

    x_values = [float(corner[0]) for corner in corners]
    y_values = [float(corner[1]) for corner in corners]
    return max(x_values) - min(x_values), max(y_values) - min(y_values)


def calculate_position_component_deg(position_error_px: float | None) -> float | None:
    """
    Convert horizontal tag offset into a diagnostic steering component.

    This value is logged only. It is not used by the PID steering loop.

    Args:
        position_error_px: Tag center minus image center in pixels.

    Returns:
        Position component in degrees, clamped to the diagnostic limit.
    """
    if position_error_px is None:
        return None

    raw_component = position_error_px * VISION_POSITION_GAIN_DEG_PER_PX
    return max(
        -VISION_POSITION_COMPONENT_LIMIT_DEG,
        min(VISION_POSITION_COMPONENT_LIMIT_DEG, raw_component),
    )


def calculate_combined_vision_error_deg(
    orientation_deg: float | None,
    position_component_deg: float | None,
) -> float | None:
    """
    Calculate a diagnostic combined vision error.

    This value is logged only. Current steering remains orientation-only.

    Args:
        orientation_deg: AprilTag orientation in degrees.
        position_component_deg: Diagnostic position component in degrees.

    Returns:
        Combined orientation plus position component, or None.
    """
    if orientation_deg is None:
        return None

    return orientation_deg + (position_component_deg or 0.0)


def expected_position_steering_direction(position_error_px: float | None) -> str:
    """
    Describe the expected steering direction from tag position alone.

    Args:
        position_error_px: Tag center minus image center in pixels.

    Returns:
        Human-readable expected steering direction.
    """
    if position_error_px is None:
        return "None"

    if position_error_px > 0:
        return "RIGHT"

    if position_error_px < 0:
        return "LEFT"

    return "CENTERED"


def update_perception_compatible(
    perception_manager: PerceptionManager,
) -> tuple[PerceptionState, list[dict]]:
    """
    Update perception using either the new or old PerceptionManager API.

    Args:
        perception_manager: Active perception manager.

    Returns:
        Updated perception state and latest raw detections.
    """
    if hasattr(perception_manager, "update_state"):
        measurement = perception_manager.update_state()
        if hasattr(perception_manager, "get_detections"):
            return measurement, perception_manager.get_detections()

        return measurement, getattr(perception_manager, "last_detections", [])

    detection_start_time = time.monotonic()
    detections = perception_manager.update()
    now = time.monotonic()
    detection_time_ms = (now - detection_start_time) * 1000.0
    measurement = perception_manager.application_state.perception
    frame = perception_manager.last_frame

    if not hasattr(update_perception_compatible, "_fps_window_start"):
        update_perception_compatible._fps_window_start = now
        update_perception_compatible._fps_frame_count = 0
        update_perception_compatible._fps_value = 0.0

    update_perception_compatible._fps_frame_count += 1
    elapsed = now - update_perception_compatible._fps_window_start
    if elapsed >= 1.0:
        update_perception_compatible._fps_value = (
            update_perception_compatible._fps_frame_count / elapsed
        )
        update_perception_compatible._fps_frame_count = 0
        update_perception_compatible._fps_window_start = now

    if frame is None:
        measurement.frame_available = False
        measurement.frame_width = None
        measurement.frame_height = None
        measurement.image_center_x = None
        measurement.image_center_y = None
    else:
        frame_height, frame_width = frame.shape[:2]
        measurement.frame_available = True
        measurement.frame_width = frame_width
        measurement.frame_height = frame_height
        measurement.image_center_x = frame_width / 2
        measurement.image_center_y = frame_height / 2

    measurement.detection_time_ms = detection_time_ms
    measurement.fps = update_perception_compatible._fps_value
    measurement.tag_count = len(detections)
    measurement.tag_ids = [detection["tag_id"] for detection in detections]

    if detections:
        first_detection = detections[0]
        measurement.tag_visible = True
        measurement.tag_id = first_detection["tag_id"]
        measurement.center_x = first_detection["center_x"]
        measurement.center_y = first_detection["center_y"]
        measurement.tag_center_x = first_detection["center_x"]
        measurement.tag_center_y = first_detection["center_y"]
        measurement.orientation_deg = first_detection.get(
            "orientation_deg",
            calculate_detection_orientation_deg(first_detection),
        )
        if measurement.image_center_x is None:
            measurement.position_error_px = None
            measurement.position_error_x = None
        else:
            measurement.position_error_px = (
                first_detection["center_x"] - measurement.image_center_x
            )
            measurement.position_error_x = measurement.position_error_px
    else:
        measurement.tag_visible = False
        measurement.tag_id = None
        measurement.center_x = None
        measurement.center_y = None
        measurement.tag_center_x = None
        measurement.tag_center_y = None
        measurement.position_error_px = None
        measurement.position_error_x = None
        measurement.orientation_deg = None

    return measurement, detections


def draw_overlay(
    frame,
    detection: dict | None,
    measurement: PerceptionState,
    startup_phase: str,
    state: str,
    distance_travelled_cm: float,
    target_distance_cm: float,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    navigation_authority: str,
    vision_lost_count: int,
    fps: float,
    detection_time_ms: float,
    tag_visible: bool,
    position_component_deg: float | None,
    vision_error_deg: float | None,
    expected_steering_direction: str,
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
        startup_phase: STARTUP before motion begins, otherwise RUNTIME.
        state: Current validation state.
        distance_travelled_cm: Estimated distance.
        target_distance_cm: Stop distance.
        reference_heading_deg: Captured heading reference.
        current_heading_deg: Latest IMU heading.
        heading_error_deg: Current heading error.
        navigation_authority: Current steering authority.
        vision_lost_count: Consecutive missed frames before VISION exits.
        fps: Camera FPS from PerceptionManager.
        detection_time_ms: Latest detection update time.
        tag_visible: True when a tag is visible.
        position_component_deg: Shadow position correction component.
        vision_error_deg: Shadow combined vision error.
        expected_steering_direction: Expected steering from tag position.
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
        ("Startup Phase", startup_phase),
        ("State", state),
        ("Distance", f"{distance_travelled_cm:.1f} / {target_distance_cm:.1f} cm"),
        ("Reference Heading", format_display(reference_heading_deg, " deg", signed=True)),
        ("Current Heading", format_display(current_heading_deg, " deg", signed=True)),
        ("Heading Error", format_display(heading_error_deg, " deg", signed=True)),
        ("Authority", navigation_authority),
        ("Vision Lost Count", str(vision_lost_count)),
        ("FPS", f"{fps:.1f}"),
        ("Detection Time", f"{detection_time_ms:.1f} ms"),
        ("Tag Visible", "YES" if tag_visible else "NO"),
        ("Tag ID", str(measurement.tag_id) if measurement.tag_id is not None else "None"),
        ("Tag Orientation", format_display(measurement.orientation_deg, " deg", signed=True)),
        ("Position Error X", format_display(measurement.position_error_x, " px", decimals=0, signed=True)),
        ("Position Component", format_display(position_component_deg, " deg", signed=True)),
        ("Vision Error Shadow", format_display(vision_error_deg, " deg", signed=True)),
        ("Position Steering", expected_steering_direction),
        ("P Term", format_display(pid_result.p_term if pid_result else None, signed=True)),
        ("I Term", format_display(pid_result.i_term if pid_result else None, signed=True)),
        ("D Term", format_display(pid_result.d_term if pid_result else None, signed=True)),
        ("PID Output", pid_text),
        ("Left Frequency", left_frequency),
        ("Right Frequency", right_frequency),
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
    startup_phase: str,
    distance_travelled_cm: float,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    navigation_authority: str,
    vision_lost_count: int,
    fps: float,
    detection_time_ms: float,
    tag_visible: bool,
    tag_id: int | None,
    image_center_x: float | None,
    tag_center_x: float | None,
    orientation_deg: float | None,
    position_error_px: float | None,
    position_component_deg: float | None,
    vision_error_deg: float | None,
    expected_steering_direction: str,
    pid_result: PIDResult | None,
    current_command: str,
) -> None:
    """
    Print PID heading-control diagnostics to the terminal.

    Args:
        startup_phase: STARTUP before motion begins, otherwise RUNTIME.
        distance_travelled_cm: Estimated travel distance.
        reference_heading_deg: Captured heading reference.
        current_heading_deg: Latest IMU heading.
        heading_error_deg: Current heading error.
        navigation_authority: Current steering authority.
        vision_lost_count: Consecutive missed frames before VISION exits.
        fps: Camera FPS from PerceptionManager.
        detection_time_ms: Latest detection update time.
        tag_visible: True when a tag is visible.
        tag_id: Latest visible tag ID.
        image_center_x: Image center x coordinate.
        tag_center_x: Tag center x coordinate.
        orientation_deg: Latest tag orientation.
        position_error_px: Latest tag horizontal position error.
        position_component_deg: Shadow position correction component.
        vision_error_deg: Shadow combined vision error.
        expected_steering_direction: Expected steering from tag position.
        pid_result: Latest PID result.
        current_command: Last transmitted command.
    """
    print("startup_phase:", startup_phase)
    print("Distance Travelled:", f"{distance_travelled_cm:.2f} cm")
    print("Reference Heading:", format_display(reference_heading_deg, " deg", signed=True))
    print("Current Heading:", format_display(current_heading_deg, " deg", signed=True))
    print("Heading Error:", format_display(heading_error_deg, " deg", signed=True))
    print("Navigation Authority:", navigation_authority)
    print("VISION_LOST_COUNT:", vision_lost_count)
    print("FPS:", f"{fps:.1f}")
    print("Detection Time:", f"{detection_time_ms:.1f} ms")
    print("Tag Visible:", "YES" if tag_visible else "NO")
    print("Tag ID:", tag_id if tag_id is not None else "None")
    print("Image Center X:", format_display(image_center_x, " px", decimals=0))
    print("Tag Center X:", format_display(tag_center_x, " px", decimals=0))
    print("Orientation Deg:", format_display(orientation_deg, " deg", signed=True))
    print("Position Error PX:", format_display(position_error_px, " px", decimals=0, signed=True))
    print("Position Component:", format_display(position_component_deg, " deg", signed=True))
    print("Vision Error Shadow:", format_display(vision_error_deg, " deg", signed=True))
    print("Expected Steering Direction:", expected_steering_direction)
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
    diagnostics = getattr(perception_manager.camera_manager, "diagnostics", None)

    configured_resolution = f"{frame_width}x{frame_height}"
    target_fps = 0.0
    pixel_format = str(frame.dtype)
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
    print(f"Image Center X: {frame_width / 2:.1f}")
    print(f"POSITION_GAIN: {POSITION_GAIN:.4f} deg/px")
    print(
        "Position Component Clamp:",
        f"-{VISION_POSITION_COMPONENT_LIMIT_DEG:.1f}..+{VISION_POSITION_COMPONENT_LIMIT_DEG:.1f} deg",
    )
    print()


def get_startup_wait_reason(
    state: str,
    measurement: PerceptionState,
    gate_status: StartGateStatus | None,
    current_heading_deg: float | None,
    imu_status: str,
    imu_ready_received: bool,
    movement_start_time: float | None,
    orientation_tolerance: float,
) -> str:
    """
    Explain which startup condition is currently blocking motion.

    Args:
        state: Current startup/navigation state.
        measurement: Latest perception state.
        gate_status: Latest vision gate status.
        current_heading_deg: Latest IMU heading.
        imu_status: Latest IMU reader status text.
        imu_ready_received: True after IMU_READY was observed.
        movement_start_time: Set after first drive command.
        orientation_tolerance: Required Tag1 orientation tolerance.

    Returns:
        Human-readable startup wait reason.
    """
    if movement_start_time is not None:
        return "Runtime active"

    if state == "WAIT_FOR_TAG1":
        if measurement.tag_id != 1:
            return "Waiting for Tag1"
        return "Tag1 detected, waiting for ALIGN_TAG1 transition"

    if state == "ALIGN_TAG1":
        if measurement.orientation_deg is None:
            return "Waiting for valid Tag1 orientation"
        if abs(measurement.orientation_deg) > orientation_tolerance:
            return "Waiting for Tag1 orientation alignment"
        return "Tag1 aligned, waiting for vision gate transition"

    if state == "VISION_START_GATE":
        if gate_status is None:
            return "Waiting for first vision gate evaluation"
        if not gate_status.orientation_pass:
            return "Vision gate waiting: orientation not stable"
        if not gate_status.position_pass:
            return "Vision gate waiting: position not centered"
        if gate_status.status_text != "READY_TO_MOVE":
            return "Vision gate waiting for 0.5 s stable lock"
        return "Vision gate ready, waiting to send CALIBRATE_IMU"

    if state == "CALIBRATE_IMU":
        if current_heading_deg is None:
            return (
                "Waiting for HEADING value after CALIBRATE_IMU; "
                f"IMU_READY seen={imu_ready_received}, status={imu_status}"
            )
        return "Calibration complete, waiting for heading capture transition"

    if state == "HEADING_HOLD_75CM":
        return "Heading reference captured, waiting for first SET_DRIVE"

    return "No startup wait"


def build_startup_conditions(
    measurement: PerceptionState,
    gate_status: StartGateStatus | None,
    imu_ready_received: bool,
    current_heading_deg: float | None,
) -> dict[str, bool]:
    """
    Build individual startup condition booleans for diagnostics.

    Args:
        measurement: Latest perception state.
        gate_status: Latest vision gate status.
        imu_ready_received: True after IMU_READY was observed.
        current_heading_deg: Latest parsed heading.

    Returns:
        Named startup conditions.
    """
    condition_tag_visible = measurement.tag_visible and measurement.tag_id == 1
    condition_orientation_valid = measurement.orientation_deg is not None
    condition_position_valid = (
        measurement.position_error_x is not None
        and abs(measurement.position_error_x) <= 10.0
    )
    condition_imu_ready = imu_ready_received
    condition_heading_available = current_heading_deg is not None
    condition_vision_gate_passed = (
        gate_status is not None and gate_status.status_text == "READY_TO_MOVE"
    )
    condition_calibration_complete = condition_heading_available

    return {
        "condition_tag_visible": condition_tag_visible,
        "condition_orientation_valid": condition_orientation_valid,
        "condition_position_valid": condition_position_valid,
        "condition_imu_ready": condition_imu_ready,
        "condition_heading_available": condition_heading_available,
        "condition_vision_gate_passed": condition_vision_gate_passed,
        "condition_calibration_complete": condition_calibration_complete,
    }


def first_failed_startup_condition(
    state: str,
    conditions: dict[str, bool],
) -> str:
    """
    Return the first condition currently blocking the active startup state.

    Args:
        state: Current startup state.
        conditions: Startup condition dictionary.

    Returns:
        Blocking condition name, or NONE.
    """
    state_conditions = {
        "WAIT_FOR_TAG1": ["condition_tag_visible"],
        "ALIGN_TAG1": [
            "condition_tag_visible",
            "condition_orientation_valid",
        ],
        "VISION_START_GATE": [
            "condition_tag_visible",
            "condition_orientation_valid",
            "condition_position_valid",
            "condition_vision_gate_passed",
        ],
        "CALIBRATE_IMU": [
            "condition_heading_available",
            "condition_calibration_complete",
        ],
        "HEADING_HOLD_75CM": [],
    }

    for condition_name in state_conditions.get(state, []):
        if not conditions[condition_name]:
            return condition_name

    return "NONE"


def find_source_line(pattern: str) -> int | None:
    """
    Find the first source line containing a pattern in this file.

    Args:
        pattern: Source text to search for.

    Returns:
        One-based line number, or None if not found.
    """
    source_path = Path(__file__)
    try:
        for line_number, line_text in enumerate(source_path.read_text().splitlines(), 1):
            if "find_source_line" in line_text:
                continue
            if pattern in line_text:
                return line_number
    except OSError:
        return None

    return None


def log_startup_diagnostics(
    state: str,
    startup_lock_count: int,
    calibration_complete: bool,
    navigation_authority: str,
    reason_startup_waiting: str,
    startup_conditions: dict[str, bool],
    blocked_condition: str,
    blocking_line: int | None,
    measurement: PerceptionState,
    gate_status: StartGateStatus | None,
    imu_status: str,
    imu_ready_received: bool,
    current_heading_deg: float | None,
    reference_heading_deg: float | None,
    last_imu_message: str,
    heading_message_count: int,
    startup_tag1_orientation_deg: float | None,
) -> None:
    """
    Print one startup diagnostic block.

    Args:
        state: Current startup/navigation state.
        startup_lock_count: Number of one-second startup diagnostic samples.
        calibration_complete: True when a valid heading is available.
        navigation_authority: Current navigation authority.
        reason_startup_waiting: Human-readable wait reason.
        startup_conditions: Individual startup booleans.
        blocked_condition: First failed condition for the current state.
        blocking_line: Source line for the active blocking transition.
        measurement: Latest perception state.
        gate_status: Latest vision gate status.
        imu_status: Latest IMU reader status.
        imu_ready_received: True after IMU_READY was observed.
        current_heading_deg: Latest parsed heading.
        reference_heading_deg: Captured navigation reference.
        last_imu_message: Last raw serial message parsed by the IMU reader.
        heading_message_count: Number of HEADING messages parsed.
        startup_tag1_orientation_deg: Tag1 orientation captured at gate pass.
    """
    print("startup_state:", state)
    print("startup_lock_count:", startup_lock_count)
    print("imu_ready_received:", imu_ready_received)
    print("current_heading_deg:", format_display(current_heading_deg, " deg", signed=True))
    print("reference_heading_deg:", format_display(reference_heading_deg, " deg", signed=True))
    print("calibration_complete:", calibration_complete)
    print("calibration_rule:", "current_heading_deg is not None")
    print("navigation_authority:", navigation_authority)
    print("reason_startup_waiting:", reason_startup_waiting)
    for condition_name, condition_value in startup_conditions.items():
        print(f"{condition_name}:", condition_value)
    if blocked_condition != "NONE":
        print("STARTUP BLOCKED BY:", blocked_condition)
        if blocking_line is not None:
            print("BLOCKING LINE:", blocking_line)
    print("imu_status:", imu_status)
    print("last_imu_message:", last_imu_message or "None")
    print("heading_message_count:", heading_message_count)
    print("tag_visible:", "YES" if measurement.tag_visible else "NO")
    print("tag_id:", measurement.tag_id if measurement.tag_id is not None else "None")
    print("tag_orientation:", format_display(measurement.orientation_deg, " deg", signed=True))
    print(
        "startup_tag1_orientation:",
        format_display(startup_tag1_orientation_deg, " deg", signed=True),
    )
    print("position_error:", format_display(measurement.position_error_x, " px", decimals=0, signed=True))
    if gate_status is not None:
        print("vision_gate_status:", gate_status.status_text)
        print("vision_gate_stable_time:", f"{gate_status.stable_time_sec:.2f} s")
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
    pid_csv_path = Path(args.csv).expanduser().resolve()
    tag_visibility_csv_path = (
        Path(args.tag_visibility_csv).expanduser().resolve()
        if args.tag_visibility_csv is not None
        else pid_csv_path.with_name(f"{pid_csv_path.stem}_tag_visibility.csv")
    )
    logger = PIDDistanceLogger(pid_csv_path)
    tag_visibility_logger = TagVisibilityLogger(tag_visibility_csv_path)
    tag_visibility_tracker = TagVisibilityTracker(tag_visibility_logger)

    state = "WAIT_FOR_TAG1"
    current_command = "NONE"
    gate_status: StartGateStatus | None = None
    gate_stable_since: float | None = None
    navigation_reference_heading_deg: float | None = None
    latest_pid_result: PIDResult | None = None
    last_pid_update_time = 0.0
    last_log_time = 0.0
    last_startup_diagnostics_time = 0.0
    last_csv_time = 0.0
    start_time = time.monotonic()
    movement_start_time: float | None = None
    navigation_authority = "NONE"
    vision_lost_count = 0
    last_vision_error_deg: float | None = None
    startup_tag1_orientation_deg: float | None = None
    last_logged_tag_id: int | None = None
    vision_authority_tag_id: int | None = None
    summary_printed = False
    processing_summary_printed = False
    startup_lock_count = 0
    imu_ready_received = False
    imu_ready_logged = False
    calibration_complete_logged = False
    heading_error_samples: list[float] = []
    pid_output_samples: list[float] = []
    left_frequency_samples: list[float] = []
    right_frequency_samples: list[float] = []
    current_heading_samples: list[float] = []

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        logger.close()
        tag_visibility_logger.close()
        return

    if not serial_controller.connect():
        logger.close()
        tag_visibility_logger.close()
        perception_manager.release()
        return

    print("ENTER WAIT_FOR_TAG1")

    try:
        while True:
            measurement, detections = update_perception_compatible(perception_manager)
            frame = perception_manager.last_frame
            detection = detections[0] if detections else None
            tag_visible = measurement.tag_visible

            if (
                not processing_summary_printed
                and frame is not None
                and measurement.fps > 0.0
            ):
                print_processing_startup_summary(
                    perception_manager,
                    frame,
                    measurement.fps,
                    measurement.detection_time_ms,
                )
                processing_summary_printed = True

            imu_reader.update_from_connection(serial_controller.serial_connection)
            if imu_reader.latest_status == "IMU_READY":
                imu_ready_received = True
                if not imu_ready_logged:
                    print("IMU_READY RECEIVED")
                    imu_ready_logged = True
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
            position_component_deg = calculate_position_component_deg(
                measurement.position_error_x,
            )
            vision_error_deg = calculate_combined_vision_error_deg(
                measurement.orientation_deg,
                position_component_deg,
            )
            expected_steering_direction = expected_position_steering_direction(
                measurement.position_error_x,
            )

            key = cv2.waitKey(1) & 0xFF
            startup_phase = "RUNTIME" if movement_start_time is not None else "STARTUP"
            calibration_complete = current_heading_deg is not None
            startup_conditions = build_startup_conditions(
                measurement,
                gate_status,
                imu_ready_received,
                current_heading_deg,
            )
            blocked_condition = first_failed_startup_condition(
                state,
                startup_conditions,
            )
            blocking_line = None
            if state == "CALIBRATE_IMU" and blocked_condition != "NONE":
                blocking_line = find_source_line("                if calibration_complete:")

            if (
                startup_phase == "STARTUP"
                and state not in {"STOPPED", "DISTANCE_COMPLETE"}
                and now - last_startup_diagnostics_time >= 1.0
            ):
                startup_lock_count += 1
                reason_startup_waiting = get_startup_wait_reason(
                    state,
                    measurement,
                    gate_status,
                    current_heading_deg,
                    imu_reader.latest_status,
                    imu_ready_received,
                    movement_start_time,
                    args.orientation_tolerance,
                )
                log_startup_diagnostics(
                    state,
                    startup_lock_count,
                    calibration_complete,
                    navigation_authority,
                    reason_startup_waiting,
                    startup_conditions,
                    blocked_condition,
                    blocking_line,
                    measurement,
                    gate_status,
                    imu_reader.latest_status,
                    imu_ready_received,
                    current_heading_deg,
                    navigation_reference_heading_deg,
                    getattr(imu_reader, "last_message", ""),
                    getattr(imu_reader, "heading_message_count", 0),
                    startup_tag1_orientation_deg,
                )
                last_startup_diagnostics_time = now

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
                print("ENTER ALIGN_TAG1")

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
                    print("ENTER VISION_START_GATE")

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
                    startup_tag1_orientation_deg = measurement.orientation_deg
                    current_command = "CALIBRATE_IMU"
                    transmit_command(serial_controller, current_command, start_time, movement_start_time)
                    imu_reader.latest_heading_deg = None
                    imu_reader.latest_status = "IMU_CALIBRATING"
                    imu_ready_received = False
                    imu_ready_logged = False
                    calibration_complete_logged = False
                    navigation_reference_heading_deg = None
                    state = "CALIBRATE_IMU"
                    print("ENTER CALIBRATE_IMU")

            elif state == "CALIBRATE_IMU":
                print(
                    "CALIBRATE_IMU CHECK:",
                    f"imu_ready_received={imu_ready_received}",
                    f"current_heading_deg={current_heading_deg}",
                    f"calibration_complete={calibration_complete}",
                    f"condition_imu_ready={startup_conditions['condition_imu_ready']}",
                    f"condition_heading_available={startup_conditions['condition_heading_available']}",
                    f"condition_calibration_complete={startup_conditions['condition_calibration_complete']}",
                )
                if calibration_complete:
                    if not calibration_complete_logged:
                        print("CALIBRATION COMPLETE")
                        calibration_complete_logged = True
                    navigation_reference_heading_deg = current_heading_deg
                    heading_error_deg = 0.0
                    pid_controller.reset()
                    distance_estimate.reset()
                    latest_pid_result = None
                    last_pid_update_time = 0.0
                    movement_start_time = None
                    vision_lost_count = 0
                    last_vision_error_deg = None
                    navigation_authority = "NONE"
                    print(
                        "NAVIGATION REFERENCE HEADING CAPTURED:",
                        format_display(
                            navigation_reference_heading_deg,
                            " deg",
                            signed=True,
                        ),
                    )
                    state = "HEADING_HOLD_75CM"
                    print("ENTER HEADING_HOLD_75CM")
                else:
                    print("STARTUP BLOCKED")
                    print(f"imu_ready_received={imu_ready_received}")
                    print(f"current_heading_deg={current_heading_deg}")
                    print(f"calibration_complete={calibration_complete}")

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
                runtime_vision_persistence_enabled = movement_start_time is not None

                if not runtime_vision_persistence_enabled:
                    vision_lost_count = 0
                    last_vision_error_deg = None
                    if (
                        navigation_reference_heading_deg is not None
                        and current_heading_deg is not None
                    ):
                        selected_authority = "IMU"
                    else:
                        selected_authority = "NONE"
                elif tag_has_valid_orientation:
                    vision_lost_count = 0
                    last_vision_error_deg = measurement.orientation_deg
                    selected_authority = "VISION"
                elif navigation_authority == "VISION":
                    vision_lost_count += 1
                    if vision_lost_count < VISION_LOST_FRAME_THRESHOLD:
                        selected_authority = "VISION"
                    elif (
                        navigation_reference_heading_deg is not None
                        and current_heading_deg is not None
                    ):
                        selected_authority = "IMU"
                    else:
                        selected_authority = "NONE"
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
                        last_vision_error_deg = None

                    navigation_authority = selected_authority
                    if navigation_authority != "NONE":
                        pid_controller.reset()
                    print("NAVIGATION AUTHORITY:", navigation_authority)
                    print("VISION_LOST_COUNT:", vision_lost_count)

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
                        selected_error_deg = (
                            measurement.orientation_deg
                            if measurement.orientation_deg is not None
                            else last_vision_error_deg
                        )
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
                        if movement_start_time is None:
                            print("FIRST SET_DRIVE COMMAND")
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

            startup_phase = "RUNTIME" if movement_start_time is not None else "STARTUP"
            tag_visibility_tracker.update(
                detections,
                elapsed_time_sec,
                distance_estimate.travelled_cm,
                measurement.detection_time_ms,
                measurement.fps,
            )

            if (now - last_csv_time) >= args.csv_interval:
                logger.write(
                    elapsed_time_sec,
                    distance_estimate.travelled_cm,
                    navigation_reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    navigation_authority,
                    vision_lost_count,
                    measurement.fps,
                    measurement.detection_time_ms,
                    tag_visible,
                    measurement.tag_id,
                    # Same orientation value shown on the OpenCV overlay.
                    measurement.orientation_deg,
                    measurement.position_error_x,
                    position_component_deg,
                    vision_error_deg,
                    latest_pid_result,
                )
                last_csv_time = now

            if (now - last_log_time) >= args.log_interval:
                log_terminal(
                    startup_phase,
                    distance_estimate.travelled_cm,
                    navigation_reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    navigation_authority,
                    vision_lost_count,
                    measurement.fps,
                    measurement.detection_time_ms,
                    tag_visible,
                    measurement.tag_id,
                    measurement.image_center_x,
                    measurement.tag_center_x,
                    measurement.orientation_deg,
                    measurement.position_error_x,
                    position_component_deg,
                    vision_error_deg,
                    expected_steering_direction,
                    latest_pid_result,
                    current_command,
                )
                last_log_time = now

            if frame is not None:
                draw_overlay(
                    frame,
                    detection,
                    measurement,
                    startup_phase,
                    state,
                    distance_estimate.travelled_cm,
                    args.travel_distance_cm,
                    navigation_reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    navigation_authority,
                    vision_lost_count,
                    measurement.fps,
                    measurement.detection_time_ms,
                    tag_visible,
                    position_component_deg,
                    vision_error_deg,
                    expected_steering_direction,
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
        tag_visibility_tracker.write_summary_once()
        transmit_command(serial_controller, "STOP", start_time, movement_start_time)
        logger.close()
        tag_visibility_logger.close()
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
    parser.add_argument(
        "--tag-visibility-csv",
        default=None,
        help="CSV file for per-tag visibility diagnostics.",
    )
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
