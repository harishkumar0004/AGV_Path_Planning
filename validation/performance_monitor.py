import time
import csv
from dataclasses import dataclass
from pathlib import Path

from navigation.motion_commands import MotionCommand
from validation.navigation_metrics import NavigationMetrics


@dataclass
class DriftRecord:
    """Stores heading measured when a route tag is detected."""

    tag_id: int
    heading_deg: float
    timestamp_sec: float


@dataclass
class DriftSegment:
    """Stores heading drift between two route tags."""

    start_tag_id: int
    end_tag_id: int
    drift_deg: float


class PerformanceMonitor:
    """Measures camera, detection, tag, and command timing performance."""

    def __init__(self, fps_window_sec: float = 1.0) -> None:
        """
        Create a performance monitor.

        Args:
            fps_window_sec: Time window used for FPS calculations.
        """
        self.metrics = NavigationMetrics()
        self.fps_window_sec = fps_window_sec
        self._camera_window_start = time.monotonic()
        self._detection_window_start = time.monotonic()
        self._camera_frame_count = 0
        self._detection_frame_count = 0
        self._current_detection_time: float | None = None
        self._command_generated_time: float | None = None
        self._logged_tags: set[int] = set()
        self._start_time = time.monotonic()
        self._drift_start_time: float | None = None
        self._drift_records: list[DriftRecord] = []
        self._drift_segments: list[DriftSegment] = []
        self._drift_logged_tags: set[int] = set()

    def record_camera_frame(self) -> None:
        """Record one camera frame and update camera FPS."""
        self._camera_frame_count += 1
        now = time.monotonic()
        elapsed = now - self._camera_window_start

        if elapsed >= self.fps_window_sec:
            self.metrics.camera_fps = self._camera_frame_count / elapsed
            self._camera_frame_count = 0
            self._camera_window_start = now

    def record_detection_cycle(self) -> None:
        """Record one perception/detection update and update detection FPS."""
        self._detection_frame_count += 1
        now = time.monotonic()
        elapsed = now - self._detection_window_start

        if elapsed >= self.fps_window_sec:
            self.metrics.detection_fps = self._detection_frame_count / elapsed
            self._detection_frame_count = 0
            self._detection_window_start = now

    def record_tag_detection(self, tag_id: int | None) -> None:
        """
        Record the currently visible tag.

        Args:
            tag_id: Visible AprilTag ID, or None when no tag is visible.
        """
        self.metrics.current_tag_id = tag_id
        self._current_detection_time = time.monotonic()

        if tag_id is None:
            return

        self.metrics.tag_detection_counts[tag_id] = (
            self.metrics.tag_detection_counts.get(tag_id, 0) + 1
        )

        if tag_id not in self._logged_tags:
            self._logged_tags.add(tag_id)
            elapsed = self._current_detection_time - self._start_time
            print(f"Tag {tag_id} detected")
            print(f"Time: {elapsed:.2f} s")

    def record_navigation_state(self, state_name: str, action_name: str) -> None:
        """
        Store latest navigation state and action labels.

        Args:
            state_name: Current NavigationState name.
            action_name: Current command/action name.
        """
        self.metrics.navigation_state = state_name
        self.metrics.current_action = action_name

    def record_alignment_error(self, error_px: float | None) -> None:
        """
        Store the latest horizontal turn-tag alignment error.

        Args:
            error_px: Pixel error from image center, or None when unavailable.
        """
        self.metrics.alignment_error_px = error_px

    def record_heading(self, heading_deg: float | None) -> None:
        """
        Store latest IMU heading.

        Args:
            heading_deg: Relative heading from ESP32, or None before ready.
        """
        self.metrics.current_heading_deg = heading_deg

    def record_tag_heading(
        self,
        tag_id: int | None,
        heading_deg: float | None,
        csv_path: Path,
    ) -> None:
        """
        Capture heading once for each route tag and write it to CSV.

        Args:
            tag_id: Detected route tag.
            heading_deg: Current IMU relative heading.
            csv_path: CSV file destination.
        """
        if tag_id is None or heading_deg is None:
            return

        if tag_id in self._drift_logged_tags:
            return

        now = time.monotonic()
        if self._drift_start_time is None:
            self._drift_start_time = now

        timestamp_sec = now - self._drift_start_time
        record = DriftRecord(
            tag_id=tag_id,
            heading_deg=heading_deg,
            timestamp_sec=timestamp_sec,
        )
        self._drift_records.append(record)
        self._drift_logged_tags.add(tag_id)

        self._append_drift_csv(csv_path, record)

        print(f"Detected Tag: {tag_id}")
        print(f"Heading: {heading_deg:.2f} deg")
        print(f"Time: {timestamp_sec:.2f} s")

        if len(self._drift_records) < 2:
            return

        previous = self._drift_records[-2]
        drift = self._wrap_angle_deg(heading_deg - previous.heading_deg)
        segment = DriftSegment(
            start_tag_id=previous.tag_id,
            end_tag_id=tag_id,
            drift_deg=drift,
        )
        self._drift_segments.append(segment)
        self.metrics.latest_drift_deg = drift

        print(
            f"Drift Tag{previous.tag_id}->Tag{tag_id}: "
            f"{drift:+.2f} deg"
        )

    def record_command_generated(self, command: MotionCommand) -> None:
        """
        Record the time when a command was generated.

        Args:
            command: Command generated by navigation.
        """
        self._command_generated_time = time.monotonic()
        self.metrics.current_action = command.name

    def record_command_transmitted(self) -> None:
        """Record the time when the generated command was transmitted."""
        if self._current_detection_time is None:
            return

        transmitted_time = time.monotonic()
        self.metrics.command_latency_ms = (
            transmitted_time - self._current_detection_time
        ) * 1000.0

    def print_drift_summary(self) -> None:
        """Print route heading drift summary."""
        if not self._drift_segments:
            print("Route Summary")
            print("No drift segments recorded.")
            return

        total_drift = sum(segment.drift_deg for segment in self._drift_segments)
        average_drift = total_drift / len(self._drift_segments)
        maximum_drift = max(
            self._drift_segments,
            key=lambda segment: abs(segment.drift_deg),
        )

        print("Route Summary")
        for segment in self._drift_segments:
            print(
                f"Tag{segment.start_tag_id}->Tag{segment.end_tag_id} "
                f"Drift: {segment.drift_deg:+.2f} deg"
            )

        print(f"Total Drift: {total_drift:+.2f} deg")
        print(f"Average Drift: {average_drift:+.2f} deg")
        print(
            f"Maximum Drift: {maximum_drift.drift_deg:+.2f} deg "
            f"(Tag{maximum_drift.start_tag_id}->"
            f"Tag{maximum_drift.end_tag_id})"
        )

    def initialize_drift_csv(self, csv_path: Path) -> None:
        """
        Reset the drift CSV for a fresh validation run.

        Args:
            csv_path: CSV destination.
        """
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["tag_id", "heading_deg", "timestamp_sec"])

    def get_tag_count_text(self) -> str:
        """
        Return compact text showing per-tag frame counts.

        Returns:
            Human-readable tag count summary.
        """
        if not self.metrics.tag_detection_counts:
            return "Tag Counts: None"

        parts = [
            f"Tag {tag_id}: {count}"
            for tag_id, count in sorted(self.metrics.tag_detection_counts.items())
        ]
        return " | ".join(parts)

    def _append_drift_csv(self, csv_path: Path, record: DriftRecord) -> None:
        """
        Append one heading record to the drift CSV file.

        Args:
            csv_path: CSV destination.
            record: Heading record to write.
        """
        with csv_path.open("a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    record.tag_id,
                    f"{record.heading_deg:.2f}",
                    f"{record.timestamp_sec:.2f}",
                ]
            )

    def _wrap_angle_deg(self, angle_deg: float) -> float:
        """
        Wrap angle to -180..180 degrees.

        Args:
            angle_deg: Angle in degrees.

        Returns:
            Wrapped angle in degrees.
        """
        while angle_deg > 180.0:
            angle_deg -= 360.0

        while angle_deg < -180.0:
            angle_deg += 360.0

        return angle_deg
