from dataclasses import dataclass, field


@dataclass
class NavigationMetrics:
    """Stores runtime measurements for AGV navigation validation."""

    # Camera frame rate measured from camera loop iterations.
    camera_fps: float = 0.0

    # AprilTag processing rate measured from perception updates.
    detection_fps: float = 0.0

    # Most recent AprilTag detection/perception update duration.
    detection_time_ms: float = 0.0

    # Number of frames where each tag ID was visible.
    tag_detection_counts: dict[int, int] = field(default_factory=dict)

    # Most recent command latency in milliseconds.
    command_latency_ms: float | None = None

    # Latest visible tag ID.
    current_tag_id: int | None = None

    # Latest navigation state text.
    navigation_state: str = "WAIT_FOR_START"

    # Latest route or motion action text.
    current_action: str = "None"

    # Latest horizontal alignment error in pixels.
    alignment_error_px: float | None = None

    # Latest IMU relative heading received from ESP32.
    current_heading_deg: float | None = None

    # Latest measured drift between two detected route tags.
    latest_drift_deg: float | None = None
