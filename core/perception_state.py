from dataclasses import dataclass


@dataclass
class PerceptionState:
    """Stores the latest camera and AprilTag information seen by perception."""

    # True when a camera frame was captured successfully.
    frame_available: bool = False

    # Width and height of the latest captured camera frame.
    frame_width: int | None = None
    frame_height: int | None = None

    # Image center of the latest frame.
    image_center_x: float | None = None
    image_center_y: float | None = None

    # True when at least one AprilTag is visible in the latest frame.
    tag_visible: bool = False

    # Number of visible tags in the latest frame.
    tag_count: int = 0

    # ID of the first visible AprilTag, or None when no tag is visible.
    tag_id: int | None = None

    # X coordinate of the visible tag center, or None when no tag is visible.
    center_x: float | None = None

    # Y coordinate of the visible tag center, or None when no tag is visible.
    center_y: float | None = None

    # More explicit aliases used by newer navigation validation code.
    tag_center_x: float | None = None
    tag_center_y: float | None = None

    # Horizontal tag offset from the image center in pixels.
    position_error_px: float | None = None

    # Backward-compatible alias used by older validation helpers.
    position_error_x: float | None = None

    # AprilTag orientation calculated from corner[0] to corner[1].
    orientation_deg: float | None = None

    # Most recent AprilTag detection duration.
    detection_time_ms: float = 0.0

    # Measured perception update rate.
    fps: float = 0.0

    # Latest visible tag IDs.
    tag_ids: list[int] | None = None
