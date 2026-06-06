from dataclasses import dataclass


@dataclass
class PerceptionState:
    """Stores the latest AprilTag information seen by perception."""

    # True when at least one AprilTag is visible in the latest frame.
    tag_visible: bool = False

    # ID of the first visible AprilTag, or None when no tag is visible.
    tag_id: int | None = None

    # X coordinate of the visible tag center, or None when no tag is visible.
    center_x: float | None = None

    # Y coordinate of the visible tag center, or None when no tag is visible.
    center_y: float | None = None
