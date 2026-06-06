from typing import Any

import numpy as np

from perception.apriltag_detector import AprilTagDetector
from perception.camera_manager import CameraManager


class PerceptionManager:
    """Single entry point for the AGV perception layer."""

    def __init__(self) -> None:
        """Create the camera and detector used by perception."""
        self.camera_manager = CameraManager()
        self.apriltag_detector = AprilTagDetector()
        self.last_frame: np.ndarray | None = None

    def initialize(self) -> bool:
        """
        Initialize the camera used by the perception layer.

        Returns:
            True if the camera initializes successfully, otherwise False.
        """
        return self.camera_manager.initialize()

    def update(self) -> list[dict[str, Any]]:
        """
        Capture one frame and run AprilTag detection on it.

        Returns:
            A list of AprilTag detections. Returns an empty list if no frame is
            available or no tags are detected.
        """
        # Get one image from the camera.
        frame = self.camera_manager.get_frame()
        if frame is None:
            self.last_frame = None
            return []

        self.last_frame = frame

        # Run AprilTag detection on the latest camera frame.
        detections = self.apriltag_detector.detect(frame)
        return detections

    def release(self) -> None:
        """Release all perception resources."""
        self.camera_manager.release()
        self.last_frame = None
