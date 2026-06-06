from typing import Any

import numpy as np

from core.application_state import ApplicationState
from perception.apriltag_detector import AprilTagDetector
from perception.camera_manager import CameraManager



class PerceptionManager:
    """Single entry point for the AGV perception layer."""

    def __init__(self, application_state: ApplicationState) -> None:
        """
        Create the camera and detector used by perception.

        Args:
            application_state: Shared application state updated by perception.
        """
        self.application_state = application_state
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
            self._update_application_state([])
            return []

        self.last_frame = frame

        # Run AprilTag detection on the latest camera frame.
        detections = self.apriltag_detector.detect(frame)
        self._update_application_state(detections)
        return detections

    def release(self) -> None:
        """Release all perception resources."""
        self.camera_manager.release()
        self.last_frame = None

    def _update_application_state(self, detections: list[dict[str, Any]]) -> None:
        """
        Store the latest perception result in ApplicationState.

        Args:
            detections: AprilTag detections returned by AprilTagDetector.
        """
        perception = self.application_state.perception

        if not detections:
            perception.tag_visible = False
            perception.tag_id = None
            perception.center_x = None
            perception.center_y = None
            return

        # For now, store the first detected tag as the latest perception result.
        first_detection = detections[0]
        perception.tag_visible = True
        perception.tag_id = first_detection["tag_id"]
        perception.center_x = first_detection["center_x"]
        perception.center_y = first_detection["center_y"]
