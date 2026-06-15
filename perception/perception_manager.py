import time
from typing import Any

import numpy as np

from core.application_state import ApplicationState
from core.perception_state import PerceptionState
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
        self.last_detections: list[dict[str, Any]] = []
        self.last_state: PerceptionState = self.application_state.perception
        self._fps_window_start = time.monotonic()
        self._fps_frame_count = 0

    def initialize(self) -> bool:
        """
        Initialize the camera used by the perception layer.

        Returns:
            True if the camera initializes successfully, otherwise False.
        """
        return self.camera_manager.initialize()

    def update_state(self) -> PerceptionState:
        """
        Capture one frame, detect AprilTags, and return PerceptionState.

        Returns:
            Updated perception state.
        """
        # Get one image from the camera.
        frame = self.camera_manager.get_frame()
        if frame is None:
            self.last_frame = None
            self.last_detections = []
            self._update_application_state(
                detections=[],
                frame=None,
                frame_available=False,
            )
            return self.last_state

        self.last_frame = frame
        self._record_frame_for_fps()

        # Run AprilTag detection on the latest camera frame.
        detections = self.apriltag_detector.detect(frame)
        self.last_detections = detections
        self._update_application_state(
            detections=detections,
            frame=frame,
            frame_available=True,
        )
        return self.last_state

    def update(self) -> list[dict[str, Any]]:
        """
        Capture one frame and return AprilTag detections.

        This method is kept for older validation scripts. New navigation code
        should prefer update_state() and consume ApplicationState.perception.

        Returns:
            A list of AprilTag detections.
        """
        self.update_state()
        return self.last_detections

    def get_state(self) -> PerceptionState:
        """
        Return the latest perception state without capturing a new frame.

        Returns:
            Latest PerceptionState.
        """
        return self.last_state

    def get_detections(self) -> list[dict[str, Any]]:
        """
        Return the latest raw detection dictionaries for overlay drawing only.

        Returns:
            Latest AprilTag detections.
        """
        return self.last_detections

    def release(self) -> None:
        """Release all perception resources."""
        self.camera_manager.release()
        self.last_frame = None
        self.last_detections = []

    def _record_frame_for_fps(self) -> None:
        """Update measured perception FPS using a one-second rolling window."""
        self._fps_frame_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_window_start

        if elapsed < 1.0:
            return

        self.application_state.perception.fps = self._fps_frame_count / elapsed
        self._fps_frame_count = 0
        self._fps_window_start = now

    def _update_application_state(
        self,
        detections: list[dict[str, Any]],
        frame: np.ndarray | None,
        frame_available: bool,
    ) -> None:
        """
        Store the latest perception result in ApplicationState.

        Args:
            detections: AprilTag detections returned by AprilTagDetector.
            frame: Latest captured camera frame.
            frame_available: True when a frame was captured successfully.
        """
        perception = self.application_state.perception
        self.last_state = perception
        perception.frame_available = frame_available
        perception.detection_time_ms = self.apriltag_detector.detection_time_ms
        perception.tag_count = len(detections)
        perception.tag_ids = [detection["tag_id"] for detection in detections]

        if frame is not None:
            frame_height, frame_width = frame.shape[:2]
            perception.frame_width = frame_width
            perception.frame_height = frame_height
            perception.image_center_x = frame_width / 2
            perception.image_center_y = frame_height / 2
        else:
            frame_width = None
            perception.frame_width = None
            perception.frame_height = None
            perception.image_center_x = None
            perception.image_center_y = None

        if not detections:
            perception.tag_visible = False
            perception.tag_id = None
            perception.center_x = None
            perception.center_y = None
            perception.tag_center_x = None
            perception.tag_center_y = None
            perception.position_error_px = None
            perception.position_error_x = None
            perception.orientation_deg = None
            return

        # For now, store the first detected tag as the latest perception result.
        first_detection = detections[0]
        perception.tag_visible = True
        perception.tag_id = first_detection["tag_id"]
        perception.center_x = first_detection["center_x"]
        perception.center_y = first_detection["center_y"]
        perception.tag_center_x = first_detection["center_x"]
        perception.tag_center_y = first_detection["center_y"]
        perception.orientation_deg = first_detection.get("orientation_deg")

        if frame_width is None:
            perception.position_error_px = None
            perception.position_error_x = None
        else:
            perception.position_error_px = first_detection["center_x"] - (frame_width / 2)
            perception.position_error_x = perception.position_error_px
