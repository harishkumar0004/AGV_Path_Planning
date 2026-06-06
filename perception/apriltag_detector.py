from typing import Any

import cv2
import numpy as np
from pupil_apriltags import Detector


class AprilTagDetector:
    """Detects AprilTags in camera frames."""

    def __init__(self, tag_family: str = "tag36h11") -> None:
        """
        Create an AprilTag detector for the requested tag family.

        Args:
            tag_family: AprilTag family name used by pupil_apriltags.
        """
        self.detector = Detector(families=tag_family)

    def detect(self, frame: np.ndarray) -> list[dict[str, Any]]:
        """
        Detect AprilTags in one camera frame.

        Args:
            frame: Color or grayscale image from CameraManager.

        Returns:
            A list of dictionaries containing tag ID, center, and corners.
        """
        # AprilTag detection works on grayscale images.
        gray_frame = self._convert_to_grayscale(frame)

        # Run pupil_apriltags detection on the grayscale image.
        raw_detections = self.detector.detect(gray_frame)

        detections: list[dict[str, Any]] = []
        for detection in raw_detections:
            center_x = float(detection.center[0])
            center_y = float(detection.center[1])
            corners = [
                (float(corner[0]), float(corner[1]))
                for corner in detection.corners
            ]

            detections.append(
                {
                    "tag_id": int(detection.tag_id),
                    "center_x": center_x,
                    "center_y": center_y,
                    "corners": corners,
                }
            )

        return detections

    def _convert_to_grayscale(self, frame: np.ndarray) -> np.ndarray:
        """
        Convert a color frame to grayscale if needed.

        Args:
            frame: Color or grayscale image.

        Returns:
            Grayscale image.
        """
        if len(frame.shape) == 2:
            return frame

        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)

        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
