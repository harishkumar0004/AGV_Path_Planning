import math
import time
from typing import Any

import cv2
import numpy as np
from pupil_apriltags import Detector


class AprilTagDetector:
    """Detects AprilTags in camera frames."""

    def __init__(
        self,
        tag_family: str = "tag36h11",
        nthreads: int = 4,
        quad_decimate: float = 2.0,
        quad_sigma: float = 0.0,
    ) -> None:
        """
        Create an AprilTag detector for the requested tag family.

        Args:
            tag_family: AprilTag family name used by pupil_apriltags.
            nthreads: Worker threads used by the detector.
            quad_decimate: Image decimation factor for faster detection.
            quad_sigma: Blur sigma before quad detection.
        """
        # self.detector = Detector(
        #     families=tag_family,
        #     nthreads=nthreads,
        #     quad_decimate=quad_decimate,
        #     quad_sigma=quad_sigma,
        # )
        self.detector = Detector(
            families=tag_family,
            nthreads=4,
            quad_decimate=3.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )
        self._gray_frame: np.ndarray | None = None
        self.detection_time_ms = 0.0
        self.tag_count = 0
        self.tag_ids: list[int] = []

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
        start_time = time.monotonic()
        raw_detections = self.detector.detect(gray_frame)
        self.detection_time_ms = (time.monotonic() - start_time) * 1000.0

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
                    "orientation_deg": self._calculate_orientation_deg(corners),
                }
            )

        self.tag_count = len(detections)
        self.tag_ids = [detection["tag_id"] for detection in detections]
        return detections

    def _calculate_orientation_deg(
        self,
        corners: list[tuple[float, float]],
    ) -> float | None:
        """
        Calculate AprilTag orientation from corner[0] to corner[1].

        Args:
            corners: AprilTag corner coordinates.

        Returns:
            Orientation normalized to -90..+90 degrees.
        """
        if len(corners) < 2:
            return None

        x0, y0 = corners[0]
        x1, y1 = corners[1]
        raw_angle_deg = math.degrees(math.atan2(y1 - y0, x1 - x0))
        return ((raw_angle_deg + 90.0) % 180.0) - 90.0

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

        frame_height, frame_width = frame.shape[:2]
        if (
            self._gray_frame is None
            or self._gray_frame.shape != (frame_height, frame_width)
        ):
            self._gray_frame = np.empty((frame_height, frame_width), dtype=np.uint8)

        if frame.shape[2] == 4:
            cv2.cvtColor(frame, cv2.COLOR_RGBA2GRAY, dst=self._gray_frame)
            return self._gray_frame

        cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY, dst=self._gray_frame)
        return self._gray_frame
