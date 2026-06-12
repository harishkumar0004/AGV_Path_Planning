from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

DEPENDENCY_ERROR: ModuleNotFoundError | None = None

try:
    import cv2

    from core.application_state import ApplicationState
    from perception.perception_manager import PerceptionManager
except ModuleNotFoundError as error:
    DEPENDENCY_ERROR = error


def calculate_tag_orientation_deg(detection: dict[str, Any] | None) -> float | None:
    """
    Calculate AprilTag orientation from the top edge of its corner coordinates.

    Args:
        detection: AprilTag detection dictionary, or None.

    Returns:
        Orientation angle normalized to -90..+90 degrees, or None.
    """
    if detection is None:
        return None

    corners = detection.get("corners", [])
    if len(corners) < 2:
        return None

    x0, y0 = corners[0]
    x1, y1 = corners[1]
    dx = x1 - x0
    dy = y1 - y0

    raw_angle_deg = math.degrees(math.atan2(dy, dx))
    return ((raw_angle_deg + 90.0) % 180.0) - 90.0


def draw_tag_overlay(
    frame,
    detection: dict[str, Any] | None,
    orientation_deg: float | None,
    position_error_x: float | None,
) -> None:
    """
    Draw AprilTag orientation diagnostics on a camera frame.

    Args:
        frame: Camera image to annotate.
        detection: First visible AprilTag detection.
        orientation_deg: Tag orientation angle.
        position_error_x: Tag center X minus image center X.
    """
    image_height, image_width = frame.shape[:2]
    image_center_x = image_width // 2
    image_center_y = image_height // 2

    cv2.line(
        frame,
        (image_center_x, 0),
        (image_center_x, image_height),
        (0, 255, 255),
        1,
    )
    cv2.line(
        frame,
        (0, image_center_y),
        (image_width, image_center_y),
        (0, 255, 255),
        1,
    )
    cv2.drawMarker(
        frame,
        (image_center_x, image_center_y),
        (0, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=20,
        thickness=2,
    )

    tag_id_text = "None"
    tag_center_x_text = "None"

    if detection is not None:
        tag_id_text = str(detection["tag_id"])
        tag_center_x = int(detection["center_x"])
        tag_center_y = int(detection["center_y"])
        tag_center_x_text = str(tag_center_x)

        corners = [
            (int(corner_x), int(corner_y))
            for corner_x, corner_y in detection["corners"]
        ]

        for index, corner in enumerate(corners):
            cv2.line(
                frame,
                corner,
                corners[(index + 1) % len(corners)],
                (0, 255, 0),
                2,
            )

        if len(corners) >= 2:
            cv2.line(frame, corners[0], corners[1], (255, 0, 255), 3)

        cv2.circle(frame, (tag_center_x, tag_center_y), 5, (0, 0, 255), -1)
        cv2.line(
            frame,
            (image_center_x, image_center_y),
            (tag_center_x, tag_center_y),
            (255, 0, 255),
            2,
        )

    orientation_text = (
        f"{orientation_deg:+.1f} deg"
        if orientation_deg is not None
        else "None"
    )
    error_text = (
        f"{position_error_x:+.1f} px"
        if position_error_x is not None
        else "None"
    )

    lines = [
        f"Tag ID: {tag_id_text}",
        f"Orientation: {orientation_text}",
        f"Tag Center X: {tag_center_x_text}",
        f"Image Center X: {image_center_x}",
        f"Position Error: {error_text}",
        "Q: QUIT",
    ]

    y = 30
    for line in lines:
        cv2.putText(
            frame,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        y += 34


def log_orientation(
    detection: dict[str, Any] | None,
    orientation_deg: float | None,
    position_error_x: float | None,
) -> None:
    """
    Print one orientation measurement.

    Args:
        detection: First visible AprilTag detection.
        orientation_deg: Tag orientation angle.
        position_error_x: Tag center X minus image center X.
    """
    if detection is None:
        print("Tag ID: None | Orientation: None | Position Error: None")
        return

    orientation_text = (
        f"{orientation_deg:+.1f}"
        if orientation_deg is not None
        else "None"
    )
    error_text = (
        f"{position_error_x:+.1f}"
        if position_error_x is not None
        else "None"
    )

    print(
        f"Tag ID: {detection['tag_id']} | "
        f"Orientation: {orientation_text} deg | "
        f"Position Error: {error_text} px"
    )


def run_validation(args: argparse.Namespace) -> None:
    """
    Run AprilTag orientation measurement validation.

    This tool does not open serial communication and does not send motor
    commands. It only displays camera-side tag orientation diagnostics.
    """
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    last_log_time = 0.0

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    print("AprilTag orientation validation started.")
    print("Place AGV at 0, +5, +10, and -10 degrees to compare readings.")

    try:
        while True:
            detections = perception_manager.update()
            frame = perception_manager.last_frame
            detection = detections[0] if detections else None

            orientation_deg = calculate_tag_orientation_deg(detection)
            position_error_x: float | None = None

            if frame is not None and detection is not None:
                image_center_x = frame.shape[1] / 2
                position_error_x = detection["center_x"] - image_center_x

            now = time.monotonic()
            if (now - last_log_time) >= args.log_interval:
                log_orientation(detection, orientation_deg, position_error_x)
                last_log_time = now

            if frame is not None:
                draw_tag_overlay(
                    frame,
                    detection,
                    orientation_deg,
                    position_error_x,
                )
                cv2.imshow("AprilTag Orientation Validation", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
    except KeyboardInterrupt:
        print("Stopping AprilTag orientation validation.")
    finally:
        perception_manager.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse AprilTag orientation validation options."""
    parser = argparse.ArgumentParser(
        description="Display validation-only AprilTag orientation measurements.",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=0.25,
        help="Seconds between terminal orientation logs.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    if any(argument in ("-h", "--help") for argument in sys.argv[1:]):
        parse_args()
        raise SystemExit(0)

    if DEPENDENCY_ERROR is not None:
        print(
            "Tag orientation validation cannot start because a required "
            f"dependency is missing: {DEPENDENCY_ERROR.name}"
        )
        print(
            "Run this validation in the Raspberry Pi environment containing "
            "OpenCV, Picamera2, and pupil_apriltags."
        )
        raise SystemExit(1)

    run_validation(parse_args())
