from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

DEPENDENCY_ERROR: ModuleNotFoundError | None = None

try:
    import cv2

    from communication.imu_serial_reader import ImuSerialReader
    from communication.serial_motor_controller import SerialMotorController
    from core.application_state import ApplicationState
    from navigation.alignment_controller import AlignmentController
    from navigation.motion_commands import MotionCommand
    from navigation.navigation_state import NavigationState
    from navigation.navigation_state_manager import NavigationStateManager
    from perception.perception_manager import PerceptionManager
    from validation.performance_monitor import PerformanceMonitor
except ModuleNotFoundError as error:
    DEPENDENCY_ERROR = error


@dataclass
class PulseCorrectionDecision:
    """Stores one validation-only steering pulse correction decision."""

    command: str
    duration_ms: int | None
    error_x: float | None
    image_center_x: float | None
    tag_center_x: float | None


class PulseCorrectionController:
    """Generates cooldown-protected steering pulse commands from tag error."""

    def __init__(
        self,
        center_deadband_px: float = 15.0,
        medium_error_px: float = 50.0,
        medium_pulse_ms: int = 100,
        large_pulse_ms: int = 150,
        cooldown_margin_ms: int = 50,
    ) -> None:
        """
        Create a pulse correction controller.

        Args:
            center_deadband_px: Error range where no correction is needed.
            medium_error_px: Error limit for the medium pulse duration.
            medium_pulse_ms: Pulse duration for medium position error.
            large_pulse_ms: Pulse duration for large position error.
            cooldown_margin_ms: Extra wait time after each pulse.
        """
        self.center_deadband_px = center_deadband_px
        self.medium_error_px = medium_error_px
        self.medium_pulse_ms = medium_pulse_ms
        self.large_pulse_ms = large_pulse_ms
        self.cooldown_margin_ms = cooldown_margin_ms
        self.cooldown_until = 0.0
        self.resume_command: str | None = None
        self.current_command = "FORWARD"

    def calculate(
        self,
        image_center_x: float | None,
        tag_center_x: float | None,
    ) -> PulseCorrectionDecision:
        """
        Convert tag center error into a steering pulse decision.

        Args:
            image_center_x: Horizontal image center.
            tag_center_x: Detected tag center X.

        Returns:
            Correction command and duration.
        """
        if image_center_x is None or tag_center_x is None:
            return PulseCorrectionDecision(
                "NO_TAG",
                None,
                None,
                image_center_x,
                tag_center_x,
            )

        error_x = image_center_x - tag_center_x
        abs_error = abs(error_x)

        if abs_error <= self.center_deadband_px:
            return PulseCorrectionDecision(
                "FORWARD",
                None,
                error_x,
                image_center_x,
                tag_center_x,
            )

        duration_ms = (
            self.medium_pulse_ms
            if abs_error <= self.medium_error_px
            else self.large_pulse_ms
        )
        direction = "LEFT_PULSE" if error_x > 0 else "RIGHT_PULSE"

        return PulseCorrectionDecision(
            direction,
            duration_ms,
            error_x,
            image_center_x,
            tag_center_x,
        )

    def can_send(self) -> bool:
        """Return True when the correction cooldown period has finished."""
        return time.monotonic() >= self.cooldown_until

    def mark_sent(
        self,
        decision: PulseCorrectionDecision,
        resume_command: str,
    ) -> None:
        """
        Store cooldown timing after a correction pulse is transmitted.

        Args:
            decision: Pulse decision that was sent.
            resume_command: Forward command to restore after the pulse.
        """
        if decision.duration_ms is None:
            return

        cooldown_sec = (
            decision.duration_ms + self.cooldown_margin_ms
        ) / 1000.0
        self.cooldown_until = time.monotonic() + cooldown_sec
        self.resume_command = resume_command
        self.current_command = f"{decision.command} {decision.duration_ms}"

    def resume_if_ready(self, serial_controller: SerialMotorController) -> bool:
        """
        Restore forward travel once a validation pulse is expected to be done.

        Args:
            serial_controller: Serial connection to ESP32.

        Returns:
            True when a forward command was resent.
        """
        if self.resume_command is None or not self.can_send():
            return False

        serial_controller.send_raw_command(self.resume_command, force=True)
        self.resume_command = None
        self.current_command = "FORWARD"
        return True


def draw_alignment_guides(
    frame,
    detections: list[dict],
    alignment_tolerance_px: float,
) -> None:
    """
    Draw camera center references and AprilTag alignment diagnostics.

    Args:
        frame: Camera image to annotate.
        detections: AprilTag detections from PerceptionManager.
        alignment_tolerance_px: Pixel threshold used for ALIGNED status.
    """
    image_height, image_width = frame.shape[:2]
    center_x = image_width // 2
    center_y = image_height // 2

    # Draw camera center reference lines.
    cv2.line(frame, (center_x, 0), (center_x, image_height), (0, 255, 255), 1)
    cv2.line(frame, (0, center_y), (image_width, center_y), (0, 255, 255), 1)

    # Draw a small crosshair at the image center.
    cv2.drawMarker(
        frame,
        (center_x, center_y),
        (0, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=20,
        thickness=2,
    )

    cv2.putText(
        frame,
        f"Image Center: ({center_x}, {center_y})",
        (image_width - 300, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )

    if not detections:
        cv2.putText(
            frame,
            "Alignment Status: NO TAG",
            (image_width - 300, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )
        return

    detection = detections[0]
    tag_center_x = int(detection["center_x"])
    tag_center_y = int(detection["center_y"])
    error_x = tag_center_x - center_x
    error_y = tag_center_y - center_y
    status = "ALIGNED" if abs(error_x) <= alignment_tolerance_px else "ALIGNING"
    status_color = (0, 255, 0) if status == "ALIGNED" else (0, 165, 255)

    # Draw AprilTag boundary.
    corners = [
        (int(corner_x), int(corner_y))
        for corner_x, corner_y in detection["corners"]
    ]
    for index, corner in enumerate(corners):
        next_corner = corners[(index + 1) % len(corners)]
        cv2.line(frame, corner, next_corner, (0, 255, 0), 2)

    # Draw tag center and visual alignment error line.
    cv2.circle(frame, (tag_center_x, tag_center_y), 5, (0, 0, 255), -1)
    cv2.line(
        frame,
        (center_x, center_y),
        (tag_center_x, tag_center_y),
        (255, 0, 255),
        2,
    )

    cv2.putText(
        frame,
        f"Tag Center: ({tag_center_x}, {tag_center_y})",
        (image_width - 300, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"Center Error X: {error_x} px",
        (image_width - 300, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"Center Error Y: {error_y} px",
        (image_width - 300, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        frame,
        f"Alignment Status: {status}",
        (image_width - 300, 150),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        status_color,
        2,
    )


def send_motion_command(
    serial_controller: SerialMotorController,
    command: MotionCommand,
    turn_right_value: float | None,
    use_calibrated_turn: bool = True,
) -> bool:
    """
    Send a navigation command, applying calibrated turn value when available.

    Args:
        serial_controller: Serial link to ESP32.
        command: Motion command generated by navigation.
        turn_right_value: Optional calibrated TURN_RIGHT value.
        use_calibrated_turn: True only for the route turn, not alignment nudges.

    Returns:
        True if the command was transmitted or already sent.
    """
    if (
        use_calibrated_turn
        and command == MotionCommand.TURN_RIGHT
        and turn_right_value is not None
    ):
        return serial_controller.send_command(
            command,
            value=turn_right_value,
            force=True,
        )

    return serial_controller.send_command(command)


def draw_metrics_overlay(
    frame,
    monitor: PerformanceMonitor,
    live_fps: float,
    frame_time_ms: float,
    pulse_decision: PulseCorrectionDecision | None,
    current_correction_command: str,
) -> None:
    """
    Draw navigation validation metrics on the camera frame.

    Args:
        frame: Camera image to annotate.
        monitor: Performance monitor containing current metrics.
    """
    metrics = monitor.metrics
    latency_text = (
        f"{metrics.command_latency_ms:.1f} ms"
        if metrics.command_latency_ms is not None
        else "None"
    )
    alignment_text = (
        f"{metrics.alignment_error_px:.1f} px"
        if metrics.alignment_error_px is not None
        else "None"
    )
    heading_text = (
        f"{metrics.current_heading_deg:.2f} deg"
        if metrics.current_heading_deg is not None
        else "None"
    )
    drift_text = (
        f"{metrics.latest_drift_deg:+.2f} deg"
        if metrics.latest_drift_deg is not None
        else "None"
    )

    image_center_text = "None"
    tag_center_text = "None"
    error_text = "None"
    if pulse_decision is not None:
        image_center_text = (
            f"{pulse_decision.image_center_x:.0f}"
            if pulse_decision.image_center_x is not None
            else "None"
        )
        tag_center_text = (
            f"{pulse_decision.tag_center_x:.0f}"
            if pulse_decision.tag_center_x is not None
            else "None"
        )
        error_text = (
            f"{pulse_decision.error_x:.1f} px"
            if pulse_decision.error_x is not None
            else "None"
        )

    lines = [
        f"Tag ID: {metrics.current_tag_id}",
        f"FPS: {live_fps:.1f}",
        f"Frame Time: {frame_time_ms:.1f} ms",
        f"Camera FPS: {metrics.camera_fps:.1f}",
        f"Detection FPS: {metrics.detection_fps:.1f}",
        f"Navigation State: {metrics.navigation_state}",
        f"Current Action: {metrics.current_action}",
        f"Alignment Error: {alignment_text}",
        f"Tag Center X: {tag_center_text}",
        f"Image Center X: {image_center_text}",
        f"Error X: {error_text}",
        f"Heading: {heading_text}",
        f"Latest Drift: {drift_text}",
        f"Current Correction Command: {current_correction_command}",
        f"Command Latency: {latency_text}",
        monitor.get_tag_count_text(),
    ]

    y = 30
    for line in lines:
        cv2.putText(
            frame,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        y += 28


def main() -> None:
    """Run live navigation validation with camera overlay and serial commands."""
    args = parse_args()
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    navigation_manager = NavigationStateManager()
    alignment_controller = AlignmentController(
        center_tolerance_px=args.alignment_tolerance,
    )
    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )
    imu_reader = ImuSerialReader(
        port=args.port,
        baudrate=args.baudrate,
        timeout=0.0,
    )
    monitor = PerformanceMonitor()
    pulse_controller = PulseCorrectionController(
        center_deadband_px=args.pulse_deadband,
        medium_error_px=args.pulse_medium_error,
        medium_pulse_ms=args.medium_pulse_ms,
        large_pulse_ms=args.large_pulse_ms,
        cooldown_margin_ms=args.pulse_cooldown_margin_ms,
    )
    if not args.enable_pulse_correction:
        pulse_controller.current_command = "DISABLED"
    last_status_query_time = 0.0
    last_alignment_command: MotionCommand | None = None
    last_alignment_command_time = 0.0
    previous_frame_time = time.monotonic()
    live_fps = 0.0
    frame_time_ms = 0.0

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    drift_csv_path = Path(args.drift_csv).expanduser().resolve()
    monitor.initialize_drift_csv(drift_csv_path)

    try:
        while True:
            current_frame_time = time.monotonic()
            frame_delta = current_frame_time - previous_frame_time
            previous_frame_time = current_frame_time
            if frame_delta > 0:
                live_fps = 1.0 / frame_delta
                frame_time_ms = frame_delta * 1000.0

            monitor.record_camera_frame()
            detections = perception_manager.update()
            monitor.record_detection_cycle()
            imu_reader.update_from_connection(serial_controller.serial_connection)
            current_heading = imu_reader.get_heading()
            monitor.record_heading(current_heading)

            frame = perception_manager.last_frame
            perception = application_state.perception
            tag_id = perception.tag_id if perception.tag_visible else None
            monitor.record_tag_detection(tag_id)
            monitor.record_tag_heading(
                tag_id=tag_id,
                heading_deg=current_heading,
                csv_path=drift_csv_path,
            )

            key = cv2.waitKey(1) & 0xFF
            start_pressed = key == ord("f") or key == ord("F")

            motion_complete = False
            if navigation_manager.state in {
                NavigationState.STOPPING,
                NavigationState.ALIGNING,
                NavigationState.TURNING,
            }:
                now = time.monotonic()
                if (now - last_status_query_time) >= args.status_query_interval:
                    motion_running = serial_controller.is_motion_running()
                    motion_complete = motion_running is False
                    last_status_query_time = now

            alignment_error: float | None = None
            alignment_complete = False
            alignment_command: MotionCommand | None = None
            turn_tag_visible = tag_id == 3

            if navigation_manager.state == NavigationState.ALIGNING and motion_complete:
                if turn_tag_visible and frame is not None and perception.center_x is not None:
                    alignment_result = alignment_controller.calculate(
                        image_width=frame.shape[1],
                        tag_center_x=perception.center_x,
                    )
                    alignment_error = alignment_result.error_x
                    alignment_complete = (
                        abs(alignment_error) <= args.alignment_tolerance
                    )
                    print(f"Alignment Error: {alignment_error:.1f}")

                    if not alignment_complete:
                        alignment_command = alignment_result.action
                else:
                    alignment_command = MotionCommand.STOP

            monitor.record_alignment_error(alignment_error)

            command = navigation_manager.update(
                tag_id=tag_id,
                start_pressed=start_pressed,
                motion_complete=motion_complete,
                alignment_complete=alignment_complete,
                tag_visible=turn_tag_visible,
            )

            action_name = (
                navigation_manager.current_action.name
                if navigation_manager.current_action is not None
                else "None"
            )
            monitor.record_navigation_state(
                navigation_manager.state.name,
                action_name,
            )

            pulse_decision = PulseCorrectionDecision(
                "NO_TAG",
                None,
                None,
                frame.shape[1] / 2 if frame is not None else None,
                None,
            )
            if frame is not None and perception.tag_visible:
                pulse_decision = pulse_controller.calculate(
                    image_center_x=frame.shape[1] / 2,
                    tag_center_x=perception.center_x,
                )

            if command is not None:
                last_alignment_command = None
                pulse_controller.resume_command = None
                monitor.record_command_generated(command)
                send_motion_command(
                    serial_controller,
                    command,
                    args.turn_right_value,
                    use_calibrated_turn=True,
                )
                monitor.record_command_transmitted()
            elif alignment_command is not None:
                monitor.metrics.current_action = alignment_command.name
                now = time.monotonic()
                alignment_due = (
                    alignment_command != last_alignment_command
                    or (now - last_alignment_command_time)
                    >= args.alignment_command_interval
                )

                if alignment_due:
                    send_motion_command(
                        serial_controller,
                        alignment_command,
                        args.turn_right_value,
                        use_calibrated_turn=False,
                    )
                    last_alignment_command = alignment_command
                    last_alignment_command_time = now
            elif args.enable_pulse_correction:
                correction_allowed_state = navigation_manager.state in {
                    NavigationState.MOVING,
                    NavigationState.PREPARE_TURN,
                }

                if correction_allowed_state:
                    resumed_forward = pulse_controller.resume_if_ready(
                        serial_controller,
                    )

                    if (
                        not resumed_forward
                        and pulse_controller.resume_command is None
                        and pulse_controller.can_send()
                    ):
                        pulse_controller.current_command = (
                            pulse_decision.command
                            if pulse_decision.duration_ms is None
                            else (
                                f"{pulse_decision.command} "
                                f"{pulse_decision.duration_ms}"
                            )
                        )

                        if pulse_decision.duration_ms is not None:
                            pulse_command = (
                                f"{pulse_decision.command} "
                                f"{pulse_decision.duration_ms}"
                            )
                            resume_command = (
                                "START_SLOW_FORWARD"
                                if navigation_manager.state
                                == NavigationState.PREPARE_TURN
                                else "START_FORWARD"
                            )
                            serial_controller.send_raw_command(
                                pulse_command,
                                force=True,
                            )
                            pulse_controller.mark_sent(
                                pulse_decision,
                                resume_command,
                            )
                            timestamp = time.monotonic()
                            print(
                                f"Time: {timestamp:.2f} | "
                                f"FPS: {live_fps:.1f} | "
                                f"Error: {pulse_decision.error_x:.1f} | "
                                f"Command: {pulse_command}"
                            )
                else:
                    pulse_controller.current_command = "DISABLED"

            if frame is not None:
                draw_alignment_guides(
                    frame,
                    detections,
                    args.alignment_tolerance,
                )
                draw_metrics_overlay(
                    frame,
                    monitor,
                    live_fps,
                    frame_time_ms,
                    pulse_decision,
                    pulse_controller.current_command,
                )
                cv2.imshow("AGV Navigation Performance", frame)

            if key == ord("q"):
                break
    except KeyboardInterrupt:
        print("Stopping navigation validation.")
    finally:
        serial_controller.send_command(MotionCommand.STOP)
        serial_controller.disconnect()
        perception_manager.release()
        monitor.print_drift_summary()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for route completion validation."""
    parser = argparse.ArgumentParser(
        description="Run navigation metrics and route completion validation.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--turn-right-value",
        type=float,
        default=50.0,
        help="Calibrated value sent as TURN_RIGHT <value> at Tag 3.",
    )
    parser.add_argument(
        "--alignment-tolerance",
        type=float,
        default=10.0,
        help="Maximum Tag 3 horizontal error before stopping for the turn.",
    )
    parser.add_argument(
        "--alignment-command-interval",
        type=float,
        default=0.5,
        help="Seconds between repeated alignment correction commands.",
    )
    parser.add_argument(
        "--status-query-interval",
        type=float,
        default=0.1,
        help="Seconds between ESP32 MotionController STATUS queries.",
    )
    parser.add_argument(
        "--enable-pulse-correction",
        action="store_true",
        help="Enable validation-only tag-centering steering pulses.",
    )
    parser.add_argument(
        "--pulse-deadband",
        type=float,
        default=15.0,
        help="Pixel error treated as centered for pulse correction.",
    )
    parser.add_argument(
        "--pulse-medium-error",
        type=float,
        default=50.0,
        help="Maximum pixel error that uses the medium pulse duration.",
    )
    parser.add_argument(
        "--medium-pulse-ms",
        type=int,
        default=200,
        help="Pulse duration for 15-50 px correction error.",
    )
    parser.add_argument(
        "--large-pulse-ms",
        type=int,
        default=300,
        help="Pulse duration for correction error greater than 50 px.",
    )
    parser.add_argument(
        "--pulse-cooldown-margin-ms",
        type=int,
        default=50,
        help="Extra cooldown after each correction pulse.",
    )
    parser.add_argument(
        "--drift-csv",
        default="validation/drift_log.csv",
        help="CSV file used to store tag heading drift measurements.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    if any(argument in ("-h", "--help") for argument in sys.argv[1:]):
        parse_args()
        raise SystemExit(0)

    if DEPENDENCY_ERROR is not None:
        print(
            "Navigation metrics validation cannot start because a required "
            f"dependency is missing: {DEPENDENCY_ERROR.name}"
        )
        print(
            "Run this validation in the Raspberry Pi environment containing "
            "OpenCV, pyserial, Picamera2, and pupil_apriltags."
        )
        raise SystemExit(1)

    main()
