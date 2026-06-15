from __future__ import annotations

import argparse
import csv
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
    from perception.perception_manager import PerceptionManager
    from validation.test_heading_lock import (
        PIDResult,
        StartGateStatus,
        TagMeasurement,
        HeadingPIDController,
        calculate_tag_orientation_deg,
        choose_initial_orientation_state,
        command_for_alignment_state,
        evaluate_start_gate,
        format_csv,
        format_display,
        format_set_drive_command,
        get_orientation_color,
        measure_tag,
        transmit_command,
        wrap_angle_deg,
    )
except ModuleNotFoundError as error:
    DEPENDENCY_ERROR = error


KP_HEADING = 100.0
KI_HEADING = 0.0
KD_HEADING = 0.0

HEADING_DEADBAND_DEG = 0.5
MAX_INTEGRAL = 50.0
MAX_CORRECTION = 1000.0

BASE_FREQUENCY_HZ = 9000.0
MIN_FREQUENCY_HZ = 8000.0
MAX_FREQUENCY_HZ = 10000.0
PID_UPDATE_INTERVAL_SEC = 0.05

DEFAULT_TRAVEL_DISTANCE_CM = 75.0
DEFAULT_WHEEL_DIAMETER_MM = 117.0
DEFAULT_PULSES_PER_REVOLUTION = 40000


@dataclass
class DistanceEstimate:
    """Stores distance travelled using commanded wheel frequencies."""

    travelled_cm: float = 0.0
    last_update_time: float | None = None

    def reset(self) -> None:
        """Clear the travelled distance estimate."""
        self.travelled_cm = 0.0
        self.last_update_time = None

    def update(
        self,
        now: float,
        pid_result: PIDResult | None,
        wheel_diameter_mm: float,
        pulses_per_revolution: int,
    ) -> None:
        """
        Integrate commanded wheel frequency into approximate travel distance.

        Args:
            now: Current monotonic timestamp.
            pid_result: Latest PID result containing left/right frequency.
            wheel_diameter_mm: Wheel diameter used by the AGV.
            pulses_per_revolution: T60 driver pulse setting.
        """
        if pid_result is None:
            self.last_update_time = now
            return

        if self.last_update_time is None:
            self.last_update_time = now
            return

        delta_time_sec = max(now - self.last_update_time, 0.0)
        self.last_update_time = now

        average_frequency_hz = (
            pid_result.left_frequency_hz + pid_result.right_frequency_hz
        ) / 2.0
        wheel_circumference_mm = 3.141592653589793 * wheel_diameter_mm
        revolutions = average_frequency_hz * delta_time_sec / pulses_per_revolution
        self.travelled_cm += revolutions * wheel_circumference_mm / 10.0


class PIDDistanceLogger:
    """Writes PID heading-control samples for the 75 cm validation run."""

    def __init__(self, csv_path: Path) -> None:
        """
        Create a fresh PID validation CSV file.

        Args:
            csv_path: Output CSV path.
        """
        self.csv_path = csv_path
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)

        with self.csv_path.open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "timestamp",
                    "distance_travelled_cm",
                    "reference_heading_deg",
                    "current_heading_deg",
                    "heading_error_deg",
                    "kp",
                    "ki",
                    "kd",
                    "p_term",
                    "i_term",
                    "d_term",
                    "pid_output",
                    "left_frequency_hz",
                    "right_frequency_hz",
                ]
            )

    def write(
        self,
        timestamp_sec: float,
        distance_travelled_cm: float,
        reference_heading_deg: float | None,
        current_heading_deg: float | None,
        heading_error_deg: float | None,
        pid_result: PIDResult | None,
    ) -> None:
        """
        Append one PID validation sample.

        Args:
            timestamp_sec: Time since program start.
            distance_travelled_cm: Estimated distance travelled.
            reference_heading_deg: Captured heading reference.
            current_heading_deg: Latest IMU heading.
            heading_error_deg: Current heading error.
            pid_result: Latest PID calculation.
        """
        with self.csv_path.open("a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    format_csv(timestamp_sec),
                    format_csv(distance_travelled_cm),
                    format_csv(reference_heading_deg),
                    format_csv(current_heading_deg),
                    format_csv(heading_error_deg),
                    format_csv(pid_result.kp if pid_result is not None else None),
                    format_csv(pid_result.ki if pid_result is not None else None),
                    format_csv(pid_result.kd if pid_result is not None else None),
                    format_csv(pid_result.p_term if pid_result is not None else None),
                    format_csv(pid_result.i_term if pid_result is not None else None),
                    format_csv(pid_result.d_term if pid_result is not None else None),
                    format_csv(pid_result.pid_output if pid_result is not None else None),
                    format_csv(pid_result.left_frequency_hz if pid_result is not None else None),
                    format_csv(pid_result.right_frequency_hz if pid_result is not None else None),
                ]
            )


def create_pid_controller() -> HeadingPIDController:
    """
    Create the heading PID controller with phase-one tuning values.

    Returns:
        Configured HeadingPIDController.
    """
    return HeadingPIDController(
        kp=KP_HEADING,
        ki=KI_HEADING,
        kd=KD_HEADING,
        max_integral=MAX_INTEGRAL,
        max_correction=MAX_CORRECTION,
        base_frequency_hz=BASE_FREQUENCY_HZ,
        min_frequency_hz=MIN_FREQUENCY_HZ,
        max_frequency_hz=MAX_FREQUENCY_HZ,
    )


def send_set_drive(
    serial_controller: SerialMotorController,
    pid_result: PIDResult,
    program_start_time: float,
    movement_start_time: float | None,
) -> str:
    """
    Send one SET_DRIVE command from the PID result.

    Args:
        serial_controller: ESP32 serial connection.
        pid_result: Current PID result.
        program_start_time: Program start time for logging.
        movement_start_time: Movement start time for logging.

    Returns:
        Command text that was transmitted.
    """
    command = format_set_drive_command(
        pid_result.left_frequency_hz,
        pid_result.right_frequency_hz,
    )
    transmit_command(
        serial_controller,
        command,
        program_start_time,
        movement_start_time,
        pid_result.heading_error_deg,
        None,
    )
    return command


def draw_overlay(
    frame,
    detection: dict | None,
    measurement: TagMeasurement,
    state: str,
    distance_travelled_cm: float,
    target_distance_cm: float,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    pid_result: PIDResult | None,
    gate_status: StartGateStatus | None,
    current_command: str,
) -> None:
    """
    Draw startup vision and PID validation information on the camera frame.

    Args:
        frame: Latest camera frame.
        detection: First visible AprilTag detection.
        measurement: Current AprilTag measurement.
        state: Current validation state.
        distance_travelled_cm: Estimated distance.
        target_distance_cm: Stop distance.
        reference_heading_deg: Captured heading reference.
        current_heading_deg: Latest IMU heading.
        heading_error_deg: Current heading error.
        pid_result: Latest PID result.
        gate_status: Current vision gate status.
        current_command: Last transmitted command.
    """
    image_height, image_width = frame.shape[:2]
    image_center_x = int(image_width / 2)
    image_center_y = int(image_height / 2)

    cv2.line(frame, (image_center_x, 0), (image_center_x, image_height), (0, 255, 255), 1)
    cv2.line(frame, (0, image_center_y), (image_width, image_center_y), (0, 255, 255), 1)
    cv2.rectangle(
        frame,
        (image_center_x - 10, image_center_y - 80),
        (image_center_x + 10, image_center_y + 80),
        (255, 255, 0),
        2,
    )

    if detection is not None:
        corners = [
            (int(corner_x), int(corner_y))
            for corner_x, corner_y in detection["corners"]
        ]
        for index, corner in enumerate(corners):
            cv2.line(frame, corner, corners[(index + 1) % len(corners)], (0, 255, 0), 2)
        if len(corners) >= 2:
            cv2.line(frame, corners[0], corners[1], (255, 0, 255), 3)
        if measurement.tag_center_x is not None and measurement.tag_center_y is not None:
            cv2.circle(
                frame,
                (int(measurement.tag_center_x), int(measurement.tag_center_y)),
                6,
                (0, 0, 255),
                -1,
            )

    pid_text = "None"
    left_frequency = "None"
    right_frequency = "None"
    if pid_result is not None:
        pid_text = f"{pid_result.pid_output:+.2f}"
        left_frequency = f"{pid_result.left_frequency_hz:.0f} Hz"
        right_frequency = f"{pid_result.right_frequency_hz:.0f} Hz"

    lines = [
        ("State", state),
        ("Distance", f"{distance_travelled_cm:.1f} / {target_distance_cm:.1f} cm"),
        ("Reference Heading", format_display(reference_heading_deg, " deg", signed=True)),
        ("Current Heading", format_display(current_heading_deg, " deg", signed=True)),
        ("Heading Error", format_display(heading_error_deg, " deg", signed=True)),
        ("P Term", format_display(pid_result.p_term if pid_result else None, signed=True)),
        ("I Term", format_display(pid_result.i_term if pid_result else None, signed=True)),
        ("D Term", format_display(pid_result.d_term if pid_result else None, signed=True)),
        ("PID Output", pid_text),
        ("Left Frequency", left_frequency),
        ("Right Frequency", right_frequency),
        ("Tag Orientation", format_display(measurement.orientation_deg, " deg", signed=True)),
        ("Position Error X", format_display(measurement.position_error_x, " px", decimals=0, signed=True)),
        ("Vision Gate", gate_status.status_text if gate_status else "None"),
        ("Command", current_command),
    ]

    y = 28
    for label, value in lines:
        color = get_orientation_color(measurement.orientation_deg) if label == "Tag Orientation" else (255, 255, 255)
        cv2.putText(
            frame,
            f"{label}: {value}",
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            color,
            2,
        )
        y += 26

    cv2.putText(
        frame,
        "Keys: S STOP | Q QUIT",
        (20, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        2,
    )


def log_terminal(
    distance_travelled_cm: float,
    reference_heading_deg: float | None,
    current_heading_deg: float | None,
    heading_error_deg: float | None,
    pid_result: PIDResult | None,
    current_command: str,
) -> None:
    """
    Print PID heading-control diagnostics to the terminal.

    Args:
        distance_travelled_cm: Estimated travel distance.
        reference_heading_deg: Captured heading reference.
        current_heading_deg: Latest IMU heading.
        heading_error_deg: Current heading error.
        pid_result: Latest PID result.
        current_command: Last transmitted command.
    """
    print("Distance Travelled:", f"{distance_travelled_cm:.2f} cm")
    print("Reference Heading:", format_display(reference_heading_deg, " deg", signed=True))
    print("Current Heading:", format_display(current_heading_deg, " deg", signed=True))
    print("Heading Error:", format_display(heading_error_deg, " deg", signed=True))
    print("P Term:", format_display(pid_result.p_term if pid_result else None, signed=True))
    print("I Term:", format_display(pid_result.i_term if pid_result else None, signed=True))
    print("D Term:", format_display(pid_result.d_term if pid_result else None, signed=True))
    print("PID Output:", format_display(pid_result.pid_output if pid_result else None, signed=True))
    print("Left Frequency:", format_display(pid_result.left_frequency_hz if pid_result else None, " Hz"))
    print("Right Frequency:", format_display(pid_result.right_frequency_hz if pid_result else None, " Hz"))
    print("Command:", current_command)
    print()


def print_run_summary(
    heading_error_samples: list[float],
    pid_output_samples: list[float],
    left_frequency_samples: list[float],
    right_frequency_samples: list[float],
) -> None:
    """
    Print a compact heading-control summary for later PID tuning.

    Args:
        heading_error_samples: Heading error samples from the run.
        pid_output_samples: PID output samples from the run.
        left_frequency_samples: Left frequency samples from the run.
        right_frequency_samples: Right frequency samples from the run.
    """
    if not heading_error_samples:
        print("PID run summary unavailable: no heading samples were recorded.")
        return

    absolute_errors = [abs(error) for error in heading_error_samples]
    all_frequencies = left_frequency_samples + right_frequency_samples

    print("PID 75 cm Run Summary")
    print("Maximum Heading Error:", f"{max(absolute_errors):.2f} deg")
    print("Average Heading Error:", f"{sum(absolute_errors) / len(absolute_errors):.2f} deg")
    print("Final Heading Error:", f"{heading_error_samples[-1]:+.2f} deg")
    print("Maximum PID Output:", f"{max(abs(output) for output in pid_output_samples):.2f}")
    if all_frequencies:
        print("Minimum Frequency Used:", f"{min(all_frequencies):.0f} Hz")
        print("Maximum Frequency Used:", f"{max(all_frequencies):.0f} Hz")
    print()


def run_validation(args: argparse.Namespace) -> None:
    """Run the 75 cm PID heading-control validation test."""
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )
    imu_reader = ImuSerialReader(
        port=args.port,
        baudrate=args.baudrate,
        timeout=0.0,
    )
    pid_controller = create_pid_controller()
    distance_estimate = DistanceEstimate()
    logger = PIDDistanceLogger(Path(args.csv).expanduser().resolve())

    state = "WAIT_FOR_TAG1"
    current_command = "NONE"
    gate_status: StartGateStatus | None = None
    gate_stable_since: float | None = None
    reference_heading_deg: float | None = None
    latest_pid_result: PIDResult | None = None
    last_pid_update_time = 0.0
    last_log_time = 0.0
    last_csv_time = 0.0
    start_time = time.monotonic()
    movement_start_time: float | None = None
    summary_printed = False
    heading_error_samples: list[float] = []
    pid_output_samples: list[float] = []
    left_frequency_samples: list[float] = []
    right_frequency_samples: list[float] = []

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    print("STATE TRANSITION >>> WAIT_FOR_TAG1")

    try:
        while True:
            detections = perception_manager.update()
            frame = perception_manager.last_frame
            detection = detections[0] if detections else None
            measurement = measure_tag(frame, detection)

            imu_reader.update_from_connection(serial_controller.serial_connection)
            current_heading_deg = imu_reader.get_heading()

            now = time.monotonic()
            elapsed_time_sec = now - start_time
            heading_error_deg = (
                wrap_angle_deg(reference_heading_deg - current_heading_deg)
                if reference_heading_deg is not None and current_heading_deg is not None
                else None
            )

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("s"), ord("S")):
                current_command = "STOP"
                transmit_command(serial_controller, current_command, start_time, movement_start_time)
                state = "STOPPED"
                print("STATE TRANSITION >>> STOPPED")

            if state == "WAIT_FOR_TAG1" and measurement.tag_id == 1:
                state = "ALIGN_TAG1"
                print("STATE TRANSITION >>> ALIGN_TAG1")

            elif state == "ALIGN_TAG1":
                alignment_state = choose_initial_orientation_state(
                    measurement.orientation_deg,
                    args.orientation_tolerance,
                )
                desired_command = command_for_alignment_state(alignment_state)
                if desired_command != "NONE" and desired_command != current_command:
                    current_command = desired_command
                    transmit_command(
                        serial_controller,
                        current_command,
                        start_time,
                        movement_start_time,
                        heading_error_deg,
                        measurement.orientation_deg,
                    )

                if alignment_state == "ALIGNED":
                    gate_stable_since = None
                    gate_status = None
                    state = "VISION_START_GATE"
                    print("STATE TRANSITION >>> VISION_START_GATE")

            elif state == "VISION_START_GATE":
                gate_status, gate_stable_since = evaluate_start_gate(
                    measurement,
                    None,
                    gate_stable_since,
                    now,
                    args.start_gate_duration,
                )

                if gate_status.status_text == "READY_TO_MOVE":
                    print("VISION START GATE PASSED")
                    current_command = "CALIBRATE_IMU"
                    transmit_command(serial_controller, current_command, start_time, movement_start_time)
                    imu_reader.latest_heading_deg = None
                    imu_reader.latest_status = "IMU_CALIBRATING"
                    reference_heading_deg = None
                    state = "CALIBRATE_IMU"
                    print("STATE TRANSITION >>> CALIBRATE_IMU")

            elif state == "CALIBRATE_IMU":
                if current_heading_deg is not None:
                    print("IMU_READY RECEIVED")
                    reference_heading_deg = current_heading_deg
                    heading_error_deg = 0.0
                    pid_controller.reset()
                    distance_estimate.reset()
                    latest_pid_result = None
                    last_pid_update_time = 0.0
                    movement_start_time = None
                    print(
                        "REFERENCE HEADING CAPTURED:",
                        format_display(reference_heading_deg, " deg", signed=True),
                    )
                    state = "HEADING_HOLD_75CM"
                    print("STATE TRANSITION >>> HEADING_HOLD_75CM")

            elif state == "HEADING_HOLD_75CM":
                distance_estimate.update(
                    now,
                    latest_pid_result,
                    args.wheel_diameter_mm,
                    args.pulses_per_revolution,
                )

                if distance_estimate.travelled_cm >= args.travel_distance_cm:
                    current_command = "STOP"
                    transmit_command(serial_controller, current_command, start_time, movement_start_time)
                    state = "DISTANCE_COMPLETE"
                    print("TRAVEL DISTANCE COMPLETE")
                    print("STATE TRANSITION >>> DISTANCE_COMPLETE")
                elif (
                    reference_heading_deg is not None
                    and current_heading_deg is not None
                    and (now - last_pid_update_time) >= PID_UPDATE_INTERVAL_SEC
                ):
                    latest_pid_result = pid_controller.update(
                        reference_heading_deg,
                        current_heading_deg,
                        now,
                    )
                    heading_error_deg = latest_pid_result.heading_error_deg
                    current_command = send_set_drive(
                        serial_controller,
                        latest_pid_result,
                        start_time,
                        movement_start_time,
                    )
                    heading_error_samples.append(latest_pid_result.heading_error_deg)
                    pid_output_samples.append(latest_pid_result.pid_output)
                    left_frequency_samples.append(latest_pid_result.left_frequency_hz)
                    right_frequency_samples.append(latest_pid_result.right_frequency_hz)
                    if movement_start_time is None:
                        movement_start_time = now
                        distance_estimate.last_update_time = now
                    last_pid_update_time = now

            elif state == "DISTANCE_COMPLETE" and not summary_printed:
                print_run_summary(
                    heading_error_samples,
                    pid_output_samples,
                    left_frequency_samples,
                    right_frequency_samples,
                )
                summary_printed = True

            if (now - last_csv_time) >= args.csv_interval:
                logger.write(
                    elapsed_time_sec,
                    distance_estimate.travelled_cm,
                    reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    latest_pid_result,
                )
                last_csv_time = now

            if (now - last_log_time) >= args.log_interval:
                log_terminal(
                    distance_estimate.travelled_cm,
                    reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    latest_pid_result,
                    current_command,
                )
                last_log_time = now

            if frame is not None:
                draw_overlay(
                    frame,
                    detection,
                    measurement,
                    state,
                    distance_estimate.travelled_cm,
                    args.travel_distance_cm,
                    reference_heading_deg,
                    current_heading_deg,
                    heading_error_deg,
                    latest_pid_result,
                    gate_status,
                    current_command,
                )
                cv2.imshow("PID Heading Lock 75 cm Validation", frame)

            if key in (ord("q"), ord("Q")):
                break
    except KeyboardInterrupt:
        print("Stopping PID heading validation.")
    finally:
        transmit_command(serial_controller, "STOP", start_time, movement_start_time)
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse 75 cm PID validation options."""
    parser = argparse.ArgumentParser(
        description="Validation-only PID heading hold over a fixed 75 cm distance.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--orientation-tolerance", type=float, default=2.0)
    parser.add_argument("--start-gate-duration", type=float, default=0.5)
    parser.add_argument("--travel-distance-cm", type=float, default=DEFAULT_TRAVEL_DISTANCE_CM)
    parser.add_argument("--wheel-diameter-mm", type=float, default=DEFAULT_WHEEL_DIAMETER_MM)
    parser.add_argument("--pulses-per-revolution", type=int, default=DEFAULT_PULSES_PER_REVOLUTION)
    parser.add_argument("--csv", default="validation/heading_lock_pid_75cm_log.csv")
    parser.add_argument("--csv-interval", type=float, default=0.1)
    parser.add_argument("--log-interval", type=float, default=0.25)
    return parser.parse_args()


if __name__ == "__main__":
    if any(argument in ("-h", "--help") for argument in sys.argv[1:]):
        parse_args()
        raise SystemExit(0)

    if DEPENDENCY_ERROR is not None:
        print(
            "PID 75 cm validation cannot start because a required dependency "
            f"is missing: {DEPENDENCY_ERROR.name}"
        )
        print(
            "Run this validation on the Raspberry Pi environment containing "
            "OpenCV, pyserial, Picamera2, and pupil_apriltags."
        )
        raise SystemExit(1)

    run_validation(parse_args())
