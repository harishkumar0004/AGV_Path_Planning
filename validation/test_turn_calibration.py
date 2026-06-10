import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

from communication.serial_motor_controller import SerialMotorController
from navigation.motion_commands import MotionCommand


@dataclass
class TurnCalibrationSample:
    """Stores one physical turn calibration measurement."""

    command_value: float
    actual_angle_deg: float


def estimate_degrees_per_command(
    samples: list[TurnCalibrationSample],
) -> float | None:
    """
    Estimate the relationship between command value and physical angle.

    The estimate assumes the relationship is approximately:

    physical_angle = command_value * scale

    Args:
        samples: Measured command/angle pairs.

    Returns:
        Estimated degrees per command unit, or None if no valid data exists.
    """
    numerator = 0.0
    denominator = 0.0

    for sample in samples:
        numerator += sample.command_value * sample.actual_angle_deg
        denominator += sample.command_value * sample.command_value

    if denominator <= 0.0:
        return None

    return numerator / denominator


def save_samples(
    output_path: Path,
    samples: list[TurnCalibrationSample],
    degrees_per_command: float | None,
) -> None:
    """
    Save calibration measurements to a CSV file.

    Args:
        output_path: CSV file path.
        samples: Measured command/angle pairs.
        degrees_per_command: Estimated scale value.
    """
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["command_value", "actual_angle_deg", "estimated_angle_deg"])

        for sample in samples:
            estimated_angle = (
                sample.command_value * degrees_per_command
                if degrees_per_command is not None
                else ""
            )
            writer.writerow(
                [
                    sample.command_value,
                    sample.actual_angle_deg,
                    estimated_angle,
                ]
            )


def run_calibration(args: argparse.Namespace) -> None:
    """
    Send turn commands and record measured physical angles.

    Args:
        args: Parsed command-line arguments.
    """
    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )

    if not serial_controller.connect():
        return

    samples: list[TurnCalibrationSample] = []

    try:
        print("Turn calibration started.")
        print("Measure the physical rotation angle after each command.")
        print("Enter the measured angle in degrees, or press Enter to skip.")

        for command_value in args.values:
            input(
                f"\nPlace the AGV at the starting heading, then press Enter "
                f"to send TURN_RIGHT {command_value:g}."
            )

            serial_controller.send_command(
                MotionCommand.TURN_RIGHT,
                value=command_value,
                force=True,
            )

            measured_text = input(
                f"Actual angle for TURN_RIGHT {command_value:g}: "
            ).strip()

            if not measured_text:
                print("Skipped measurement.")
                continue

            try:
                actual_angle = float(measured_text)
            except ValueError:
                print("Invalid angle. Skipped measurement.")
                continue

            samples.append(
                TurnCalibrationSample(
                    command_value=command_value,
                    actual_angle_deg=actual_angle,
                )
            )

        degrees_per_command = estimate_degrees_per_command(samples)

        print("\nCalibration Results")
        print("Command Value | Actual Angle | Estimated Angle")

        for sample in samples:
            estimated_angle = (
                sample.command_value * degrees_per_command
                if degrees_per_command is not None
                else 0.0
            )
            print(
                f"{sample.command_value:13.2f} | "
                f"{sample.actual_angle_deg:12.2f} | "
                f"{estimated_angle:15.2f}"
            )

        if degrees_per_command is None:
            print("Not enough valid measurements to estimate 90 degree command.")
            return

        command_for_90 = 90.0 / degrees_per_command
        print(f"\nEstimated scale: {degrees_per_command:.3f} deg / command")
        print(f"Estimated command for 90 deg: {command_for_90:.2f}")
        print(
            "Verify with: "
            f"python3 validation/test_navigation_metrics.py "
            f"--turn-right-value {command_for_90:.2f}"
        )

        if args.output:
            save_samples(Path(args.output), samples, degrees_per_command)
            print(f"Saved calibration log: {args.output}")
    finally:
        serial_controller.send_command(MotionCommand.STOP, force=True)
        serial_controller.disconnect()


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for turn calibration."""
    parser = argparse.ArgumentParser(
        description="Calibrate TURN_RIGHT command value against physical angle.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--values",
        type=float,
        nargs="+",
        default=[10.0, 20.0, 30.0, 40.0, 50.0],
        help="TURN_RIGHT command values to test.",
    )
    parser.add_argument(
        "--output",
        default="validation/turn_calibration_log.csv",
        help="CSV file for calibration measurements.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_calibration(parse_args())
