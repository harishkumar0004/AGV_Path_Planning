from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

try:
    from communication.serial_motor_controller import SerialMotorController
except ModuleNotFoundError as error:
    print(
        "Steering calibration cannot start because a required dependency is "
        f"missing: {error.name}"
    )
    print("Run this validation on the Raspberry Pi environment with pyserial.")
    raise SystemExit(1)


CSV_PATH = Path(__file__).with_name("steering_calibration.csv")

COMMAND_MAP = {
    "a": ("LEFT", 50),
    "s": ("LEFT", 100),
    "d": ("LEFT", 150),
    "f": ("LEFT", 200),
    "g": ("LEFT", 300),
    "j": ("RIGHT", 50),
    "k": ("RIGHT", 100),
    "l": ("RIGHT", 150),
    ";": ("RIGHT", 200),
    "h": ("RIGHT", 300),
}


def ensure_csv_file(csv_path: Path) -> None:
    """
    Create the steering calibration CSV file with a header if needed.

    Args:
        csv_path: Destination CSV path for measured calibration results.
    """
    if csv_path.exists() and csv_path.stat().st_size > 0:
        return

    with csv_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["pulse_ms", "direction", "actual_rotation_deg"])


def append_measurement(
    csv_path: Path,
    pulse_ms: int,
    direction: str,
    actual_rotation_deg: float,
) -> None:
    """
    Append one measured steering response row to the CSV file.

    Args:
        csv_path: Destination CSV path.
        pulse_ms: Pulse duration sent to the ESP32.
        direction: LEFT or RIGHT.
        actual_rotation_deg: Operator-measured heading change in degrees.
    """
    with csv_path.open("a", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([pulse_ms, direction, actual_rotation_deg])


def print_menu(csv_path: Path) -> None:
    """
    Print keyboard controls and calibration instructions.

    Args:
        csv_path: CSV path where results are stored.
    """
    print()
    print("Steering Pulse Calibration")
    print("--------------------------")
    print("Place the AGV on a floor reference line before each pulse.")
    print("After each pulse, measure the actual heading change and enter it.")
    print()
    print("Left pulses:")
    print("  A -> LEFT_PULSE 50")
    print("  S -> LEFT_PULSE 100")
    print("  D -> LEFT_PULSE 150")
    print("  F -> LEFT_PULSE 200")
    print("  G -> LEFT_PULSE 300")
    print()
    print("Right pulses:")
    print("  J -> RIGHT_PULSE 50")
    print("  K -> RIGHT_PULSE 100")
    print("  L -> RIGHT_PULSE 150")
    print("  ; -> RIGHT_PULSE 200")
    print("  H -> RIGHT_PULSE 300")
    print()
    print("Q -> Quit")
    print(f"CSV log: {csv_path}")
    print()


def read_actual_rotation() -> float | None:
    """
    Ask the operator for the measured heading change.

    Returns:
        Measured rotation in degrees, or None when the user skips logging.
    """
    while True:
        text = input(
            "Measured actual rotation in degrees "
            "(blank to skip CSV row): "
        ).strip()

        if text == "":
            return None

        try:
            return float(text)
        except ValueError:
            print("Please enter a number, for example 3.5, or blank to skip.")


def run_calibration(args: argparse.Namespace) -> None:
    """
    Run the validation-only steering pulse calibration tool.

    This program sends LEFT_PULSE and RIGHT_PULSE commands only. It does not
    use RouteManager, NavigationStateManager, AlignmentController, or any route
    execution logic.
    """
    csv_path = Path(args.csv).expanduser().resolve()
    ensure_csv_file(csv_path)
    print_menu(csv_path)

    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )

    if not serial_controller.connect():
        return

    try:
        while True:
            key = input("Command key: ").strip().lower()

            if key == "":
                continue

            if key == "q":
                break

            if key not in COMMAND_MAP:
                print("Unknown key. Use the menu commands above.")
                continue

            direction, pulse_ms = COMMAND_MAP[key]
            message = f"{direction}_PULSE {pulse_ms}"
            sent = serial_controller.send_raw_command(message, force=True)

            if not sent:
                print(f"Failed to send: {message}")
                continue

            print(f"Sent: {message}")
            print("Measure heading change and record result.")

            actual_rotation_deg = read_actual_rotation()
            if actual_rotation_deg is None:
                print("CSV row skipped.")
                continue

            append_measurement(
                csv_path,
                pulse_ms,
                direction,
                actual_rotation_deg,
            )
            print(
                "Recorded: "
                f"{pulse_ms},{direction},{actual_rotation_deg:g}"
            )
    except KeyboardInterrupt:
        print("\nStopping steering calibration.")
    finally:
        serial_controller.send_raw_command("STOP", force=True)
        serial_controller.disconnect()


def parse_args() -> argparse.Namespace:
    """Parse steering calibration command-line options."""
    parser = argparse.ArgumentParser(
        description="Validate steering pulse duration versus heading change.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--csv",
        default=str(CSV_PATH),
        help="CSV file used to store measured steering calibration rows.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_calibration(parse_args())
