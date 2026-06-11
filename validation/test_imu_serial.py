from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

DEPENDENCY_ERROR: ModuleNotFoundError | None = None
CV2_AVAILABLE = True

try:
    from communication.imu_serial_reader import ImuSerialReader
except ModuleNotFoundError as error:
    DEPENDENCY_ERROR = error

try:
    import cv2
    import numpy as np
except ModuleNotFoundError:
    CV2_AVAILABLE = False


class SerialRateMonitor:
    """Measures valid HEADING message rate from ESP32."""

    def __init__(self, window_sec: float = 1.0) -> None:
        """
        Create a serial rate monitor.

        Args:
            window_sec: Time window used to calculate rate.
        """
        self.window_sec = window_sec
        self.window_start = time.monotonic()
        self.message_count = 0
        self.rate_hz = 0.0

    def record_message(self) -> float:
        """
        Record one valid heading message.

        Returns:
            Current message rate in Hz.
        """
        self.message_count += 1
        now = time.monotonic()
        elapsed = now - self.window_start

        if elapsed >= self.window_sec:
            self.rate_hz = self.message_count / elapsed
            self.message_count = 0
            self.window_start = now

        return self.rate_hz


def draw_overlay(
    heading: float | None,
    serial_rate_hz: float,
    status: str,
) -> None:
    """
    Draw current IMU serial data in an OpenCV window.

    Args:
        heading: Latest heading in degrees.
        serial_rate_hz: Valid heading message rate.
        status: Connection/status text.
    """
    if not CV2_AVAILABLE:
        return

    frame = np.zeros((260, 640, 3), dtype=np.uint8)
    heading_text = f"{heading:.2f} deg" if heading is not None else "None"

    lines = [
        f"Heading: {heading_text}",
        f"Serial Rate: {serial_rate_hz:.1f} Hz",
        f"Status: {status}",
        "Rotate AGV manually to verify heading direction.",
        "Q: Quit",
    ]

    y = 40
    for line in lines:
        cv2.putText(
            frame,
            line,
            (30, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
        )
        y += 40

    cv2.imshow("IMU Serial Validation", frame)


def run_validation(args: argparse.Namespace) -> None:
    """Connect to ESP32 and display received HEADING values."""
    imu_reader = ImuSerialReader(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
    )
    rate_monitor = SerialRateMonitor()
    last_print_time = 0.0

    if not imu_reader.connect():
        return

    print("Waiting for HEADING lines from ESP32...")

    try:
        while True:
            received_heading = imu_reader.update()

            if received_heading:
                serial_rate_hz = rate_monitor.record_message()
            else:
                serial_rate_hz = rate_monitor.rate_hz

            heading = imu_reader.get_heading()
            now = time.monotonic()

            if (now - last_print_time) >= args.print_interval:
                heading_text = f"{heading:.2f} deg" if heading is not None else "None"
                print(
                    f"Heading: {heading_text} | "
                    f"Serial Rate: {serial_rate_hz:.1f} Hz | "
                    f"Status: {imu_reader.latest_status}"
                )
                last_print_time = now

            draw_overlay(
                heading,
                serial_rate_hz,
                imu_reader.latest_status,
            )

            if CV2_AVAILABLE:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q")):
                    break

            time.sleep(args.loop_delay)
    except KeyboardInterrupt:
        print("Stopping IMU serial validation.")
    finally:
        imu_reader.disconnect()
        if CV2_AVAILABLE:
            cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse IMU serial validation options."""
    parser = argparse.ArgumentParser(
        description="Validate ESP32 to Raspberry Pi IMU heading serial data.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument(
        "--print-interval",
        type=float,
        default=0.25,
        help="Seconds between terminal status lines.",
    )
    parser.add_argument(
        "--loop-delay",
        type=float,
        default=0.005,
        help="Small sleep to avoid busy-waiting.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    if any(argument in ("-h", "--help") for argument in sys.argv[1:]):
        parse_args()
        raise SystemExit(0)

    if DEPENDENCY_ERROR is not None:
        print(
            "IMU serial validation cannot start because a required dependency "
            f"is missing: {DEPENDENCY_ERROR.name}"
        )
        print("Install Raspberry Pi serial dependency:")
        print("  pip install pyserial")
        raise SystemExit(1)

    run_validation(parse_args())
