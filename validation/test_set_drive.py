from __future__ import annotations

import argparse
import time

import serial


def send_command(serial_connection: serial.Serial, command: str) -> None:
    """
    Send one command to ESP32 and print it.

    Args:
        serial_connection: Open serial connection.
        command: Human-readable ESP32 command.
    """
    print(f"TX: {command}")
    serial_connection.write(f"{command}\n".encode("utf-8"))
    serial_connection.flush()
    time.sleep(0.2)

    while serial_connection.in_waiting > 0:
        line = serial_connection.readline().decode(
            "utf-8",
            errors="replace",
        ).strip()
        if line:
            print(line)


def run_test(port: str, baudrate: int, hold_time_sec: float) -> None:
    """
    Run a manual SET_DRIVE steering test.

    Args:
        port: USB serial port connected to ESP32.
        baudrate: ESP32 serial baud rate.
        hold_time_sec: Seconds to hold each drive command.
    """
    commands = [
        "SET_DRIVE 10000 10000",
        "SET_DRIVE 9000 11000",
        "SET_DRIVE 11000 9000",
        "STOP",
    ]

    with serial.Serial(port=port, baudrate=baudrate, timeout=0.1) as serial_connection:
        time.sleep(2.0)

        for command in commands:
            send_command(serial_connection, command)
            time.sleep(hold_time_sec)


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Manual SET_DRIVE validation for ESP32 differential drive.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--hold-time", type=float, default=2.0)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run_test(args.port, args.baudrate, args.hold_time)
