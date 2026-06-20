import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agv_high_level.communication.esp32_client import ESP32Client
from agv_high_level.config.settings import ESP32_BAUDRATE, ESP32_PORT


def print_responses(client: ESP32Client, duration: float = 0.75) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        for line in client.read_lines():
            print(line)
        time.sleep(0.02)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Exercise the latest ESP32 status and alignment commands."
    )
    parser.add_argument("--port", default=ESP32_PORT)
    parser.add_argument("--baudrate", type=int, default=ESP32_BAUDRATE)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    client = ESP32Client(port=args.port, baudrate=args.baudrate)

    try:
        client.connect()

        client.status()
        print_responses(client)

        client.align_mode_pose()
        print_responses(client)

        client.nav_status()
        print_responses(client)
    finally:
        if client.connected:
            client.stop()
            print_responses(client, duration=0.25)
        client.close()


if __name__ == "__main__":
    main()
