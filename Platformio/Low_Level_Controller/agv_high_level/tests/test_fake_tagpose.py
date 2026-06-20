import argparse
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agv_high_level.communication.esp32_client import ESP32Client
from agv_high_level.config.settings import ESP32_BAUDRATE, ESP32_PORT
from agv_high_level.perception.tag_pose import TagPose
from agv_high_level.perception.tag_pose_streamer import TagPoseStreamer


def print_responses(client: ESP32Client, duration: float = 0.75) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        for line in client.read_lines():
            print(line)
        time.sleep(0.02)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send visible and lost fake TAGPOSE updates to the ESP32."
    )
    parser.add_argument("--port", default=ESP32_PORT)
    parser.add_argument("--baudrate", type=int, default=ESP32_BAUDRATE)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    client = ESP32Client(port=args.port, baudrate=args.baudrate)
    streamer = TagPoseStreamer(client)

    try:
        client.connect()

        streamer.update(
            TagPose(
                visible=True,
                tag_id=0,
                x_m=0.0011,
                y_m=-0.0009,
                z_m=0.11,
                yaw_deg=-0.90,
                timestamp=time.monotonic(),
            )
        )
        time.sleep(0.2)
        client.status()
        print_responses(client)

        streamer.update(TagPose.lost())
        time.sleep(0.2)
        client.status()
        print_responses(client)
    finally:
        if client.connected:
            client.stop()
            print_responses(client, duration=0.25)
        client.close()


if __name__ == "__main__":
    main()
