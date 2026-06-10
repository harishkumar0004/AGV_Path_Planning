import argparse
import time

from communication.serial_motor_controller import SerialMotorController
from core.application_state import ApplicationState
from navigation.motion_commands import MotionCommand
from navigation.navigation_state import NavigationState
from navigation.navigation_state_manager import NavigationStateManager
from perception.perception_manager import PerceptionManager


def send_motion_command(
    serial_controller: SerialMotorController,
    command: MotionCommand,
    turn_right_value: float | None,
) -> bool:
    """
    Send one motion command to ESP32.

    Args:
        serial_controller: Serial link to ESP32.
        command: Motion command from NavigationStateManager.
        turn_right_value: Optional calibrated TURN_RIGHT value.

    Returns:
        True when the command was sent or already active.
    """
    if command == MotionCommand.TURN_RIGHT and turn_right_value is not None:
        print(f"Calibrated Turn Command: TURN_RIGHT {turn_right_value:g}")
        return serial_controller.send_command(
            command,
            value=turn_right_value,
            force=True,
        )

    return serial_controller.send_command(command)


def run_route_completion_test(args: argparse.Namespace) -> None:
    """
    Run the physical Tag 1 -> Tag 2 -> Tag 3 -> Tag 4 route test.

    Args:
        args: Parsed command-line arguments.
    """
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    navigation_manager = NavigationStateManager()
    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )
    stop_time: float | None = None
    tag_4_detected = False

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    print("Route completion test started.")
    print("Place Tag 1 in view to start automatically.")
    print("This test uses camera perception and prints Tag 4 detection status.")

    try:
        while True:
            perception_manager.update()
            perception = application_state.perception
            tag_id = perception.tag_id if perception.tag_visible else None

            if tag_id == 4:
                tag_4_detected = True
                print("Tag 4 detected after turn.")

            motion_complete = False
            if navigation_manager.state == NavigationState.TURNING:
                if navigation_manager.stop_before_turn_sent and not navigation_manager.turn_right_sent:
                    if stop_time is None:
                        stop_time = time.monotonic()

                    motion_complete = (
                        time.monotonic() - stop_time
                    ) >= args.stop_to_turn_delay
                else:
                    stop_time = None
            else:
                stop_time = None

            command = navigation_manager.update(
                tag_id=tag_id,
                start_pressed=tag_id == 1,
                motion_complete=motion_complete,
            )

            if command is not None:
                send_motion_command(
                    serial_controller,
                    command,
                    args.turn_right_value,
                )

            if navigation_manager.state == NavigationState.ROUTE_COMPLETE:
                print("Route Complete")
                print(f"Tag 4 Verification: {'PASS' if tag_4_detected else 'FAIL'}")
                break
    except KeyboardInterrupt:
        print("Stopping route completion test.")
    finally:
        serial_controller.send_command(MotionCommand.STOP, force=True)
        serial_controller.disconnect()
        perception_manager.release()


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for route completion testing."""
    parser = argparse.ArgumentParser(
        description="Validate calibrated turn and Tag 4 route completion.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--turn-right-value",
        type=float,
        default=None,
        help="Calibrated value sent as TURN_RIGHT <value> at Tag 3.",
    )
    parser.add_argument(
        "--stop-to-turn-delay",
        type=float,
        default=0.5,
        help="Seconds to wait after STOP before sending TURN_RIGHT.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    run_route_completion_test(parse_args())
