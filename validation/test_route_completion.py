import argparse
import time

from communication.serial_motor_controller import SerialMotorController
from core.application_state import ApplicationState
from navigation.alignment_controller import AlignmentController
from navigation.motion_commands import MotionCommand
from navigation.navigation_state import NavigationState
from navigation.navigation_state_manager import NavigationStateManager
from perception.perception_manager import PerceptionManager


def send_motion_command(
    serial_controller: SerialMotorController,
    command: MotionCommand,
    turn_right_value: float | None,
    use_calibrated_turn: bool = True,
) -> bool:
    """
    Send one motion command to ESP32.

    Args:
        serial_controller: Serial link to ESP32.
        command: Motion command from NavigationStateManager.
        turn_right_value: Optional calibrated TURN_RIGHT value.
        use_calibrated_turn: True only for route turn, not alignment corrections.

    Returns:
        True when the command was sent or already active.
    """
    if (
        use_calibrated_turn
        and command == MotionCommand.TURN_RIGHT
        and turn_right_value is not None
    ):
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
    alignment_controller = AlignmentController(
        center_tolerance_px=args.alignment_tolerance,
    )
    serial_controller = SerialMotorController(
        port=args.port,
        baudrate=args.baudrate,
    )
    last_status_query_time = 0.0
    last_alignment_command: MotionCommand | None = None
    last_alignment_command_time = 0.0
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
            frame = perception_manager.last_frame
            perception = application_state.perception
            tag_id = perception.tag_id if perception.tag_visible else None

            if tag_id == 4:
                tag_4_detected = True
                print("Tag 4 detected after turn.")

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

            command = navigation_manager.update(
                tag_id=tag_id,
                start_pressed=tag_id == 1,
                motion_complete=motion_complete,
                alignment_complete=alignment_complete,
                tag_visible=turn_tag_visible,
            )

            if command is not None:
                last_alignment_command = None
                send_motion_command(
                    serial_controller,
                    command,
                    args.turn_right_value,
                    use_calibrated_turn=True,
                )
            elif alignment_command is not None:
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
    return parser.parse_args()


if __name__ == "__main__":
    run_route_completion_test(parse_args())
