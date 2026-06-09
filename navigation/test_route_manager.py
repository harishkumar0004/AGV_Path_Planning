import sys
import time

from communication.serial_motor_controller import SerialMotorController
from navigation.route_actions import RouteAction
from navigation.route_manager import RouteManager
from navigation.motion_commands import MotionCommand


def route_action_to_motion_command(action: RouteAction) -> MotionCommand:
    """
    Convert a route action into a serial motion command.

    Args:
        action: Route action produced by RouteManager.

    Returns:
        MotionCommand sent to ESP32.
    """
    if action == RouteAction.START_FORWARD:
        return MotionCommand.START_FORWARD

    if action == RouteAction.START_SLOW_FORWARD:
        return MotionCommand.START_SLOW_FORWARD

    if action == RouteAction.TURN_RIGHT:
        return MotionCommand.TURN_RIGHT

    return MotionCommand.STOP


def run_simulation() -> None:
    """Demonstrate fixed-route execution with repeated tag frames."""
    route_manager = RouteManager()

    # Repeated tags simulate the same AprilTag staying visible for many frames.
    detected_tags = [
        1,
        1,
        2,
        2,
        2,
        3,
        3,
        3,
        4,
        4,
    ]

    for tag_id in detected_tags:
        motion_idle = route_manager.state.waiting_for_turn_idle
        action = route_manager.process_tag(tag_id, motion_idle=motion_idle)
        if action is not None:
            print(f"Route Action: {action.name}")


def run_live_route() -> None:
    """Run the physical Tag 1 -> Tag 2 -> Tag 3 -> Tag 4 route test."""
    from core.application_state import ApplicationState
    from perception.perception_manager import PerceptionManager

    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    route_manager = RouteManager()
    serial_controller = SerialMotorController()
    stop_before_turn_time: float | None = None
    stop_before_turn_delay_sec = 0.5

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    try:
        while True:
            perception_manager.update()
            perception = application_state.perception
            tag_id = perception.tag_id if perception.tag_visible else None
            motion_idle = False

            if route_manager.state.waiting_for_turn_idle:
                if stop_before_turn_time is None:
                    stop_before_turn_time = time.monotonic()

                elapsed_time = time.monotonic() - stop_before_turn_time
                motion_idle = elapsed_time >= stop_before_turn_delay_sec
                tag_id = route_manager.state.pending_turn_tag
            else:
                stop_before_turn_time = None

            action = route_manager.process_tag(tag_id, motion_idle=motion_idle)
            if action is not None:
                command = route_action_to_motion_command(action)
                serial_controller.send_command(command)

                if action == RouteAction.STOP:
                    stop_before_turn_time = time.monotonic()

            if route_manager.state.route_completed:
                break
    except KeyboardInterrupt:
        print("Stopping route test.")
    finally:
        serial_controller.send_command(MotionCommand.STOP)
        serial_controller.disconnect()
        perception_manager.release()


def main() -> None:
    """Run route simulation by default, or live physical test with --live."""
    if len(sys.argv) > 1 and sys.argv[1] == "--live":
        run_live_route()
        return

    run_simulation()


if __name__ == "__main__":
    main()
