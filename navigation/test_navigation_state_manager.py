import time

import cv2

from communication.serial_motor_controller import SerialMotorController
from core.application_state import ApplicationState
from navigation.motion_commands import MotionCommand
from navigation.navigation_state import NavigationState
from navigation.navigation_state_manager import NavigationStateManager
from perception.perception_manager import PerceptionManager


def draw_overlay(
    frame,
    navigation_manager: NavigationStateManager,
    tag_id: int | None,
) -> None:
    """
    Draw navigation status on the camera frame.

    Args:
        frame: Camera image to annotate.
        navigation_manager: Navigation manager containing current state/action.
        tag_id: Current visible AprilTag ID, or None.
    """
    action_text = (
        navigation_manager.current_action.name
        if navigation_manager.current_action is not None
        else "None"
    )

    lines = [
        f"State: {navigation_manager.state.name}",
        f"Tag ID: {tag_id}",
        f"Action: {action_text}",
        navigation_manager.status_message,
    ]

    y = 35
    for line in lines:
        cv2.putText(
            frame,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
        )
        y += 35


def main() -> None:
    """Run the live navigation state manager test."""
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    navigation_manager = NavigationStateManager()
    serial_controller = SerialMotorController()
    stop_time: float | None = None
    stop_to_turn_delay_sec = 0.5

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    if not serial_controller.connect():
        perception_manager.release()
        return

    try:
        while True:
            perception_manager.update()
            frame = perception_manager.last_frame
            perception = application_state.perception
            tag_id = perception.tag_id if perception.tag_visible else None

            key = cv2.waitKey(1) & 0xFF
            start_pressed = key == ord("f") or key == ord("F")

            motion_complete = False
            if navigation_manager.state == NavigationState.TURNING:
                if navigation_manager.stop_before_turn_sent and not navigation_manager.turn_right_sent:
                    if stop_time is None:
                        stop_time = time.monotonic()

                    motion_complete = (
                        time.monotonic() - stop_time
                    ) >= stop_to_turn_delay_sec
                else:
                    stop_time = None
            else:
                stop_time = None

            command = navigation_manager.update(
                tag_id=tag_id,
                start_pressed=start_pressed,
                motion_complete=motion_complete,
            )

            if command is not None:
                serial_controller.send_command(command)

            if frame is not None:
                draw_overlay(frame, navigation_manager, tag_id)
                cv2.imshow("AGV Navigation State Manager", frame)

            if key == ord("q"):
                break

            if navigation_manager.state == NavigationState.ROUTE_COMPLETE:
                navigation_manager.status_message = "Route Complete"
    except KeyboardInterrupt:
        print("Stopping navigation state manager test.")
    finally:
        serial_controller.send_command(MotionCommand.STOP)
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
