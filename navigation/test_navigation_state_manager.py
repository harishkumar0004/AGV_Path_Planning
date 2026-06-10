import time

import cv2

from communication.serial_motor_controller import SerialMotorController
from core.application_state import ApplicationState
from navigation.alignment_controller import AlignmentController
from navigation.motion_commands import MotionCommand
from navigation.navigation_state import NavigationState
from navigation.navigation_state_manager import NavigationStateManager
from perception.perception_manager import PerceptionManager


def draw_overlay(
    frame,
    navigation_manager: NavigationStateManager,
    tag_id: int | None,
    alignment_error: float | None,
) -> None:
    """
    Draw navigation status on the camera frame.

    Args:
        frame: Camera image to annotate.
        navigation_manager: Navigation manager containing current state/action.
        tag_id: Current visible AprilTag ID, or None.
        alignment_error: Horizontal Tag 3 alignment error in pixels.
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
        f"Alignment Error: {alignment_error}",
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
    alignment_controller = AlignmentController(center_tolerance_px=10.0)
    serial_controller = SerialMotorController()
    last_status_query_time = 0.0
    last_alignment_command: MotionCommand | None = None
    last_alignment_command_time = 0.0
    status_query_interval_sec = 0.1
    alignment_command_interval_sec = 0.5

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
            if navigation_manager.state in {
                NavigationState.STOPPING,
                NavigationState.ALIGNING,
                NavigationState.TURNING,
            }:
                now = time.monotonic()
                if (now - last_status_query_time) >= status_query_interval_sec:
                    motion_running = serial_controller.is_motion_running()
                    motion_complete = motion_running is False
                    last_status_query_time = now

            alignment_error: float | None = None
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
                    alignment_complete = abs(alignment_error) <= 10.0
                    print(f"Alignment Error: {alignment_error:.1f}")

                    if not alignment_complete:
                        alignment_command = alignment_result.action
                else:
                    alignment_command = MotionCommand.STOP

            command = navigation_manager.update(
                tag_id=tag_id,
                start_pressed=start_pressed,
                motion_complete=motion_complete,
                alignment_complete=alignment_complete,
                tag_visible=turn_tag_visible,
            )

            if command is not None:
                last_alignment_command = None
                if command == MotionCommand.TURN_RIGHT:
                    serial_controller.send_command(command, value=50, force=True)
                else:
                    serial_controller.send_command(command)
            elif alignment_command is not None:
                now = time.monotonic()
                alignment_due = (
                    alignment_command != last_alignment_command
                    or (now - last_alignment_command_time)
                    >= alignment_command_interval_sec
                )

                if alignment_due:
                    serial_controller.send_command(alignment_command)
                    last_alignment_command = alignment_command
                    last_alignment_command_time = now

            if frame is not None:
                draw_overlay(frame, navigation_manager, tag_id, alignment_error)
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
