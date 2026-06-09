import time

import cv2

from communication.serial_motor_controller import SerialMotorController
from core.application_state import ApplicationState
from navigation.motion_commands import MotionCommand
from navigation.navigation_state import NavigationState
from navigation.navigation_state_manager import NavigationStateManager
from perception.perception_manager import PerceptionManager
from validation.performance_monitor import PerformanceMonitor


def draw_metrics_overlay(frame, monitor: PerformanceMonitor) -> None:
    """
    Draw navigation validation metrics on the camera frame.

    Args:
        frame: Camera image to annotate.
        monitor: Performance monitor containing current metrics.
    """
    metrics = monitor.metrics
    latency_text = (
        f"{metrics.command_latency_ms:.1f} ms"
        if metrics.command_latency_ms is not None
        else "None"
    )

    lines = [
        f"Tag ID: {metrics.current_tag_id}",
        f"Camera FPS: {metrics.camera_fps:.1f}",
        f"Detection FPS: {metrics.detection_fps:.1f}",
        f"Navigation State: {metrics.navigation_state}",
        f"Current Action: {metrics.current_action}",
        f"Command Latency: {latency_text}",
        monitor.get_tag_count_text(),
    ]

    y = 30
    for line in lines:
        cv2.putText(
            frame,
            line,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        y += 28


def main() -> None:
    """Run live navigation validation with camera overlay and serial commands."""
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)
    navigation_manager = NavigationStateManager()
    serial_controller = SerialMotorController()
    monitor = PerformanceMonitor()
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
            monitor.record_camera_frame()
            perception_manager.update()
            monitor.record_detection_cycle()

            frame = perception_manager.last_frame
            perception = application_state.perception
            tag_id = perception.tag_id if perception.tag_visible else None
            monitor.record_tag_detection(tag_id)

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

            action_name = (
                navigation_manager.current_action.name
                if navigation_manager.current_action is not None
                else "None"
            )
            monitor.record_navigation_state(
                navigation_manager.state.name,
                action_name,
            )

            if command is not None:
                monitor.record_command_generated(command)
                serial_controller.send_command(command)
                monitor.record_command_transmitted()

            if frame is not None:
                draw_metrics_overlay(frame, monitor)
                cv2.imshow("AGV Navigation Performance", frame)

            if key == ord("q"):
                break
    except KeyboardInterrupt:
        print("Stopping navigation validation.")
    finally:
        serial_controller.send_command(MotionCommand.STOP)
        serial_controller.disconnect()
        perception_manager.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
