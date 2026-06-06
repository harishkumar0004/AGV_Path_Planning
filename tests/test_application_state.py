from core.application_state import ApplicationState
from perception.perception_manager import PerceptionManager


def main() -> None:
    """Continuously update perception and print shared ApplicationState."""
    application_state = ApplicationState()
    perception_manager = PerceptionManager(application_state)

    if not perception_manager.initialize():
        print("PerceptionManager failed to initialize camera.")
        return

    try:
        while True:
            # Update perception from camera frame to AprilTag detections.
            perception_manager.update()

            # Print only the perception part of the shared ApplicationState.
            perception = application_state.perception
            print(
                "PerceptionState("
                f"tag_visible={perception.tag_visible}, "
                f"tag_id={perception.tag_id}, "
                f"center_x={perception.center_x}, "
                f"center_y={perception.center_y}"
                ")"
            )
    except KeyboardInterrupt:
        print("Stopping perception ApplicationState test.")
    finally:
        # Always release camera resources before exiting.
        perception_manager.release()


if __name__ == "__main__":
    main()
