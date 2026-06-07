from controller.agv_controller import AGVController


def main() -> None:
    """Run the AGV controller loop for live perception state transitions."""
    controller = AGVController()

    if not controller.initialize():
        print("AGVController failed to initialize.")
        return

    try:
        while controller.is_initialized:
            # Each update runs: perception -> events -> state machine.
            controller.update()
    except KeyboardInterrupt:
        print("Stopping AGVController.")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
