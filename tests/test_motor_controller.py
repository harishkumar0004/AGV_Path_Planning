from control.motor_controller import MotorController


def main() -> None:
    """Call each simulated motor command once."""
    motor_controller = MotorController()

    # These commands only print to the terminal. No hardware is used.
    motor_controller.forward(50)
    motor_controller.backward(50)
    motor_controller.turn_left(25)
    motor_controller.turn_right(25)
    motor_controller.stop()


if __name__ == "__main__":
    main()
