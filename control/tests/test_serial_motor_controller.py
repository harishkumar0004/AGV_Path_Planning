from control.serial_commands import SerialCommand
from control.serial_motor_controller import SerialMotorController


def main() -> None:
    """Send a simple serial command sequence to the Arduino Mega."""
    controller = SerialMotorController(port="/dev/ttyUSB0", baudrate=115200)

    if not controller.connect():
        return

    try:
        # Send only simple text commands. No JSON or binary protocol is used.
        controller.send_command(SerialCommand.FORWARD)
        controller.send_command(SerialCommand.STOP)

        controller.send_command(SerialCommand.LEFT)
        controller.send_command(SerialCommand.STOP)

        controller.send_command(SerialCommand.RIGHT)
        controller.send_command(SerialCommand.STOP)

        controller.send_command(SerialCommand.BACKWARD)
        controller.send_command(SerialCommand.STOP)
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
