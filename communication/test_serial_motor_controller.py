from communication.serial_motor_controller import SerialMotorController
from navigation.motion_commands import MotionCommand


def main() -> None:
    """Manually test Raspberry Pi to ESP32 serial motion commands."""
    controller = SerialMotorController(port="/dev/ttyUSB0", baudrate=115200)

    if not controller.connect():
        return

    try:
        commands = [
            MotionCommand.START_FORWARD,
            MotionCommand.START_FORWARD,
            MotionCommand.START_SLOW_FORWARD,
            MotionCommand.START_SLOW_FORWARD,
            MotionCommand.TURN_RIGHT,
            MotionCommand.TURN_RIGHT,
            MotionCommand.STOP,
        ]

        for command in commands:
            controller.send_command(command)
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
