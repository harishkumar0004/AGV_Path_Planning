from control.motor_commands import MotorCommand


class MotorController:
    """Simulates motor control by printing motor commands to the terminal."""

    def forward(self, speed: int) -> None:
        """
        Simulate moving the robot forward.

        Args:
            speed: Motor speed value used for simulation output.
        """
        self._print_command(MotorCommand.FORWARD, speed)

    def backward(self, speed: int) -> None:
        """
        Simulate moving the robot backward.

        Args:
            speed: Motor speed value used for simulation output.
        """
        self._print_command(MotorCommand.BACKWARD, speed)

    def turn_left(self, speed: int) -> None:
        """
        Simulate turning the robot left.

        Args:
            speed: Motor speed value used for simulation output.
        """
        self._print_command(MotorCommand.TURN_LEFT, speed)

    def turn_right(self, speed: int) -> None:
        """
        Simulate turning the robot right.

        Args:
            speed: Motor speed value used for simulation output.
        """
        self._print_command(MotorCommand.TURN_RIGHT, speed)

    def stop(self) -> None:
        """Simulate stopping the robot."""
        print(MotorCommand.STOP.name)

    def _print_command(self, command: MotorCommand, speed: int) -> None:
        """
        Print one simulated motor command.

        Args:
            command: Motor command being simulated.
            speed: Speed value printed with the command.
        """
        print(f"{command.name} {speed}")
