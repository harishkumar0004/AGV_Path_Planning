import serial

from control.serial_commands import SerialCommand


class SerialMotorController:
    """Sends simple text motor commands over serial to the ESP32."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout: float = 1.0,
    ) -> None:
        """
        Create a serial motor controller.

        Args:
            port: Serial port connected to the ESP32.
            baudrate: Serial baud rate used by both Pi and ESP32.
            timeout: Serial read/write timeout in seconds.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection: serial.Serial | None = None
        self.last_command: SerialCommand | None = None

    def connect(self) -> bool:
        """
        Open the serial connection to the ESP32.

        Returns:
            True if the connection opens successfully, otherwise False.
        """
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
            print(f"Connected to ESP32 on {self.port}")
            return True
        except serial.SerialException as error:
            print(f"Serial connection failed: {error}")
            self.serial_connection = None
            return False

    def send_command(self, command: SerialCommand) -> bool:
        """
        Send one human-readable command to the ESP32.

        Args:
            command: SerialCommand value to send.

        Returns:
            True if the command was sent, otherwise False.
        """
        if self.serial_connection is None or not self.serial_connection.is_open:
            print("Serial connection is not open.")
            return False

        if command == self.last_command:
            return True

        print(f"TX: {command.value}")
        message = f"{command.value}\n"
        self.serial_connection.write(message.encode("utf-8"))
        self.last_command = command
        return True

    def disconnect(self) -> None:
        """Close the serial connection if it is open."""
        if self.serial_connection is not None and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Disconnected from ESP32.")

        self.serial_connection = None
        self.last_command = None
