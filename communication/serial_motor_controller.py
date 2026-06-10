import time

import serial

from navigation.motion_commands import MotionCommand


class SerialMotorController:
    """Sends deduplicated text motion commands from Raspberry Pi to ESP32."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout: float = 1.0,
        reconnect_delay_sec: float = 0.5,
    ) -> None:
        """
        Create a serial motor controller.

        Args:
            port: USB serial port connected to the ESP32.
            baudrate: Serial baud rate used by Raspberry Pi and ESP32.
            timeout: Serial timeout in seconds.
            reconnect_delay_sec: Delay before retrying a failed reconnect.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.reconnect_delay_sec = reconnect_delay_sec
        self.serial_connection: serial.Serial | None = None
        self.last_command: MotionCommand | None = None
        self.last_message: str | None = None

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
            self.last_command = None
            self.last_message = None
            print(f"Connected to ESP32 on {self.port}")
            return True
        except serial.SerialException as error:
            print(f"Serial connection failed: {error}")
            self.serial_connection = None
            return False

    def send_command(
        self,
        command: MotionCommand,
        value: float | int | None = None,
        force: bool = False,
    ) -> bool:
        """
        Send one motion command if it changed from the previous command.

        Args:
            command: Motion command to send to ESP32.
            value: Optional numeric command value, for example TURN_RIGHT 30.
            force: True to transmit even if the command matches the previous one.

        Returns:
            True if command is already sent or successfully transmitted.
        """
        message = self._format_message(command, value)

        if not force and message == self.last_message:
            return True

        if not self._ensure_connected():
            return False

        return self._write_message(message, command)

    def send_raw_command(self, message: str, force: bool = True) -> bool:
        """
        Send a raw newline-terminated command string to ESP32.

        Args:
            message: Text command, such as TURN_RIGHT 20.
            force: True to transmit even if it matches the previous message.

        Returns:
            True if command is already sent or successfully transmitted.
        """
        formatted_message = message.strip()

        if not formatted_message:
            return False

        if not force and formatted_message == self.last_message:
            return True

        if not self._ensure_connected():
            return False

        return self._write_message(formatted_message)

    def disconnect(self) -> None:
        """Close the serial connection."""
        if self.serial_connection is not None and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Disconnected from ESP32.")

        self.serial_connection = None
        self.last_command = None
        self.last_message = None

    def _ensure_connected(self) -> bool:
        """
        Ensure the serial connection is open, reconnecting if needed.

        Returns:
            True if serial is connected, otherwise False.
        """
        if self.serial_connection is not None and self.serial_connection.is_open:
            return True

        print("Serial connection lost. Reconnecting...")
        time.sleep(self.reconnect_delay_sec)
        return self.connect()

    def _format_message(
        self,
        command: MotionCommand,
        value: float | int | None,
    ) -> str:
        """
        Convert a command and optional value into the serial protocol text.

        Args:
            command: Motion command to transmit.
            value: Optional numeric command value.

        Returns:
            Human-readable serial message without the newline.
        """
        if value is None:
            return command.value

        return f"{command.value} {value:g}"

    def _write_message(
        self,
        message: str,
        command: MotionCommand | None = None,
    ) -> bool:
        """
        Write one newline-terminated text command to ESP32.

        Args:
            message: Command message to transmit without newline.
            command: Optional MotionCommand used for compatibility tracking.

        Returns:
            True if transmission succeeds, otherwise False.
        """
        if self.serial_connection is None:
            return False

        try:
            serial_message = f"{message}\n"
            self.serial_connection.write(serial_message.encode("utf-8"))
            self.serial_connection.flush()
            self.last_command = command
            self.last_message = message
            print(f"TX: {message}")
            return True
        except serial.SerialException as error:
            print(f"Serial write failed: {error}")
            self.disconnect()
            return False
