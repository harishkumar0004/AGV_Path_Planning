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
            print(f"Connected to ESP32 on {self.port}")
            return True
        except serial.SerialException as error:
            print(f"Serial connection failed: {error}")
            self.serial_connection = None
            return False

    def send_command(self, command: MotionCommand) -> bool:
        """
        Send one motion command if it changed from the previous command.

        Args:
            command: Motion command to send to ESP32.

        Returns:
            True if command is already sent or successfully transmitted.
        """
        if command == self.last_command:
            return True

        if not self._ensure_connected():
            return False

        return self._write_command(command)

    def disconnect(self) -> None:
        """Close the serial connection."""
        if self.serial_connection is not None and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Disconnected from ESP32.")

        self.serial_connection = None
        self.last_command = None

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

    def _write_command(self, command: MotionCommand) -> bool:
        """
        Write one newline-terminated text command to ESP32.

        Args:
            command: Motion command to transmit.

        Returns:
            True if transmission succeeds, otherwise False.
        """
        if self.serial_connection is None:
            return False

        try:
            message = f"{command.value}\n"
            self.serial_connection.write(message.encode("utf-8"))
            self.serial_connection.flush()
            self.last_command = command
            print(f"TX: {command.value}")
            return True
        except serial.SerialException as error:
            print(f"Serial write failed: {error}")
            self.disconnect()
            return False
