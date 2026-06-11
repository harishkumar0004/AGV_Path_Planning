from __future__ import annotations

import time

import serial


class ImuSerialReader:
    """Reads HEADING lines sent from the ESP32 IMU manager."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout: float = 0.1,
        reconnect_delay_sec: float = 0.5,
    ) -> None:
        """
        Create an IMU serial reader.

        Args:
            port: USB serial port connected to the ESP32.
            baudrate: Serial baud rate used by ESP32 and Raspberry Pi.
            timeout: Serial read timeout in seconds.
            reconnect_delay_sec: Delay before reconnect attempts.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.reconnect_delay_sec = reconnect_delay_sec
        self.serial_connection: serial.Serial | None = None
        self.latest_heading_deg: float | None = None
        self.latest_status: str = "DISCONNECTED"
        self.last_heading_time: float | None = None

    def connect(self) -> bool:
        """
        Open the serial connection to ESP32.

        Returns:
            True when the connection opens successfully.
        """
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
            self.latest_status = "CONNECTED"
            print(f"Connected to ESP32 on {self.port}")
            return True
        except serial.SerialException as error:
            print(f"IMU serial connection failed: {error}")
            self.serial_connection = None
            self.latest_status = "DISCONNECTED"
            return False

    def update(self) -> bool:
        """
        Read available serial lines and update the latest heading.

        Returns:
            True when a new HEADING value was parsed.
        """
        if not self._ensure_connected() or self.serial_connection is None:
            return False

        parsed_heading = False

        try:
            while self.serial_connection.in_waiting > 0:
                line = self.serial_connection.readline().decode(
                    "utf-8",
                    errors="replace",
                ).strip()

                if self._parse_line(line):
                    parsed_heading = True

            return parsed_heading
        except serial.SerialException as error:
            print(f"IMU serial read failed: {error}")
            self.disconnect()
            return False

    def get_heading(self) -> float | None:
        """
        Return the latest received relative heading.

        Returns:
            Heading in degrees, or None before the first valid HEADING line.
        """
        return self.latest_heading_deg

    def disconnect(self) -> None:
        """Close the serial connection."""
        if self.serial_connection is not None and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Disconnected from ESP32 IMU serial.")

        self.serial_connection = None
        self.latest_status = "DISCONNECTED"

    def _ensure_connected(self) -> bool:
        """
        Ensure the serial connection is open, reconnecting when needed.

        Returns:
            True when serial is connected.
        """
        if self.serial_connection is not None and self.serial_connection.is_open:
            return True

        self.latest_status = "RECONNECTING"
        time.sleep(self.reconnect_delay_sec)
        return self.connect()

    def _parse_line(self, line: str) -> bool:
        """
        Parse one ESP32 serial line.

        Args:
            line: Text line received from ESP32.

        Returns:
            True if the line contained a valid HEADING value.
        """
        if not line:
            return False

        if line in {"BOOT", "IMU_CALIBRATING", "IMU_READY", "IMU_ERROR"}:
            self.latest_status = line
            return False

        if not line.startswith("HEADING:"):
            return False

        value_text = line.split(":", maxsplit=1)[1].strip()

        try:
            self.latest_heading_deg = float(value_text)
            self.last_heading_time = time.monotonic()
            self.latest_status = "CONNECTED"
            return True
        except ValueError:
            return False
