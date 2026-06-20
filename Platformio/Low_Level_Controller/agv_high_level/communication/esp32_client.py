from __future__ import annotations

from typing import Optional

import serial

from agv_high_level.config.settings import ESP32_BAUDRATE, ESP32_PORT


class ESP32Client:
    def __init__(
        self,
        port: str = ESP32_PORT,
        baudrate: int = ESP32_BAUDRATE,
        timeout: float = 0.1,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        if self.connected:
            return

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def send_line(self, line: str) -> None:
        connection = self._require_connection()
        command = line.rstrip("\r\n")
        if not command:
            raise ValueError("ESP32 command cannot be empty")

        connection.write(f"{command}\n".encode("utf-8"))
        connection.flush()

    def read_lines(self) -> list[str]:
        connection = self._require_connection()
        lines: list[str] = []

        while connection.in_waiting > 0:
            raw_line = connection.readline()
            line = raw_line.decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)

        return lines

    def stop(self) -> None:
        self.send_line("STOP")

    def status(self) -> None:
        self.send_line("STATUS")

    def align_mode_pose(self) -> None:
        self.send_line("ALIGN MODE POSE")

    def align_on(self) -> None:
        self.send_line("ALIGN ON")

    def align_off(self) -> None:
        self.send_line("ALIGN OFF")

    def nav_start(self) -> None:
        self.send_line("NAV START")

    def nav_stop(self) -> None:
        self.send_line("NAV STOP")

    def nav_reset(self) -> None:
        self.send_line("NAV RESET")

    def nav_status(self) -> None:
        self.send_line("NAV STATUS")

    def send_tagpose(
        self,
        tag_id: int,
        x_m: float,
        y_m: float,
        yaw_deg: float,
    ) -> None:
        # The latest ESP32 TAGPOSE command does not include the observed tag ID.
        _ = tag_id
        self.send_line(f"TAGPOSE {x_m:.4f} {y_m:.4f} {yaw_deg:.2f}")

    def send_tag_lost(self) -> None:
        self.send_line("TAG LOST")

    def _require_connection(self) -> serial.Serial:
        if not self.connected or self._serial is None:
            raise RuntimeError("ESP32 client is not connected")
        return self._serial
