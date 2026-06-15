from dataclasses import dataclass
from typing import Any

import numpy as np
from picamera2 import Picamera2


@dataclass
class CameraDiagnostics:
    """Stores camera configuration values used for startup diagnostics."""

    camera_model: str
    selected_mode: str
    configured_width: int
    configured_height: int
    pixel_format: str
    expected_fps: float
    actual_width: int | None = None
    actual_height: int | None = None


class CameraManager:
    """Manages direct access to the Raspberry Pi camera."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        pixel_format: str = "RGB888",
        target_fps: float = 30.0,
        print_diagnostics: bool = True,
    ) -> None:
        """
        Create a camera manager with the requested image size.

        Args:
            width: Width of the captured image in pixels.
            height: Height of the captured image in pixels.
            pixel_format: Camera output format requested from Picamera2.
            target_fps: Requested camera frame rate.
            print_diagnostics: True to print startup and first-frame details.
        """
        self.width = width
        self.height = height
        self.pixel_format = pixel_format
        self.target_fps = target_fps
        self.print_diagnostics = print_diagnostics
        self.camera: Picamera2 | None = None
        self.is_started = False
        self.diagnostics: CameraDiagnostics | None = None
        self._actual_frame_reported = False

    def initialize(self) -> bool:
        """
        Create, configure, and start the Raspberry Pi camera.

        Returns:
            True if the camera starts successfully, otherwise False.
        """
        try:
            # Create the Picamera2 object that talks to the Raspberry Pi camera.
            self.camera = Picamera2()

            # Configure the camera to return normal preview frames at 640x480.
            frame_duration_us = int(1_000_000 / self.target_fps)
            config = self.camera.create_preview_configuration(
                main={
                    "size": (self.width, self.height),
                    "format": self.pixel_format,
                },
                controls={
                    "FrameDurationLimits": (frame_duration_us, frame_duration_us),
                },
            )
            self.camera.configure(config)
            self._store_diagnostics(config)
            self._print_startup_diagnostics()

            # Start the camera before any frame can be captured.
            self.camera.start()
            self.is_started = True
            return True
        except Exception as error:
            print(f"Camera initialization failed: {error}")
            self.release()
            return False

    def get_frame(self) -> np.ndarray | None:
        """
        Capture one frame from the camera.

        Returns:
            A NumPy image array if capture succeeds, otherwise None.
        """
        if self.camera is None or not self.is_started:
            return None

        try:
            # Picamera2 returns the image as a NumPy array.
            frame = self.camera.capture_array()
            self._record_actual_frame_resolution(frame)
            return frame
        except Exception as error:
            print(f"Frame capture failed: {error}")
            return None

    def release(self) -> None:
        """Stop the camera safely and release camera resources."""
        if self.camera is None:
            self.is_started = False
            return

        try:
            if self.is_started:
                self.camera.stop()
        except Exception as error:
            print(f"Camera stop failed: {error}")
        finally:
            self.camera.close()
            self.camera = None
            self.is_started = False

    def _store_diagnostics(self, config: dict[str, Any]) -> None:
        """
        Store camera configuration values for later reporting.

        Args:
            config: Picamera2 configuration returned by create_preview_configuration.
        """
        if self.camera is None:
            return

        camera_properties = getattr(self.camera, "camera_properties", {}) or {}
        camera_model = str(
            camera_properties.get("Model")
            or camera_properties.get("model")
            or "Unknown"
        )
        main_config = config.get("main", {})
        configured_size = main_config.get("size", (self.width, self.height))
        selected_mode = self._describe_selected_mode(camera_properties)

        self.diagnostics = CameraDiagnostics(
            camera_model=camera_model,
            selected_mode=selected_mode,
            configured_width=int(configured_size[0]),
            configured_height=int(configured_size[1]),
            pixel_format=str(main_config.get("format", self.pixel_format)),
            expected_fps=self.target_fps,
        )

    def _describe_selected_mode(self, camera_properties: dict[str, Any]) -> str:
        """
        Build a beginner-friendly camera mode description.

        Args:
            camera_properties: Picamera2 camera properties.

        Returns:
            Selected mode text for diagnostics.
        """
        pixel_array_size = camera_properties.get("PixelArraySize")
        if pixel_array_size is None:
            return "Preview configuration"

        return f"Preview from sensor array {pixel_array_size}"

    def _record_actual_frame_resolution(self, frame: np.ndarray) -> None:
        """
        Print and store the first captured frame resolution.

        Args:
            frame: Captured camera frame.
        """
        if self._actual_frame_reported:
            return

        self._actual_frame_reported = True
        actual_height, actual_width = frame.shape[:2]

        if self.diagnostics is not None:
            self.diagnostics.actual_width = actual_width
            self.diagnostics.actual_height = actual_height

        if self.print_diagnostics:
            print("Camera actual frame resolution:")
            print(f"Actual Frame Resolution: {actual_width}x{actual_height}")
            print()

    def _print_startup_diagnostics(self) -> None:
        """Print camera startup diagnostics once after configuration."""
        if not self.print_diagnostics or self.diagnostics is None:
            return

        print("Camera startup diagnostics:")
        print(f"Camera Model: {self.diagnostics.camera_model}")
        print(f"Selected Camera Mode: {self.diagnostics.selected_mode}")
        print(
            "Configured Resolution: "
            f"{self.diagnostics.configured_width}x{self.diagnostics.configured_height}"
        )
        print(f"Pixel Format: {self.diagnostics.pixel_format}")
        print(f"Expected Camera FPS: {self.diagnostics.expected_fps:.1f}")
        print()
