from picamera2 import Picamera2
import numpy as np


class CameraManager:
    """Manages direct access to the Raspberry Pi camera."""

    def __init__(self, width: int = 640, height: int = 480) -> None:
        """
        Create a camera manager with the requested image size.

        Args:
            width: Width of the captured image in pixels.
            height: Height of the captured image in pixels.
        """
        self.width = width
        self.height = height
        self.camera: Picamera2 | None = None
        self.is_started = False

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
            config = self.camera.create_preview_configuration(
                main={"size": (self.width, self.height)},
                controls={"FrameDurationLimits": (8333, 8333)}
            )
            self.camera.configure(config)

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
