from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

DEPENDENCY_ERROR: ModuleNotFoundError | None = None

try:
    import cv2
    import numpy as np
    from smbus2 import SMBus
except ModuleNotFoundError as error:
    DEPENDENCY_ERROR = error


@dataclass
class ImuSample:
    """One raw IMU sample converted into physical units."""

    accel_g: tuple[float, float, float]
    gyro_rad_s: tuple[float, float, float]


@dataclass
class Orientation:
    """Filtered orientation angles in degrees."""

    roll: float
    pitch: float
    yaw: float


class HW290ImuReader:
    """
    Reads accelerometer and gyroscope data from an HW-290/MPU6050-style IMU.

    The common HW-290 module is typically used over I2C at address 0x68. This
    validation reader keeps the hardware access isolated from navigation code.
    """

    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C

    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0

    def __init__(self, bus_number: int = 1, address: int = 0x68) -> None:
        """
        Create an IMU reader.

        Args:
            bus_number: Raspberry Pi I2C bus number.
            address: I2C address of the IMU.
        """
        self.bus_number = bus_number
        self.address = address
        self.bus: SMBus | None = None

    def initialize(self) -> bool:
        """
        Open I2C and wake the IMU.

        Returns:
            True when the IMU initializes successfully.
        """
        try:
            self.bus = SMBus(self.bus_number)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
            time.sleep(0.1)
            return True
        except OSError as error:
            print(f"IMU initialization failed: {error}")
            self.release()
            return False

    def read_sample(self) -> ImuSample | None:
        """
        Read one accelerometer/gyroscope sample.

        Returns:
            ImuSample when a read succeeds, otherwise None.
        """
        if self.bus is None:
            return None

        try:
            data = self.bus.read_i2c_block_data(
                self.address,
                self.ACCEL_XOUT_H,
                14,
            )
        except OSError as error:
            print(f"IMU read failed: {error}")
            return None

        accel_x = self._combine_bytes(data[0], data[1]) / self.ACCEL_SCALE
        accel_y = self._combine_bytes(data[2], data[3]) / self.ACCEL_SCALE
        accel_z = self._combine_bytes(data[4], data[5]) / self.ACCEL_SCALE

        gyro_x_deg_s = self._combine_bytes(data[8], data[9]) / self.GYRO_SCALE
        gyro_y_deg_s = self._combine_bytes(data[10], data[11]) / self.GYRO_SCALE
        gyro_z_deg_s = self._combine_bytes(data[12], data[13]) / self.GYRO_SCALE

        return ImuSample(
            accel_g=(accel_x, accel_y, accel_z),
            gyro_rad_s=(
                math.radians(gyro_x_deg_s),
                math.radians(gyro_y_deg_s),
                math.radians(gyro_z_deg_s),
            ),
        )

    def release(self) -> None:
        """Close the I2C bus."""
        if self.bus is not None:
            self.bus.close()
            self.bus = None

    @staticmethod
    def _combine_bytes(high_byte: int, low_byte: int) -> int:
        """
        Convert two MPU register bytes into a signed 16-bit integer.

        Args:
            high_byte: High register byte.
            low_byte: Low register byte.

        Returns:
            Signed integer sensor reading.
        """
        value = (high_byte << 8) | low_byte
        if value >= 0x8000:
            value -= 0x10000
        return value


class ValidationMadgwickFilter:
    """
    Minimal IMU-only Madgwick filter for validation.

    This stays inside the validation layer because the production architecture
    does not yet own an IMU subsystem.
    """

    def __init__(self, beta: float = 0.1) -> None:
        """
        Create the filter.

        Args:
            beta: Filter gain. Higher values trust accelerometer correction more.
        """
        self.beta = beta
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

    def update(
        self,
        gyro_rad_s: tuple[float, float, float],
        accel_g: tuple[float, float, float],
        delta_time: float,
    ) -> Orientation:
        """
        Update orientation from one gyro/accelerometer sample.

        Args:
            gyro_rad_s: Angular velocity in radians/second.
            accel_g: Acceleration in g.
            delta_time: Seconds since previous sample.

        Returns:
            Current roll, pitch, and yaw in degrees.
        """
        gx, gy, gz = gyro_rad_s
        ax, ay, az = accel_g

        accel_norm = math.sqrt((ax * ax) + (ay * ay) + (az * az))
        if accel_norm == 0.0:
            return self.to_euler_degrees()

        ax /= accel_norm
        ay /= accel_norm
        az /= accel_norm

        q0 = self.q0
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3

        f1 = 2.0 * (q1 * q3 - q0 * q2) - ax
        f2 = 2.0 * (q0 * q1 + q2 * q3) - ay
        f3 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - az

        j_11_or_24 = 2.0 * q2
        j_12_or_23 = 2.0 * q3
        j_13_or_22 = 2.0 * q0
        j_14_or_21 = 2.0 * q1
        j_32 = 4.0 * q1
        j_33 = 4.0 * q2

        gradient_0 = (
            j_14_or_21 * f2
            - j_11_or_24 * f1
        )
        gradient_1 = (
            j_12_or_23 * f1
            + j_13_or_22 * f2
            - j_32 * f3
        )
        gradient_2 = (
            j_12_or_23 * f2
            - j_33 * f3
            - j_13_or_22 * f1
        )
        gradient_3 = (
            j_14_or_21 * f1
            + j_11_or_24 * f2
        )

        gradient_norm = math.sqrt(
            gradient_0 * gradient_0
            + gradient_1 * gradient_1
            + gradient_2 * gradient_2
            + gradient_3 * gradient_3
        )

        if gradient_norm > 0.0:
            gradient_0 /= gradient_norm
            gradient_1 /= gradient_norm
            gradient_2 /= gradient_norm
            gradient_3 /= gradient_norm

        q_dot_0 = (
            0.5 * (-q1 * gx - q2 * gy - q3 * gz)
            - self.beta * gradient_0
        )
        q_dot_1 = (
            0.5 * (q0 * gx + q2 * gz - q3 * gy)
            - self.beta * gradient_1
        )
        q_dot_2 = (
            0.5 * (q0 * gy - q1 * gz + q3 * gx)
            - self.beta * gradient_2
        )
        q_dot_3 = (
            0.5 * (q0 * gz + q1 * gy - q2 * gx)
            - self.beta * gradient_3
        )

        q0 += q_dot_0 * delta_time
        q1 += q_dot_1 * delta_time
        q2 += q_dot_2 * delta_time
        q3 += q_dot_3 * delta_time

        quaternion_norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if quaternion_norm > 0.0:
            self.q0 = q0 / quaternion_norm
            self.q1 = q1 / quaternion_norm
            self.q2 = q2 / quaternion_norm
            self.q3 = q3 / quaternion_norm

        return self.to_euler_degrees()

    def to_euler_degrees(self) -> Orientation:
        """
        Convert the current quaternion to roll, pitch, and yaw.

        Returns:
            Orientation in degrees.
        """
        q0 = self.q0
        q1 = self.q1
        q2 = self.q2
        q3 = self.q3

        roll = math.atan2(
            2.0 * (q0 * q1 + q2 * q3),
            1.0 - 2.0 * (q1 * q1 + q2 * q2),
        )
        pitch_value = 2.0 * (q0 * q2 - q3 * q1)
        pitch_value = max(-1.0, min(1.0, pitch_value))
        pitch = math.asin(pitch_value)
        yaw = math.atan2(
            2.0 * (q0 * q3 + q1 * q2),
            1.0 - 2.0 * (q2 * q2 + q3 * q3),
        )

        return Orientation(
            roll=math.degrees(roll),
            pitch=math.degrees(pitch),
            yaw=math.degrees(yaw),
        )


class ExternalMadgwickFilter:
    """
    Adapter for the common `ahrs.filters.Madgwick` implementation.

    If the Raspberry Pi environment already has this Madgwick implementation,
    the validation script can use it without changing the rest of the code.
    """

    def __init__(self, beta: float = 0.1) -> None:
        """
        Create the external Madgwick adapter.

        Args:
            beta: Filter gain.
        """
        from ahrs.filters import Madgwick

        self.filter = Madgwick(gain=beta)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    def update(
        self,
        gyro_rad_s: tuple[float, float, float],
        accel_g: tuple[float, float, float],
        delta_time: float,
    ) -> Orientation:
        """
        Update orientation using the external Madgwick implementation.

        Args:
            gyro_rad_s: Angular velocity in radians/second.
            accel_g: Acceleration in g.
            delta_time: Seconds since previous sample.

        Returns:
            Current roll, pitch, and yaw in degrees.
        """
        self.quaternion = self.filter.updateIMU(
            self.quaternion,
            gyr=np.array(gyro_rad_s, dtype=float),
            acc=np.array(accel_g, dtype=float),
            dt=delta_time,
        )
        return quaternion_to_euler_degrees(self.quaternion)


def quaternion_to_euler_degrees(quaternion) -> Orientation:
    """
    Convert a quaternion into roll, pitch, and yaw.

    Args:
        quaternion: Quaternion in w, x, y, z order.

    Returns:
        Orientation in degrees.
    """
    q0, q1, q2, q3 = quaternion

    roll = math.atan2(
        2.0 * (q0 * q1 + q2 * q3),
        1.0 - 2.0 * (q1 * q1 + q2 * q2),
    )
    pitch_value = 2.0 * (q0 * q2 - q3 * q1)
    pitch_value = max(-1.0, min(1.0, pitch_value))
    pitch = math.asin(pitch_value)
    yaw = math.atan2(
        2.0 * (q0 * q3 + q1 * q2),
        1.0 - 2.0 * (q2 * q2 + q3 * q3),
    )

    return Orientation(
        roll=math.degrees(roll),
        pitch=math.degrees(pitch),
        yaw=math.degrees(yaw),
    )


def create_madgwick_filter(beta: float):
    """
    Create the best available Madgwick filter for validation.

    Args:
        beta: Filter gain.

    Returns:
        External Madgwick filter when available, otherwise validation fallback.
    """
    try:
        madgwick = ExternalMadgwickFilter(beta=beta)
        print("Using external ahrs.filters.Madgwick implementation.")
        return madgwick
    except ModuleNotFoundError:
        print("Using validation-only Madgwick fallback implementation.")
        return ValidationMadgwickFilter(beta=beta)


class UpdateRateMonitor:
    """Measures the IMU update rate over a rolling one-second window."""

    def __init__(self, window_sec: float = 1.0) -> None:
        """
        Create an update-rate monitor.

        Args:
            window_sec: Time window used to calculate rate.
        """
        self.window_sec = window_sec
        self.window_start = time.monotonic()
        self.sample_count = 0
        self.rate_hz = 0.0

    def record_sample(self) -> float:
        """
        Record one sample and return the latest rate.

        Returns:
            Current update rate in Hz.
        """
        self.sample_count += 1
        now = time.monotonic()
        elapsed = now - self.window_start

        if elapsed >= self.window_sec:
            self.rate_hz = self.sample_count / elapsed
            self.sample_count = 0
            self.window_start = now

        return self.rate_hz


def draw_overlay(
    frame,
    orientation: Orientation,
    update_rate_hz: float,
) -> None:
    """
    Draw filtered IMU orientation on an OpenCV window.

    Args:
        frame: Image used as the display canvas.
        orientation: Current roll, pitch, and yaw.
        update_rate_hz: Current IMU update rate.
    """
    lines = [
        f"Roll: {orientation.roll:.2f} deg",
        f"Pitch: {orientation.pitch:.2f} deg",
        f"Yaw: {orientation.yaw:.2f} deg",
        f"IMU Rate: {update_rate_hz:.1f} Hz",
        "Rotate AGV manually and observe which value changes.",
        "Q: Quit",
    ]

    y = 45
    for line in lines:
        cv2.putText(
            frame,
            line,
            (30, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        y += 40


def log_orientation(
    start_time: float,
    orientation: Orientation,
    update_rate_hz: float,
) -> None:
    """
    Print timestamped filtered orientation values.

    Args:
        start_time: Monotonic start time.
        orientation: Current roll, pitch, and yaw.
        update_rate_hz: Current IMU update rate.
    """
    elapsed = time.monotonic() - start_time
    print(
        f"Time: {elapsed:.2f}s | "
        f"Roll: {orientation.roll:.2f} deg | "
        f"Pitch: {orientation.pitch:.2f} deg | "
        f"Yaw: {orientation.yaw:.2f} deg | "
        f"IMU Rate: {update_rate_hz:.1f} Hz"
    )


def run_validation(args: argparse.Namespace) -> None:
    """Run IMU acquisition and orientation display validation."""
    imu = HW290ImuReader(bus_number=args.bus, address=args.address)
    madgwick = create_madgwick_filter(beta=args.beta)
    rate_monitor = UpdateRateMonitor()
    start_time = time.monotonic()
    previous_update_time = start_time
    last_log_time = 0.0

    if not imu.initialize():
        return

    print("IMU validation started.")
    print("Rotate the AGV manually and observe roll, pitch, and yaw.")
    print("Press Q in the OpenCV window to quit.")

    try:
        while True:
            sample = imu.read_sample()
            now = time.monotonic()
            delta_time = now - previous_update_time
            previous_update_time = now

            if sample is None:
                time.sleep(0.01)
                continue

            if delta_time <= 0.0:
                delta_time = 1.0 / 100.0

            orientation = madgwick.update(
                gyro_rad_s=sample.gyro_rad_s,
                accel_g=sample.accel_g,
                delta_time=delta_time,
            )
            update_rate_hz = rate_monitor.record_sample()

            if (now - last_log_time) >= args.log_interval:
                log_orientation(start_time, orientation, update_rate_hz)
                last_log_time = now

            frame = np.zeros((360, 640, 3), dtype=np.uint8)
            draw_overlay(frame, orientation, update_rate_hz)
            cv2.imshow("IMU Validation", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break

            if args.sample_delay > 0.0:
                time.sleep(args.sample_delay)
    except KeyboardInterrupt:
        print("Stopping IMU validation.")
    finally:
        imu.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    """Parse IMU validation command-line options."""
    parser = argparse.ArgumentParser(
        description="Validate HW-290 IMU acquisition with a Madgwick filter.",
    )
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=0x68,
        help="I2C address, for example 0x68 or 0x69.",
    )
    parser.add_argument(
        "--beta",
        type=float,
        default=0.1,
        help="Madgwick filter gain.",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=0.25,
        help="Seconds between terminal log lines.",
    )
    parser.add_argument(
        "--sample-delay",
        type=float,
        default=0.0,
        help="Optional delay between IMU reads.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    if any(argument in ("-h", "--help") for argument in sys.argv[1:]):
        parse_args()
        raise SystemExit(0)

    if DEPENDENCY_ERROR is not None:
        print(
            "IMU validation cannot start because a required dependency is "
            f"missing: {DEPENDENCY_ERROR.name}"
        )
        print("Install the Raspberry Pi dependencies, for example:")
        print("  pip install smbus2 opencv-python numpy")
        raise SystemExit(1)

    run_validation(parse_args())
