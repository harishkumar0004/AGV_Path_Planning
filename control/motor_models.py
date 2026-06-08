from dataclasses import dataclass
from math import pi


@dataclass
class WheelMotionConfig:
    """Configuration used to convert wheel travel distance into driver steps."""

    # Wheel diameter in meters.
    wheel_diameter_m: float

    # Driver pulse setting, such as 1000, 2000, 5000, or 40000 pulses per rev.
    driver_pulses_per_rev: int

    # Motor revolutions per wheel revolution. Use 1.0 for direct drive.
    gear_ratio: float = 1.0


def wheel_distance_to_steps(
    wheel_distance_m: float,
    config: WheelMotionConfig,
) -> int:
    """
    Convert wheel travel distance into driver steps.

    Args:
        wheel_distance_m: Desired wheel travel distance in meters.
        config: Wheel and driver conversion settings.

    Returns:
        Total driver steps needed for the requested wheel distance.
    """
    _validate_config(wheel_distance_m, config)

    wheel_circumference_m = pi * config.wheel_diameter_m
    wheel_revolutions = wheel_distance_m / wheel_circumference_m
    motor_revolutions = wheel_revolutions * config.gear_ratio
    total_steps = motor_revolutions * config.driver_pulses_per_rev

    return int(round(total_steps))


def _validate_config(wheel_distance_m: float, config: WheelMotionConfig) -> None:
    """
    Validate distance-to-steps conversion inputs.

    Args:
        wheel_distance_m: Desired wheel travel distance in meters.
        config: Wheel and driver conversion settings.
    """
    if wheel_distance_m < 0:
        raise ValueError("wheel_distance_m must not be negative.")

    if config.wheel_diameter_m <= 0:
        raise ValueError("wheel_diameter_m must be greater than zero.")

    if config.driver_pulses_per_rev <= 0:
        raise ValueError("driver_pulses_per_rev must be greater than zero.")

    if config.gear_ratio <= 0:
        raise ValueError("gear_ratio must be greater than zero.")
