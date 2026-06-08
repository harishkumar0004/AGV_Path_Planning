from dataclasses import dataclass


@dataclass
class MotionConstraints:
    """Velocity and acceleration limits for stepper motion planning."""

    # Maximum allowed velocity in driver steps per second.
    max_velocity_steps_per_sec: float

    # Acceleration limit in driver steps per second squared.
    acceleration_steps_per_sec2: float


@dataclass
class MotionProfile:
    """Step distances for each phase of a trapezoidal or triangular move."""

    # Number of steps used while accelerating.
    acceleration_steps: int

    # Number of steps used at constant velocity.
    cruise_steps: int

    # Number of steps used while decelerating.
    deceleration_steps: int

    # Total number of steps in the complete move.
    total_steps: int

    @property
    def profile_type(self) -> str:
        """Return whether the move is trapezoidal or triangular."""
        if self.cruise_steps > 0:
            return "trapezoidal"

        return "triangular"


def generate_trapezoidal_profile(
    total_steps: int,
    max_velocity: float,
    acceleration: float,
) -> MotionProfile:
    """
    Generate a trapezoidal or triangular motion profile in step units.

    Args:
        total_steps: Total move distance in driver steps.
        max_velocity: Maximum velocity in steps per second.
        acceleration: Acceleration in steps per second squared.

    Returns:
        MotionProfile containing acceleration, cruise, and deceleration steps.
    """
    _validate_inputs(total_steps, max_velocity, acceleration)

    # Distance needed to accelerate from 0 to max velocity:
    # v^2 = 2 * a * d  ->  d = v^2 / (2 * a)
    acceleration_distance = (max_velocity * max_velocity) / (2 * acceleration)

    # If acceleration and deceleration cannot both fit, the move is triangular.
    if 2 * acceleration_distance >= total_steps:
        acceleration_steps = total_steps // 2
        deceleration_steps = total_steps - acceleration_steps
        return MotionProfile(
            acceleration_steps=acceleration_steps,
            cruise_steps=0,
            deceleration_steps=deceleration_steps,
            total_steps=total_steps,
        )

    acceleration_steps = int(round(acceleration_distance))
    deceleration_steps = acceleration_steps
    cruise_steps = total_steps - acceleration_steps - deceleration_steps

    return MotionProfile(
        acceleration_steps=acceleration_steps,
        cruise_steps=cruise_steps,
        deceleration_steps=deceleration_steps,
        total_steps=total_steps,
    )


def _validate_inputs(
    total_steps: int,
    max_velocity: float,
    acceleration: float,
) -> None:
    """
    Validate motion profile inputs.

    Args:
        total_steps: Total move distance in driver steps.
        max_velocity: Maximum velocity in steps per second.
        acceleration: Acceleration in steps per second squared.
    """
    if total_steps <= 0:
        raise ValueError("total_steps must be greater than zero.")

    if max_velocity <= 0:
        raise ValueError("max_velocity must be greater than zero.")

    if acceleration <= 0:
        raise ValueError("acceleration must be greater than zero.")
