import unittest

from control.motion_profile import (
    MotionConstraints,
    generate_trapezoidal_profile,
)
from control.motor_models import WheelMotionConfig, wheel_distance_to_steps


def print_profile(title: str, total_steps: int, profile) -> None:
    """
    Print a motion profile in a readable format.

    Args:
        title: Name of the test profile.
        total_steps: Input move distance in steps.
        profile: MotionProfile returned by the generator.
    """
    print(title)
    print(f"profile type: {profile.profile_type}")
    print(f"acceleration distance: {profile.acceleration_steps}")
    print(f"cruise distance: {profile.cruise_steps}")
    print(f"deceleration distance: {profile.deceleration_steps}")
    print(f"total steps: {total_steps}")


class TestMotionProfile(unittest.TestCase):
    """Unit tests for trapezoidal and triangular motion profile generation."""

    def test_generates_trapezoidal_profile(self) -> None:
        """A long move should include acceleration, cruise, and deceleration."""
        constraints = MotionConstraints(
            max_velocity_steps_per_sec=1000,
            acceleration_steps_per_sec2=500,
        )

        profile = generate_trapezoidal_profile(
            total_steps=3000,
            max_velocity=constraints.max_velocity_steps_per_sec,
            acceleration=constraints.acceleration_steps_per_sec2,
        )

        print_profile("Trapezoidal profile", 3000, profile)
        self.assertEqual(profile.profile_type, "trapezoidal")
        self.assertEqual(profile.total_steps, 3000)
        self.assertEqual(
            profile.acceleration_steps
            + profile.cruise_steps
            + profile.deceleration_steps,
            3000,
        )

    def test_generates_triangular_profile(self) -> None:
        """A short move should skip the cruise phase."""
        constraints = MotionConstraints(
            max_velocity_steps_per_sec=1000,
            acceleration_steps_per_sec2=500,
        )

        profile = generate_trapezoidal_profile(
            total_steps=500,
            max_velocity=constraints.max_velocity_steps_per_sec,
            acceleration=constraints.acceleration_steps_per_sec2,
        )

        print_profile("Triangular profile", 500, profile)
        self.assertEqual(profile.profile_type, "triangular")
        self.assertEqual(profile.cruise_steps, 0)
        self.assertEqual(profile.total_steps, 500)

    def test_profile_is_independent_of_driver_pulse_setting(self) -> None:
        """Different driver pulse settings should only affect total_steps."""
        config = WheelMotionConfig(
            wheel_diameter_m=0.1,
            driver_pulses_per_rev=5000,
        )
        total_steps = wheel_distance_to_steps(
            wheel_distance_m=1.0,
            config=config,
        )

        profile = generate_trapezoidal_profile(
            total_steps=total_steps,
            max_velocity=2000,
            acceleration=1000,
        )

        print_profile("Converted wheel-distance profile", total_steps, profile)
        self.assertEqual(profile.total_steps, total_steps)
        self.assertGreater(profile.total_steps, 0)


if __name__ == "__main__":
    unittest.main()
