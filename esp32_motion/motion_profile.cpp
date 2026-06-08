#include "motion_profile.h"


MotionProfile::MotionProfile() {
  _total_steps = 0;
  _acceleration_steps = 0;
  _cruise_steps = 0;
  _deceleration_steps = 0;
  _max_frequency_hz = 1.0;
  _acceleration_hz_per_sec = 1.0;
  _min_start_frequency_hz = 1.0;
  _triangular = false;
}


void MotionProfile::configure(
  uint32_t total_steps,
  float max_frequency_hz,
  float acceleration_hz_per_sec,
  float min_start_frequency_hz
) {
  _total_steps = total_steps;
  _max_frequency_hz = max_frequency_hz;
  _acceleration_hz_per_sec = acceleration_hz_per_sec;
  _min_start_frequency_hz = min_start_frequency_hz;

  if (_min_start_frequency_hz < 1.0) {
    _min_start_frequency_hz = 1.0;
  }

  if (_min_start_frequency_hz > _max_frequency_hz) {
    _min_start_frequency_hz = _max_frequency_hz;
  }

  float acceleration_distance =
    ((_max_frequency_hz * _max_frequency_hz) -
     (_min_start_frequency_hz * _min_start_frequency_hz)) /
    (2.0 * _acceleration_hz_per_sec);

  _acceleration_steps = (uint32_t)(acceleration_distance + 0.5);
  _deceleration_steps = _acceleration_steps;

  if ((_acceleration_steps + _deceleration_steps) >= _total_steps) {
    _triangular = true;
    _acceleration_steps = _total_steps / 2;
    _deceleration_steps = _total_steps - _acceleration_steps;
    _cruise_steps = 0;

    float peak_frequency =
      sqrt(
        (_min_start_frequency_hz * _min_start_frequency_hz) +
        (2.0 * _acceleration_hz_per_sec * _acceleration_steps)
      );
    if (peak_frequency >= 1.0) {
      _max_frequency_hz = peak_frequency;
    }
    return;
  }

  _triangular = false;
  _cruise_steps =
    _total_steps - _acceleration_steps - _deceleration_steps;
}


MotionProfileState MotionProfile::calculate(uint32_t current_step_count) const {
  MotionProfileState state;
  state.current_step_count = current_step_count;
  state.remaining_steps =
    current_step_count >= _total_steps ? 0 : _total_steps - current_step_count;

  if (_total_steps == 0) {
    state.phase = MOTION_IDLE;
    state.target_frequency_hz = 0.0;
    return state;
  }

  if (current_step_count >= _total_steps) {
    state.phase = MOTION_COMPLETE;
    state.target_frequency_hz = 0.0;
    return state;
  }

  if (current_step_count < _acceleration_steps) {
    state.phase = MOTION_ACCELERATION;
    state.target_frequency_hz =
      calculateFrequencyForStep(current_step_count + 1);
    return state;
  }

  if (current_step_count >= (_total_steps - _deceleration_steps)) {
    state.phase = MOTION_DECELERATION;
    state.target_frequency_hz = calculateFrequencyForStep(state.remaining_steps);
    return state;
  }

  state.phase = MOTION_CRUISE;
  state.target_frequency_hz = _max_frequency_hz;
  return state;
}


bool MotionProfile::isTriangular() const {
  return _triangular;
}


uint32_t MotionProfile::getTotalSteps() const {
  return _total_steps;
}


float MotionProfile::calculateFrequencyForStep(uint32_t phase_step) const {
  float frequency = sqrt(
    (_min_start_frequency_hz * _min_start_frequency_hz) +
    (2.0 * _acceleration_hz_per_sec * (phase_step - 1))
  );

  if (frequency < _min_start_frequency_hz) {
    return _min_start_frequency_hz;
  }

  if (frequency > _max_frequency_hz) {
    return _max_frequency_hz;
  }

  return frequency;
}
