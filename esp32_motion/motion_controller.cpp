#include "motion_controller.h"


MotionController::MotionController(
  MotorConfig config,
  DifferentialDrive &drive
) : _drive(drive) {
  _config = config;
  _motion_mode = MODE_IDLE;
  _last_phase = MOTION_IDLE;
  _last_diagnostics_ms = 0;
  _profile_start_step_count = 0;
  _continuous_cruise_step = 0;
  _last_profile_frequency_hz = 1.0;
  _continuous_profile_active = false;
}


void MotionController::begin() {
  _drive.begin();
  _drive.setFrequency(1.0);
}


void MotionController::update() {
  if (!_drive.isRunning()) {
    if (_motion_mode == MODE_TURNING_LEFT || _motion_mode == MODE_TURNING_RIGHT) {
      _motion_mode = MODE_IDLE;
      printMotionMode();
    }
    return;
  }

  MotionProfileState state = calculateActiveProfileState();

  if (state.phase == MOTION_COMPLETE) {
    _drive.stop();
    _motion_mode = MODE_IDLE;
    _last_phase = MOTION_IDLE;
    _continuous_profile_active = false;
    _last_profile_frequency_hz = 1.0;
    printDiagnostics(state);
    printMotionMode();
    return;
  }

  applyProfileState(state);
  printDiagnostics(state);
}


void MotionController::moveForward(float distance_mm) {
  _motion_mode = MODE_IDLE;
  _continuous_profile_active = false;
  uint32_t steps = distanceMmToSteps(distance_mm);
  startProfile(steps, _config.max_frequency_hz);
  _drive.moveForward(steps);
}


void MotionController::moveBackward(float distance_mm) {
  _motion_mode = MODE_IDLE;
  _continuous_profile_active = false;
  uint32_t steps = distanceMmToSteps(distance_mm);
  startProfile(steps, _config.max_frequency_hz);
  _drive.moveBackward(steps);
}


void MotionController::turnLeft(float angle_deg) {
  _motion_mode = MODE_TURNING_LEFT;
  _continuous_profile_active = false;
  uint32_t steps = angleDegToTurnSteps(angle_deg);
  startProfile(steps, _config.max_frequency_hz);
  _drive.turnLeft(steps);
}


void MotionController::turnRight(float angle_deg) {
  _motion_mode = MODE_TURNING_RIGHT;
  _continuous_profile_active = false;
  uint32_t steps = angleDegToTurnSteps(angle_deg);
  startProfile(steps, _config.max_frequency_hz);
  _drive.turnRight(steps);
}


void MotionController::stop() {
  if (!_drive.isRunning()) {
    _motion_mode = MODE_IDLE;
    _last_phase = MOTION_IDLE;
    _continuous_profile_active = false;
    _last_profile_frequency_hz = 1.0;
    printMotionMode();
    return;
  }

  startStopProfile();
  _motion_mode = MODE_STOPPING;
  printMotionMode();
}


bool MotionController::isRunning() const {
  return _drive.isRunning();
}


void MotionController::startForwardMode() {
  _motion_mode = MODE_FORWARD;
  startContinuousProfile(_config.max_frequency_hz);
  _drive.moveForward(0);
  printMotionMode();
}


void MotionController::startSlowForwardMode() {
  _motion_mode = MODE_SLOW_FORWARD;
  startContinuousProfile(_config.max_frequency_hz / 3.0);
  _drive.moveForward(0);
  printMotionMode();
}


uint32_t MotionController::distanceMmToSteps(float distance_mm) const {
  float wheel_circumference_mm = PI * _config.wheel_diameter_mm;
  float wheel_revolutions = distance_mm / wheel_circumference_mm;
  float steps = wheel_revolutions * _config.pulses_per_revolution;

  if (steps < 1.0) {
    return 1;
  }

  return (uint32_t)(steps + 0.5);
}


uint32_t MotionController::angleDegToTurnSteps(float angle_deg) const {
  float turn_circumference_mm = PI * _config.wheel_base_mm;
  float wheel_distance_mm = turn_circumference_mm * (angle_deg / 360.0);

  return distanceMmToSteps(wheel_distance_mm);
}


uint32_t MotionController::frequencyToAccelerationSteps(float frequency_hz) const {
  float acceleration_distance =
    (frequency_hz * frequency_hz) /
    (2.0 * _config.max_acceleration_hz_per_sec);

  if (acceleration_distance < 1.0) {
    return 1;
  }

  return (uint32_t)(acceleration_distance + 0.5);
}


void MotionController::startProfile(uint32_t steps, float max_frequency_hz) {
  _profile.configure(
    steps,
    max_frequency_hz,
    _config.max_acceleration_hz_per_sec
  );
  _profile_start_step_count = 0;
  _continuous_cruise_step = 0;
  _continuous_profile_active = false;
  _last_phase = MOTION_IDLE;
  _last_diagnostics_ms = 0;
}


void MotionController::startContinuousProfile(float cruise_frequency_hz) {
  uint32_t acceleration_steps = frequencyToAccelerationSteps(cruise_frequency_hz);
  uint32_t total_steps = (acceleration_steps * 3) + 1;

  startProfile(total_steps, cruise_frequency_hz);
  _continuous_cruise_step = acceleration_steps;
  _continuous_profile_active = true;
}


void MotionController::startStopProfile() {
  float stop_frequency_hz = _last_profile_frequency_hz;
  uint32_t stop_steps = frequencyToAccelerationSteps(stop_frequency_hz);
  uint32_t total_steps = (stop_steps * 2) + 1;

  _profile.configure(
    total_steps,
    stop_frequency_hz,
    _config.max_acceleration_hz_per_sec
  );

  uint32_t deceleration_start_step = stop_steps + 1;
  _profile_start_step_count =
    _drive.getStepCount() > deceleration_start_step
      ? _drive.getStepCount() - deceleration_start_step
      : 0;
  _continuous_cruise_step = 0;
  _continuous_profile_active = false;
  _last_phase = MOTION_IDLE;
  _last_diagnostics_ms = 0;
}


MotionProfileState MotionController::calculateActiveProfileState() const {
  uint32_t profile_step = _drive.getStepCount() - _profile_start_step_count;

  if (_continuous_profile_active && profile_step > _continuous_cruise_step) {
    profile_step = _continuous_cruise_step;
  }

  return _profile.calculate(profile_step);
}


void MotionController::applyProfileState(const MotionProfileState &state) {
  _last_profile_frequency_hz = state.target_frequency_hz;
  _drive.setFrequency(state.target_frequency_hz);
}


void MotionController::printDiagnostics(const MotionProfileState &state) {
  unsigned long now_ms = millis();
  bool phase_changed = state.phase != _last_phase;
  bool diagnostics_due = (now_ms - _last_diagnostics_ms) >= 250;

  if (!phase_changed && !diagnostics_due) {
    return;
  }

  Serial.print("Phase: ");
  Serial.print(motionPhaseToText(state.phase));
  Serial.print(" | Frequency Hz: ");
  Serial.print(state.target_frequency_hz);
  Serial.print(" | Step count: ");
  Serial.print(state.current_step_count);
  Serial.print(" | Remaining steps: ");
  Serial.println(state.remaining_steps);

  _last_phase = state.phase;
  _last_diagnostics_ms = now_ms;
}


void MotionController::printMotionMode() {
  unsigned long now_ms = millis();
  bool diagnostics_due = (now_ms - _last_diagnostics_ms) >= 250;

  if (!diagnostics_due) {
    return;
  }

  Serial.print("Motion Mode: ");
  Serial.println(motionModeToText(_motion_mode));
  _last_diagnostics_ms = now_ms;
}


const char *MotionController::motionModeToText(MotionMode mode) const {
  switch (mode) {
    case MODE_IDLE:
      return "IDLE";
    case MODE_FORWARD:
      return "FORWARD_MODE";
    case MODE_SLOW_FORWARD:
      return "SLOW_FORWARD_MODE";
    case MODE_STOPPING:
      return "STOPPING";
    case MODE_TURNING_LEFT:
      return "TURNING_LEFT";
    case MODE_TURNING_RIGHT:
      return "TURNING_RIGHT";
    default:
      return "UNKNOWN";
  }
}
