#include "motion_controller.h"


MotionController::MotionController(
  MotorConfig config,
  DifferentialDrive &drive
) : _drive(drive) {
  _config = config;
  _last_phase = MOTION_IDLE;
  _last_diagnostics_ms = 0;
  _previous_motion = COMMAND_NONE;
  _active_motion = COMMAND_NONE;
  _pending_motion = COMMAND_NONE;
  _pending_steps = 0;
  _pending_left_forward = true;
  _pending_right_forward = true;
  _has_previous_direction = false;
  _previous_left_forward = true;
  _previous_right_forward = true;
  _settling_active = false;
  _settling_until_ms = 0;
}


void MotionController::begin() {
  _drive.begin();
  _drive.setFrequency(_config.min_start_frequency_hz);
}


void MotionController::update() {
  if (_settling_active) {
    if ((long)(millis() - _settling_until_ms) >= 0) {
      startPendingMotion();
    }
    return;
  }

  if (!_drive.isRunning()) {
    if (_active_motion != COMMAND_NONE && _drive.isComplete()) {
      MotionProfileState state = _profile.calculate(_drive.getStepCount());
      completeActiveMotion();
      printDiagnostics(state);
    }
    return;
  }

  uint32_t current_steps = _drive.getStepCount();
  MotionProfileState state = _profile.calculate(current_steps);

  if (state.phase == MOTION_COMPLETE) {
    completeActiveMotion();
    printDiagnostics(state);
    return;
  }

  _drive.setFrequency(state.target_frequency_hz);
  printDiagnostics(state);
}


void MotionController::moveForward(float distance_mm) {
  uint32_t steps = distanceMmToSteps(distance_mm);
  requestMotion(COMMAND_FORWARD, steps, true, true);
}


void MotionController::moveBackward(float distance_mm) {
  uint32_t steps = distanceMmToSteps(distance_mm);
  requestMotion(COMMAND_BACKWARD, steps, false, false);
}


void MotionController::turnLeft(float angle_deg) {
  uint32_t steps = angleDegToTurnSteps(angle_deg);
  requestMotion(COMMAND_TURN_LEFT, steps, false, true);
}


void MotionController::turnRight(float angle_deg) {
  uint32_t steps = angleDegToTurnSteps(angle_deg);
  requestMotion(COMMAND_TURN_RIGHT, steps, true, false);
}


void MotionController::stop() {
  _drive.stop();
  _last_phase = MOTION_IDLE;
  _settling_active = false;
  _pending_motion = COMMAND_NONE;
  _active_motion = COMMAND_NONE;
}


bool MotionController::isRunning() const {
  return _drive.isRunning() || _settling_active || _pending_motion != COMMAND_NONE;
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


void MotionController::requestMotion(
  MotionCommandType new_motion,
  uint32_t steps,
  bool left_forward,
  bool right_forward
) {
  if (steps == 0) {
    return;
  }

  bool direction_changed =
    _has_previous_direction &&
    ((left_forward != _previous_left_forward) ||
     (right_forward != _previous_right_forward));

  uint32_t settling_ms =
    direction_changed ? _config.direction_change_settling_ms : 0;

  printTransitionDiagnostics(
    _previous_motion,
    new_motion,
    direction_changed,
    settling_ms
  );

  _drive.stop();
  _drive.setFrequency(_config.min_start_frequency_hz);

  _pending_motion = new_motion;
  _pending_steps = steps;
  _pending_left_forward = left_forward;
  _pending_right_forward = right_forward;
  configureProfile(steps);

  if (settling_ms > 0) {
    _settling_active = true;
    _settling_until_ms = millis() + settling_ms;
    return;
  }

  startPendingMotion();
}


void MotionController::startPendingMotion() {
  if (_pending_motion == COMMAND_NONE || _pending_steps == 0) {
    _settling_active = false;
    return;
  }

  _settling_active = false;
  _active_motion = _pending_motion;

  _drive.setFrequency(_config.min_start_frequency_hz);

  switch (_pending_motion) {
    case COMMAND_FORWARD:
      _drive.moveForward(_pending_steps);
      break;
    case COMMAND_BACKWARD:
      _drive.moveBackward(_pending_steps);
      break;
    case COMMAND_TURN_LEFT:
      _drive.turnLeft(_pending_steps);
      break;
    case COMMAND_TURN_RIGHT:
      _drive.turnRight(_pending_steps);
      break;
    default:
      break;
  }

  _previous_left_forward = _pending_left_forward;
  _previous_right_forward = _pending_right_forward;
  _has_previous_direction = true;
  _pending_motion = COMMAND_NONE;
  _pending_steps = 0;
}


void MotionController::completeActiveMotion() {
  _drive.stop();
  _previous_motion = _active_motion;
  _active_motion = COMMAND_NONE;
  _last_phase = MOTION_IDLE;
}


void MotionController::configureProfile(uint32_t steps) {
  _profile.configure(
    steps,
    _config.max_frequency_hz,
    _config.max_acceleration_hz_per_sec,
    _config.min_start_frequency_hz
  );
  _drive.setFrequency(_config.min_start_frequency_hz);
  _last_phase = MOTION_IDLE;
  _last_diagnostics_ms = 0;
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


void MotionController::printTransitionDiagnostics(
  MotionCommandType previous_motion,
  MotionCommandType new_motion,
  bool direction_changed,
  uint32_t settling_ms
) {
  Serial.print("Previous motion: ");
  Serial.print(motionCommandToText(previous_motion));
  Serial.print(" | New motion: ");
  Serial.print(motionCommandToText(new_motion));
  Serial.print(" | Direction changed: ");
  Serial.print(direction_changed ? "YES" : "NO");
  Serial.print(" | Startup frequency Hz: ");
  Serial.print(_config.min_start_frequency_hz);
  Serial.print(" | Settling ms: ");
  Serial.println(settling_ms);
}


const char *MotionController::motionCommandToText(MotionCommandType motion) const {
  switch (motion) {
    case COMMAND_FORWARD:
      return "FORWARD";
    case COMMAND_BACKWARD:
      return "BACKWARD";
    case COMMAND_TURN_LEFT:
      return "TURN_LEFT";
    case COMMAND_TURN_RIGHT:
      return "TURN_RIGHT";
    case COMMAND_NONE:
      return "NONE";
    default:
      return "UNKNOWN";
  }
}
