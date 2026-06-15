#include "serial_command_handler.h"


const float DEFAULT_TURN_LEFT_VALUE = 10.0;
const float DEFAULT_TURN_RIGHT_VALUE = 10.0;
const float VALIDATION_STEERING_FREQUENCY_HZ = 2000.0;
const uint32_t DEFAULT_STEERING_PULSE_MS = 100;
const float MIN_SET_DRIVE_FREQUENCY_HZ = 1000.0;
const float MAX_SET_DRIVE_FREQUENCY_HZ = 10000.0;


SerialCommandHandler::SerialCommandHandler(
  HardwareSerial &serial,
  MotionController &motion_controller,
  uint32_t command_timeout_ms,
  DifferentialDrive *validation_drive,
  ImuManager *imu_manager,
  float direct_drive_acceleration_hz_per_sec
) : _serial(serial), _motion_controller(motion_controller) {
  _validation_drive = validation_drive;
  _imu_manager = imu_manager;
  _command_timeout_ms = command_timeout_ms;
  _last_command_ms = 0;
  _has_received_command = false;
  _command_buffer = "";
  _validation_pulse_active = false;
  _steering_correction_active = false;
  _forward_mode_active = false;
  _direct_drive_active = false;
  _direct_drive_stop_requested = false;
  _validation_pulse_end_ms = 0;
  _last_drive_update_ms = 0;
  _last_direct_drive_diagnostics_ms = 0;
  _direct_drive_acceleration_hz_per_sec = direct_drive_acceleration_hz_per_sec;
  _target_left_frequency_hz = 0.0;
  _target_right_frequency_hz = 0.0;
  _current_left_frequency_hz = 0.0;
  _current_right_frequency_hz = 0.0;
}


void SerialCommandHandler::begin(uint32_t baudrate) {
  _serial.begin(baudrate);
  _serial.println("ESP32 serial command handler ready");
}


void SerialCommandHandler::update() {
  updateValidationSteeringPulse();
  updateDirectDriveRamp();

  while (_serial.available() > 0) {
    char incoming_char = (char)_serial.read();

    if (incoming_char == '\n') {
      processCommand(_command_buffer);
      _command_buffer = "";
    } else if (incoming_char != '\r') {
      _command_buffer += incoming_char;
    }
  }

  checkSerialSafety();
}


bool SerialCommandHandler::isValidationPulseActive() const {
  return _validation_pulse_active;
}


bool SerialCommandHandler::isDirectDriveActive() const {
  return _direct_drive_active;
}


void SerialCommandHandler::processCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) {
    return;
  }

  String command_name = command;
  int separator_index = command.indexOf(' ');
  if (separator_index > 0) {
    command_name = command.substring(0, separator_index);
  }

  _last_command_ms = millis();
  _has_received_command = true;

  _serial.print("RX: ");
  _serial.println(command);
  printCommandState();

  if (command_name == "STATUS") {
    _serial.print("STATUS: ");
    _serial.println(
      (
        _motion_controller.isRunning() ||
        _validation_pulse_active ||
        _steering_correction_active ||
        _direct_drive_active
      )
        ? "RUNNING"
        : "IDLE"
    );
    return;
  }

  if (command_name == "CALIBRATE_IMU") {
    if (_imu_manager == nullptr) {
      _serial.println("IMU_ERROR");
      _serial.println("Warning: IMU manager is not configured.");
      return;
    }

    _serial.println("IMU_CALIBRATING");

    if (!_imu_manager->recalibrate()) {
      _serial.println("IMU_ERROR");
      return;
    }

    _serial.println("IMU_READY");
    _imu_manager->printHeadingSerial(_serial);
    return;
  }

  if (command_name == "LEFT_PULSE") {
    uint32_t duration_ms =
      (uint32_t)getCommandValue(command, DEFAULT_STEERING_PULSE_MS);
    _serial.print("Validation Pulse: LEFT ");
    _serial.print(duration_ms);
    _serial.println(" ms");
    startValidationSteeringPulse(true, duration_ms);
    return;
  }

  if (command_name == "RIGHT_PULSE") {
    uint32_t duration_ms =
      (uint32_t)getCommandValue(command, DEFAULT_STEERING_PULSE_MS);
    _serial.print("Validation Pulse: RIGHT ");
    _serial.print(duration_ms);
    _serial.println(" ms");
    startValidationSteeringPulse(false, duration_ms);
    return;
  }

  if (command_name == "START_LEFT_CORRECTION") {
    _serial.println("Validation Correction: START_LEFT_CORRECTION");
    startValidationSteeringCorrection(true);
    _serial.println("STEERING_LEFT_ACTIVE");
    printCommandState();
    return;
  }

  if (command_name == "START_RIGHT_CORRECTION") {
    _serial.println("Validation Correction: START_RIGHT_CORRECTION");
    startValidationSteeringCorrection(false);
    _serial.println("STEERING_RIGHT_ACTIVE");
    printCommandState();
    return;
  }

  if (command_name == "STOP_CORRECTION") {
    _serial.println("Validation Correction: STOP_CORRECTION");
    stopValidationSteeringCorrection();
    _serial.println("STEERING_CORRECTION_STOPPED");
    _serial.print("FORWARD_MODE_ACTIVE = ");
    _serial.println(_forward_mode_active ? "TRUE" : "FALSE");
    printCommandState();
    return;
  }

  if (command_name == "START_FORWARD") {
    _serial.println("Motion Mode: FORWARD_MODE");
    _direct_drive_active = false;
    _motion_controller.startForwardMode();
    _forward_mode_active = true;
    _serial.println("FORWARD_MODE_ACTIVE");
    printCommandState();
    return;
  }

  if (command_name == "START_SLOW_FORWARD") {
    _serial.println("Motion Mode: SLOW_FORWARD_MODE");
    _direct_drive_active = false;
    _motion_controller.startSlowForwardMode();
    _forward_mode_active = true;
    _serial.println("FORWARD_MODE_ACTIVE");
    printCommandState();
    return;
  }

  if (command_name == "SET_DRIVE") {
    if (_validation_drive == nullptr) {
      _serial.println("Warning: validation drive is not configured.");
      return;
    }

    float left_frequency_hz = 0.0;
    float right_frequency_hz = 0.0;

    if (!parseSetDriveCommand(
      command,
      left_frequency_hz,
      right_frequency_hz
    )) {
      _serial.println("Warning: SET_DRIVE requires two positive frequencies.");
      return;
    }

    left_frequency_hz = clampDriveFrequency(left_frequency_hz);
    right_frequency_hz = clampDriveFrequency(right_frequency_hz);

    setDirectDriveTarget(left_frequency_hz, right_frequency_hz);

    _serial.println("SET_DRIVE");
    _serial.print("Target Left: ");
    _serial.println(left_frequency_hz);
    _serial.print("Target Right: ");
    _serial.println(right_frequency_hz);
    _serial.print("Current Left: ");
    _serial.println(_current_left_frequency_hz);
    _serial.print("Current Right: ");
    _serial.println(_current_right_frequency_hz);
    printCommandState();
    return;
  }

  if (command_name == "STOP") {
    _serial.println("Motion Mode: STOPPING");
    if (_direct_drive_active) {
      _serial.println("Direct Drive: ramping to zero");
      requestDirectDriveStop();
      printCommandState();
      return;
    }

    _serial.println("Executing stop()");
    if (_validation_drive != nullptr) {
      stopValidationSteeringCorrection();
    }
    _motion_controller.stop();
    _forward_mode_active = false;
    _steering_correction_active = false;
    _validation_pulse_active = false;
    _direct_drive_active = false;
    _direct_drive_stop_requested = false;
    _target_left_frequency_hz = 0.0;
    _target_right_frequency_hz = 0.0;
    _current_left_frequency_hz = 0.0;
    _current_right_frequency_hz = 0.0;
    _validation_pulse_end_ms = 0;
    _serial.println("ALL_MOTION_STOPPED");
    printCommandState();
    return;
  }

  if (command_name == "TURN_LEFT") {
    float turn_value = getCommandValue(command, DEFAULT_TURN_LEFT_VALUE);
    _serial.println("Motion Mode: TURNING_LEFT");
    _serial.print("Executing turnLeft(");
    _serial.print(turn_value);
    _serial.println(")");
    _motion_controller.turnLeft(turn_value);
    return;
  }

  if (command_name == "LEFT") {
    float turn_value = getCommandValue(command, DEFAULT_TURN_LEFT_VALUE);
    _serial.print("Executing turnLeft(");
    _serial.print(turn_value);
    _serial.println(")");
    _motion_controller.turnLeft(turn_value);
    return;
  }

  if (command_name == "RIGHT") {
    float turn_value = getCommandValue(command, DEFAULT_TURN_RIGHT_VALUE);
    _serial.print("Executing turnRight(");
    _serial.print(turn_value);
    _serial.println(")");
    _motion_controller.turnRight(turn_value);
    return;
  }

  if (command_name == "TURN_RIGHT") {
    float turn_value = getCommandValue(command, DEFAULT_TURN_RIGHT_VALUE);
    _serial.println("Motion Mode: TURNING_RIGHT");
    _serial.print("Executing turnRight(");
    _serial.print(turn_value);
    _serial.println(")");
    _motion_controller.turnRight(turn_value);
    return;
  }

  _serial.print("Warning: unknown command ignored: ");
  _serial.println(command);
}


float SerialCommandHandler::getCommandValue(
  String command,
  float default_value
) const {
  int separator_index = command.indexOf(' ');

  if (separator_index < 0 || separator_index >= command.length() - 1) {
    return default_value;
  }

  String value_text = command.substring(separator_index + 1);
  value_text.trim();

  float parsed_value = value_text.toFloat();
  if (parsed_value <= 0.0) {
    return default_value;
  }

  return parsed_value;
}


bool SerialCommandHandler::parseSetDriveCommand(
  String command,
  float &left_frequency_hz,
  float &right_frequency_hz
) const {
  int first_separator_index = command.indexOf(' ');

  if (
    first_separator_index < 0 ||
    first_separator_index >= command.length() - 1
  ) {
    return false;
  }

  int second_separator_index =
    command.indexOf(' ', first_separator_index + 1);

  if (
    second_separator_index < 0 ||
    second_separator_index >= command.length() - 1
  ) {
    return false;
  }

  String left_text =
    command.substring(first_separator_index + 1, second_separator_index);
  String right_text = command.substring(second_separator_index + 1);

  left_text.trim();
  right_text.trim();

  left_frequency_hz = left_text.toFloat();
  right_frequency_hz = right_text.toFloat();

  return left_frequency_hz > 0.0 && right_frequency_hz > 0.0;
}


float SerialCommandHandler::clampDriveFrequency(float frequency_hz) const {
  if (frequency_hz < MIN_SET_DRIVE_FREQUENCY_HZ) {
    return MIN_SET_DRIVE_FREQUENCY_HZ;
  }

  if (frequency_hz > MAX_SET_DRIVE_FREQUENCY_HZ) {
    return MAX_SET_DRIVE_FREQUENCY_HZ;
  }

  return frequency_hz;
}


void SerialCommandHandler::printCommandState() const {
  _serial.print("Forward Active: ");
  _serial.println(_forward_mode_active ? "TRUE" : "FALSE");
  _serial.print("Steering Active: ");
  _serial.println(_steering_correction_active ? "TRUE" : "FALSE");
  _serial.print("Direct Drive Active: ");
  _serial.println(_direct_drive_active ? "TRUE" : "FALSE");
}


void SerialCommandHandler::checkSerialSafety() {
  if (!_has_received_command || _command_timeout_ms == 0) {
    return;
  }

  if (!_motion_controller.isRunning() && !_direct_drive_active) {
    return;
  }

  if ((long)(millis() - (_last_command_ms + _command_timeout_ms)) >= 0) {
    _serial.println("Serial command timeout. Stopping motors.");
    if (_direct_drive_active) {
      requestDirectDriveStop();
    } else {
      _motion_controller.stop();
    }
    _forward_mode_active = false;
    _steering_correction_active = false;
    _validation_pulse_active = false;
    _validation_pulse_end_ms = 0;
    _has_received_command = false;
  }
}


void SerialCommandHandler::setDirectDriveTarget(
  float left_frequency_hz,
  float right_frequency_hz
) {
  _target_left_frequency_hz = left_frequency_hz;
  _target_right_frequency_hz = right_frequency_hz;
  _forward_mode_active = true;
  _direct_drive_active = true;
  _direct_drive_stop_requested = false;

  if (_last_drive_update_ms == 0) {
    _last_drive_update_ms = millis();
  }
}


void SerialCommandHandler::requestDirectDriveStop() {
  _target_left_frequency_hz = 0.0;
  _target_right_frequency_hz = 0.0;
  _direct_drive_stop_requested = true;
  _forward_mode_active = false;
  _steering_correction_active = false;
  _validation_pulse_active = false;
  _validation_pulse_end_ms = 0;

  if (_last_drive_update_ms == 0) {
    _last_drive_update_ms = millis();
  }
}


void SerialCommandHandler::updateDirectDriveRamp() {
  if (!_direct_drive_active || _validation_drive == nullptr) {
    return;
  }

  unsigned long now_ms = millis();

  if (_last_drive_update_ms == 0) {
    _last_drive_update_ms = now_ms;
    return;
  }

  float dt_sec = (float)(now_ms - _last_drive_update_ms) / 1000.0;
  if (dt_sec <= 0.0) {
    return;
  }

  _last_drive_update_ms = now_ms;

  if (_direct_drive_acceleration_hz_per_sec <= 0.0) {
    _serial.println(
      "Warning: direct drive acceleration must be configured."
    );
    return;
  }

  float max_delta = _direct_drive_acceleration_hz_per_sec * dt_sec;

  _current_left_frequency_hz = moveToward(
    _current_left_frequency_hz,
    _target_left_frequency_hz,
    max_delta
  );
  _current_right_frequency_hz = moveToward(
    _current_right_frequency_hz,
    _target_right_frequency_hz,
    max_delta
  );

  if (
    _current_left_frequency_hz <= 0.0 &&
    _current_right_frequency_hz <= 0.0
  ) {
    _validation_drive->stop();
  } else {
    _validation_drive->setMotorFrequencies(
      _current_left_frequency_hz,
      _current_right_frequency_hz
    );
  }

  printDirectDriveDiagnostics();

  if (_direct_drive_stop_requested && frequenciesReachedTarget()) {
    _validation_drive->stop();
    _direct_drive_active = false;
    _direct_drive_stop_requested = false;
    _current_left_frequency_hz = 0.0;
    _current_right_frequency_hz = 0.0;
    _last_drive_update_ms = 0;
    _serial.println("ALL_MOTION_STOPPED");
    printCommandState();
  }
}


float SerialCommandHandler::moveToward(
  float current_value,
  float target_value,
  float max_delta
) const {
  if (current_value < target_value) {
    current_value += max_delta;
    if (current_value > target_value) {
      return target_value;
    }
    return current_value;
  }

  if (current_value > target_value) {
    current_value -= max_delta;
    if (current_value < target_value) {
      return target_value;
    }
    return current_value;
  }

  return current_value;
}


bool SerialCommandHandler::frequenciesReachedTarget() const {
  return
    _current_left_frequency_hz == _target_left_frequency_hz &&
    _current_right_frequency_hz == _target_right_frequency_hz;
}


void SerialCommandHandler::printDirectDriveDiagnostics() {
  unsigned long now_ms = millis();
  if ((now_ms - _last_direct_drive_diagnostics_ms) < 250) {
    return;
  }

  _serial.print("Target Left: ");
  _serial.println(_target_left_frequency_hz);
  _serial.print("Target Right: ");
  _serial.println(_target_right_frequency_hz);
  _serial.print("Current Left: ");
  _serial.println(_current_left_frequency_hz);
  _serial.print("Current Right: ");
  _serial.println(_current_right_frequency_hz);

  _last_direct_drive_diagnostics_ms = now_ms;
}


void SerialCommandHandler::startValidationSteeringPulse(
  bool turn_left,
  uint32_t duration_ms
) {
  if (_validation_drive == nullptr) {
    _serial.println(
      "Warning: validation drive is not configured for pulse commands."
    );
    return;
  }

  if (duration_ms == 0) {
    _serial.println("Warning: pulse duration must be greater than 0 ms.");
    return;
  }

  _validation_drive->stop();
  _validation_drive->setFrequency(VALIDATION_STEERING_FREQUENCY_HZ);

  if (turn_left) {
    _validation_drive->turnLeft(0);
  } else {
    _validation_drive->turnRight(0);
  }

  _validation_pulse_active = true;
  _steering_correction_active = true;
  _validation_pulse_end_ms = millis() + duration_ms;

  _serial.print("Correction Frequency Hz: ");
  _serial.println(VALIDATION_STEERING_FREQUENCY_HZ);
}


void SerialCommandHandler::startValidationSteeringCorrection(bool turn_left) {
  if (_validation_drive == nullptr) {
    _serial.println(
      "Warning: validation drive is not configured for correction commands."
    );
    return;
  }

  _validation_drive->setFrequency(VALIDATION_STEERING_FREQUENCY_HZ);

  if (turn_left) {
    _validation_drive->turnLeft(0);
  } else {
    _validation_drive->turnRight(0);
  }

  _steering_correction_active = true;
  _validation_pulse_end_ms = 0;

  _serial.print("Correction Frequency Hz: ");
  _serial.println(VALIDATION_STEERING_FREQUENCY_HZ);
}


void SerialCommandHandler::stopValidationSteeringCorrection() {
  if (_validation_drive != nullptr && !_forward_mode_active) {
    _validation_drive->stop();
  }

  _steering_correction_active = false;
  _validation_pulse_end_ms = 0;
}


void SerialCommandHandler::updateValidationSteeringPulse() {
  if (!_validation_pulse_active || _validation_drive == nullptr) {
    return;
  }

  if (_validation_pulse_end_ms == 0) {
    return;
  }

  if ((long)(millis() - _validation_pulse_end_ms) < 0) {
    return;
  }

  stopValidationSteeringCorrection();
  _validation_pulse_active = false;
  _serial.println("Validation Pulse Complete: STOP");
}
