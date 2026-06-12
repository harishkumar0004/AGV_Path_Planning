#include "serial_command_handler.h"


const float DEFAULT_TURN_LEFT_VALUE = 10.0;
const float DEFAULT_TURN_RIGHT_VALUE = 10.0;
const float VALIDATION_STEERING_FREQUENCY_HZ = 2000.0;
const uint32_t DEFAULT_STEERING_PULSE_MS = 100;


SerialCommandHandler::SerialCommandHandler(
  HardwareSerial &serial,
  MotionController &motion_controller,
  uint32_t command_timeout_ms,
  DifferentialDrive *validation_drive,
  ImuManager *imu_manager
) : _serial(serial), _motion_controller(motion_controller) {
  _validation_drive = validation_drive;
  _imu_manager = imu_manager;
  _command_timeout_ms = command_timeout_ms;
  _last_command_ms = 0;
  _has_received_command = false;
  _command_buffer = "";
  _validation_pulse_active = false;
  _validation_pulse_end_ms = 0;
}


void SerialCommandHandler::begin(uint32_t baudrate) {
  _serial.begin(baudrate);
  _serial.println("ESP32 serial command handler ready");
}


void SerialCommandHandler::update() {
  updateValidationSteeringPulse();

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

  if (command_name == "STATUS") {
    _serial.print("STATUS: ");
    _serial.println(
      (_motion_controller.isRunning() || _validation_pulse_active)
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
    return;
  }

  if (command_name == "START_RIGHT_CORRECTION") {
    _serial.println("Validation Correction: START_RIGHT_CORRECTION");
    startValidationSteeringCorrection(false);
    return;
  }

  if (command_name == "STOP_CORRECTION") {
    _serial.println("Validation Correction: STOP_CORRECTION");
    stopValidationSteeringCorrection();
    return;
  }

  if (command_name == "START_FORWARD") {
    _serial.println("Motion Mode: FORWARD_MODE");
    _motion_controller.startForwardMode();
    return;
  }

  if (command_name == "START_SLOW_FORWARD") {
    _serial.println("Motion Mode: SLOW_FORWARD_MODE");
    _motion_controller.startSlowForwardMode();
    return;
  }

  if (command_name == "STOP") {
    _serial.println("Motion Mode: STOPPING");
    _serial.println("Executing stop()");
    if (_validation_drive != nullptr) {
      stopValidationSteeringCorrection();
    }
    _motion_controller.stop();
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


void SerialCommandHandler::checkSerialSafety() {
  if (!_has_received_command || _command_timeout_ms == 0) {
    return;
  }

  if (!_motion_controller.isRunning()) {
    return;
  }

  if ((long)(millis() - (_last_command_ms + _command_timeout_ms)) >= 0) {
    _serial.println("Serial command timeout. Stopping motors.");
    _motion_controller.stop();
    _has_received_command = false;
  }
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

  _motion_controller.stop();
  _validation_drive->stop();
  _validation_drive->setFrequency(VALIDATION_STEERING_FREQUENCY_HZ);

  if (turn_left) {
    _validation_drive->turnLeft(0);
  } else {
    _validation_drive->turnRight(0);
  }

  _validation_pulse_active = true;
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

  _motion_controller.stop();
  _validation_drive->stop();
  _validation_drive->setFrequency(VALIDATION_STEERING_FREQUENCY_HZ);

  if (turn_left) {
    _validation_drive->turnLeft(0);
  } else {
    _validation_drive->turnRight(0);
  }

  _validation_pulse_active = true;
  _validation_pulse_end_ms = 0;

  _serial.print("Correction Frequency Hz: ");
  _serial.println(VALIDATION_STEERING_FREQUENCY_HZ);
}


void SerialCommandHandler::stopValidationSteeringCorrection() {
  if (_validation_drive != nullptr) {
    _validation_drive->stop();
  }

  _validation_pulse_active = false;
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
  _serial.println("Validation Pulse Complete: STOP");
}
