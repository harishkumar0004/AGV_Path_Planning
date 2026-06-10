#include "serial_command_handler.h"


const float DEFAULT_TURN_LEFT_VALUE = 10.0;
const float DEFAULT_TURN_RIGHT_VALUE = 10.0;


SerialCommandHandler::SerialCommandHandler(
  HardwareSerial &serial,
  MotionController &motion_controller,
  uint32_t command_timeout_ms
) : _serial(serial), _motion_controller(motion_controller) {
  _command_timeout_ms = command_timeout_ms;
  _last_command_ms = 0;
  _has_received_command = false;
  _command_buffer = "";
}


void SerialCommandHandler::begin(uint32_t baudrate) {
  _serial.begin(baudrate);
  _serial.println("ESP32 serial command handler ready");
}


void SerialCommandHandler::update() {
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
    _serial.println(_motion_controller.isRunning() ? "RUNNING" : "IDLE");
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
