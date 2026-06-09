#include "serial_command_handler.h"


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

  _last_command_ms = millis();
  _has_received_command = true;

  _serial.print("RX: ");
  _serial.println(command);

  if (command == "FORWARD") {
    _serial.println("Executing moveForward(100)");
    _motion_controller.moveForward(100.0);
    return;
  }

  if (command == "START_CONTINUOUS_FORWARD") {
    _serial.println("Executing moveForward(100000)");
    _motion_controller.moveForward(100000.0);
    return;
  }

  if (command == "SLOW_FORWARD") {
    _serial.println("Executing moveForward(50)");
    _motion_controller.moveForward(50.0);
    return;
  }

  if (command == "SLOW_MODE") {
    _serial.println("Executing moveForward(100)");
    _motion_controller.moveForward(100.0);
    return;
  }

  if (command == "LEFT") {
    _serial.println("Executing turnLeft(10)");
    _motion_controller.turnLeft(10.0);
    return;
  }

  if (command == "RIGHT") {
    _serial.println("Executing turnRight(10)");
    _motion_controller.turnRight(10.0);
    return;
  }

  if (command == "TURN_RIGHT") {
    _serial.println("Executing turnRight(10)");
    _motion_controller.turnRight(10.0);
    return;
  }

  if (command == "STOP") {
    _serial.println("Executing stop()");
    _motion_controller.stop();
    return;
  }

  _serial.print("Warning: unknown command ignored: ");
  _serial.println(command);
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
