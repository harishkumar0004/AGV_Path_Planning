#ifndef ESP32_SERIAL_COMMAND_HANDLER_H
#define ESP32_SERIAL_COMMAND_HANDLER_H

#include <Arduino.h>
#include "motion_controller.h"


class SerialCommandHandler {
public:
  SerialCommandHandler(
    HardwareSerial &serial,
    MotionController &motion_controller,
    uint32_t command_timeout_ms = 2000
  );

  void begin(uint32_t baudrate);
  void update();

private:
  HardwareSerial &_serial;
  MotionController &_motion_controller;
  uint32_t _command_timeout_ms;
  unsigned long _last_command_ms;
  bool _has_received_command;
  String _command_buffer;

  void processCommand(String command);
  float getCommandValue(String command, float default_value) const;
  void checkSerialSafety();
};

#endif
