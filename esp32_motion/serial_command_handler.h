#ifndef ESP32_SERIAL_COMMAND_HANDLER_H
#define ESP32_SERIAL_COMMAND_HANDLER_H

#include <Arduino.h>
#include "differential_drive.h"
#include "motion_controller.h"


class SerialCommandHandler {
public:
  SerialCommandHandler(
    HardwareSerial &serial,
    MotionController &motion_controller,
    uint32_t command_timeout_ms = 2000,
    DifferentialDrive *validation_drive = nullptr
  );

  void begin(uint32_t baudrate);
  void update();
  bool isValidationPulseActive() const;

private:
  HardwareSerial &_serial;
  MotionController &_motion_controller;
  DifferentialDrive *_validation_drive;
  uint32_t _command_timeout_ms;
  unsigned long _last_command_ms;
  bool _has_received_command;
  String _command_buffer;
  bool _validation_pulse_active;
  unsigned long _validation_pulse_end_ms;

  void processCommand(String command);
  float getCommandValue(String command, float default_value) const;
  void checkSerialSafety();
  void startValidationSteeringPulse(
    bool turn_left,
    uint32_t duration_ms
  );
  void updateValidationSteeringPulse();
};

#endif
