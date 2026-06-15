#ifndef ESP32_SERIAL_COMMAND_HANDLER_H
#define ESP32_SERIAL_COMMAND_HANDLER_H

#include <Arduino.h>
#include "differential_drive.h"
#include "imu_manager.h"
#include "motion_controller.h"


class SerialCommandHandler {
public:
  SerialCommandHandler(
    HardwareSerial &serial,
    MotionController &motion_controller,
    uint32_t command_timeout_ms = 2000,
    DifferentialDrive *validation_drive = nullptr,
    ImuManager *imu_manager = nullptr,
    float direct_drive_acceleration_hz_per_sec = 0.0
  );

  void begin(uint32_t baudrate);
  void update();
  bool isValidationPulseActive() const;
  bool isDirectDriveActive() const;

private:
  HardwareSerial &_serial;
  MotionController &_motion_controller;
  DifferentialDrive *_validation_drive;
  ImuManager *_imu_manager;
  uint32_t _command_timeout_ms;
  unsigned long _last_command_ms;
  bool _has_received_command;
  String _command_buffer;
  bool _validation_pulse_active;
  bool _steering_correction_active;
  bool _forward_mode_active;
  bool _direct_drive_active;
  bool _direct_drive_stop_requested;
  unsigned long _validation_pulse_end_ms;
  unsigned long _last_drive_update_ms;
  unsigned long _last_direct_drive_diagnostics_ms;
  float _direct_drive_acceleration_hz_per_sec;
  float _target_left_frequency_hz;
  float _target_right_frequency_hz;
  float _current_left_frequency_hz;
  float _current_right_frequency_hz;

  void processCommand(String command);
  float getCommandValue(String command, float default_value) const;
  bool parseSetDriveCommand(
    String command,
    float &left_frequency_hz,
    float &right_frequency_hz
  ) const;
  float clampDriveFrequency(float frequency_hz) const;
  void printCommandState() const;
  void checkSerialSafety();
  void startValidationSteeringPulse(
    bool turn_left,
    uint32_t duration_ms
  );
  void startValidationSteeringCorrection(bool turn_left);
  void stopValidationSteeringCorrection();
  void updateValidationSteeringPulse();
  void setDirectDriveTarget(
    float left_frequency_hz,
    float right_frequency_hz
  );
  void requestDirectDriveStop();
  void updateDirectDriveRamp();
  float moveToward(
    float current_value,
    float target_value,
    float max_delta
  ) const;
  bool frequenciesReachedTarget() const;
  void printDirectDriveDiagnostics();
};

#endif
