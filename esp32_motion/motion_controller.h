#ifndef ESP32_MOTION_CONTROLLER_H
#define ESP32_MOTION_CONTROLLER_H

#include <Arduino.h>
#include "differential_drive.h"
#include "motor_config.h"
#include "motion_profile.h"
#include "motion_types.h"


enum MotionMode {
  MODE_IDLE,
  MODE_FORWARD,
  MODE_SLOW_FORWARD,
  MODE_STOPPING,
  MODE_TURNING_LEFT,
  MODE_TURNING_RIGHT
};


class MotionController {
public:
  MotionController(MotorConfig config, DifferentialDrive &drive);

  void begin();
  void update();
  void startForwardMode();
  void startSlowForwardMode();
  void moveForward(float distance_mm);
  void moveBackward(float distance_mm);
  void turnLeft(float angle_deg);
  void turnRight(float angle_deg);
  void stop();
  bool isRunning() const;

private:
  MotorConfig _config;
  DifferentialDrive &_drive;
  MotionProfile _profile;
  MotionMode _motion_mode;
  MotionPhase _last_phase;
  unsigned long _last_diagnostics_ms;
  uint32_t _profile_start_step_count;
  uint32_t _profile_step_offset;
  uint32_t _continuous_cruise_step;
  float _last_profile_frequency_hz;
  float _speed_transition_target_hz;
  bool _continuous_profile_active;
  bool _speed_transition_active;

  uint32_t distanceMmToSteps(float distance_mm) const;
  uint32_t angleDegToTurnSteps(float angle_deg) const;
  uint32_t frequencyToAccelerationSteps(float frequency_hz) const;
  void startProfile(uint32_t steps, float max_frequency_hz);
  void startContinuousProfile(float cruise_frequency_hz);
  void startContinuousCruise(float cruise_frequency_hz);
  void startSpeedTransition(float target_frequency_hz, MotionMode target_mode);
  void startStopProfile();
  MotionProfileState calculateActiveProfileState() const;
  void applyProfileState(const MotionProfileState &state);
  void printDiagnostics(const MotionProfileState &state);
  void printMotionMode();
  const char *motionModeToText(MotionMode mode) const;
};

#endif
