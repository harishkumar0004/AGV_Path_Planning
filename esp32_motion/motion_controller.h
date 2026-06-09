#ifndef ESP32_MOTION_CONTROLLER_H
#define ESP32_MOTION_CONTROLLER_H

#include <Arduino.h>
#include "differential_drive.h"
#include "motor_config.h"
#include "motion_profile.h"
#include "motion_types.h"


class MotionController {
public:
  MotionController(MotorConfig config, DifferentialDrive &drive);

  void begin();
  void update();
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
  MotionPhase _last_phase;
  unsigned long _last_diagnostics_ms;

  uint32_t distanceMmToSteps(float distance_mm) const;
  uint32_t angleDegToTurnSteps(float angle_deg) const;
  void startProfile(uint32_t steps);
  void printDiagnostics(const MotionProfileState &state);
};

#endif
