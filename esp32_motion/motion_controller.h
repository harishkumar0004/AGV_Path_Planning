#ifndef ESP32_MOTION_CONTROLLER_H
#define ESP32_MOTION_CONTROLLER_H

#include <Arduino.h>
#include "differential_drive.h"
#include "motor_config.h"
#include "motion_profile.h"
#include "motion_types.h"


enum MotionCommandType {
  COMMAND_NONE,
  COMMAND_FORWARD,
  COMMAND_BACKWARD,
  COMMAND_TURN_LEFT,
  COMMAND_TURN_RIGHT
};


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
  MotionCommandType _previous_motion;
  MotionCommandType _active_motion;
  MotionCommandType _pending_motion;
  uint32_t _pending_steps;
  bool _pending_left_forward;
  bool _pending_right_forward;
  bool _has_previous_direction;
  bool _previous_left_forward;
  bool _previous_right_forward;
  bool _settling_active;
  unsigned long _settling_until_ms;

  uint32_t distanceMmToSteps(float distance_mm) const;
  uint32_t angleDegToTurnSteps(float angle_deg) const;
  void requestMotion(
    MotionCommandType new_motion,
    uint32_t steps,
    bool left_forward,
    bool right_forward
  );
  void startPendingMotion();
  void completeActiveMotion();
  void configureProfile(uint32_t steps);
  void printDiagnostics(const MotionProfileState &state);
  void printTransitionDiagnostics(
    MotionCommandType previous_motion,
    MotionCommandType new_motion,
    bool direction_changed,
    uint32_t settling_ms
  );
  const char *motionCommandToText(MotionCommandType motion) const;
};

#endif
