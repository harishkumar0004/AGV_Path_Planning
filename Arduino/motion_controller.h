#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include "step_generator.h"


enum MotionPhase {
  PHASE_ACCELERATION,
  PHASE_CRUISE,
  PHASE_DECELERATION,
  PHASE_STOPPED
};


class MotionController {
public:
  MotionController(
    StepGenerator &leftMotor,
    StepGenerator &rightMotor,
    float maxSpeedStepsPerSec,
    float accelerationStepsPerSec2
  );

  void begin();
  void setMotionParameters(
    float maxSpeedStepsPerSec,
    float accelerationStepsPerSec2
  );
  void moveForward(long steps);
  void moveBackward(long steps);
  void turnLeft(long steps);
  void turnRight(long steps);
  void stop();

private:
  StepGenerator &_leftMotor;
  StepGenerator &_rightMotor;
  float _maxSpeedStepsPerSec;
  float _accelerationStepsPerSec2;
  bool _stopRequested;

  void executeMove(
    long steps,
    bool leftForward,
    bool rightForward,
    const char *motionName
  );
  MotionPhase getPhase(
    long stepIndex,
    long totalSteps,
    long accelerationSteps,
    long decelerationSteps
  );
  float calculateSpeed(MotionPhase phase, long phaseStep);
  unsigned long calculateStepIntervalMicros(float speedStepsPerSec);
  void waitMicros(unsigned long intervalMicros);
  void printDiagnostics(
    const char *motionName,
    MotionPhase phase,
    float speedStepsPerSec,
    long remainingSteps
  );
  const char *phaseToText(MotionPhase phase);
};

#endif
