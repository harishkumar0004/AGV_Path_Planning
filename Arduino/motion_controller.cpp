#include "motion_controller.h"


MotionController::MotionController(
  StepGenerator &leftMotor,
  StepGenerator &rightMotor,
  float maxSpeedStepsPerSec,
  float accelerationStepsPerSec2
) : _leftMotor(leftMotor), _rightMotor(rightMotor) {
  _maxSpeedStepsPerSec = maxSpeedStepsPerSec;
  _accelerationStepsPerSec2 = accelerationStepsPerSec2;
  _stopRequested = false;
}


void MotionController::begin() {
  _leftMotor.begin();
  _rightMotor.begin();
  stop();
}


void MotionController::setMotionParameters(
  float maxSpeedStepsPerSec,
  float accelerationStepsPerSec2
) {
  if (maxSpeedStepsPerSec > 0) {
    _maxSpeedStepsPerSec = maxSpeedStepsPerSec;
  }

  if (accelerationStepsPerSec2 > 0) {
    _accelerationStepsPerSec2 = accelerationStepsPerSec2;
  }
}


void MotionController::moveForward(long steps) {
  executeMove(steps, true, true, "FORWARD");
}


void MotionController::moveBackward(long steps) {
  executeMove(steps, false, false, "BACKWARD");
}


void MotionController::turnLeft(long steps) {
  executeMove(steps, false, true, "TURN_LEFT");
}


void MotionController::turnRight(long steps) {
  executeMove(steps, true, false, "TURN_RIGHT");
}


void MotionController::stop() {
  _stopRequested = true;
  _leftMotor.disable();
  _rightMotor.disable();
  Serial.println("Motion stopped");
}


void MotionController::executeMove(
  long steps,
  bool leftForward,
  bool rightForward,
  const char *motionName
) {
  if (steps <= 0) {
    return;
  }

  _stopRequested = false;
  _leftMotor.setDirection(leftForward);
  _rightMotor.setDirection(rightForward);
  _leftMotor.enable();
  _rightMotor.enable();

  float accelerationDistance =
    (_maxSpeedStepsPerSec * _maxSpeedStepsPerSec) /
    (2.0 * _accelerationStepsPerSec2);

  long accelerationSteps = (long)(accelerationDistance + 0.5);
  long decelerationSteps = accelerationSteps;

  if ((accelerationSteps + decelerationSteps) > steps) {
    accelerationSteps = steps / 2;
    decelerationSteps = steps - accelerationSteps;
  }

  MotionPhase previousPhase = PHASE_STOPPED;

  Serial.print("Starting motion: ");
  Serial.println(motionName);

  for (long stepIndex = 0; stepIndex < steps; stepIndex++) {
    if (_stopRequested) {
      break;
    }

    MotionPhase phase = getPhase(
      stepIndex,
      steps,
      accelerationSteps,
      decelerationSteps
    );

    long phaseStep = stepIndex + 1;
    if (phase == PHASE_CRUISE) {
      phaseStep = accelerationSteps;
    } else if (phase == PHASE_DECELERATION) {
      phaseStep = steps - stepIndex;
    }

    float speedStepsPerSec = calculateSpeed(phase, phaseStep);
    unsigned long stepIntervalMicros =
      calculateStepIntervalMicros(speedStepsPerSec);

    _leftMotor.generateStep();
    _rightMotor.generateStep();

    if (phase != previousPhase || (stepIndex % 100 == 0)) {
      printDiagnostics(
        motionName,
        phase,
        speedStepsPerSec,
        steps - stepIndex - 1
      );
      previousPhase = phase;
    }

    waitMicros(stepIntervalMicros);
  }

  _leftMotor.disable();
  _rightMotor.disable();
  Serial.print("Completed motion: ");
  Serial.println(motionName);
}


MotionPhase MotionController::getPhase(
  long stepIndex,
  long totalSteps,
  long accelerationSteps,
  long decelerationSteps
) {
  if (stepIndex < accelerationSteps) {
    return PHASE_ACCELERATION;
  }

  if (stepIndex >= (totalSteps - decelerationSteps)) {
    return PHASE_DECELERATION;
  }

  return PHASE_CRUISE;
}


float MotionController::calculateSpeed(MotionPhase phase, long phaseStep) {
  if (phase == PHASE_CRUISE) {
    return _maxSpeedStepsPerSec;
  }

  float speed = sqrt(2.0 * _accelerationStepsPerSec2 * phaseStep);

  if (speed > _maxSpeedStepsPerSec) {
    return _maxSpeedStepsPerSec;
  }

  if (speed < 1.0) {
    return 1.0;
  }

  return speed;
}


unsigned long MotionController::calculateStepIntervalMicros(
  float speedStepsPerSec
) {
  return (unsigned long)(1000000.0 / speedStepsPerSec);
}


void MotionController::waitMicros(unsigned long intervalMicros) {
  while (intervalMicros > 16000UL) {
    delayMicroseconds(16000);
    intervalMicros -= 16000UL;
  }

  delayMicroseconds((unsigned int)intervalMicros);
}


void MotionController::printDiagnostics(
  const char *motionName,
  MotionPhase phase,
  float speedStepsPerSec,
  long remainingSteps
) {
  Serial.print("Motion: ");
  Serial.print(motionName);
  Serial.print(" | Phase: ");
  Serial.print(phaseToText(phase));
  Serial.print(" | Speed steps/sec: ");
  Serial.print(speedStepsPerSec);
  Serial.print(" | Remaining steps: ");
  Serial.println(remainingSteps);
}


const char *MotionController::phaseToText(MotionPhase phase) {
  switch (phase) {
    case PHASE_ACCELERATION:
      return "Acceleration";
    case PHASE_CRUISE:
      return "Constant Velocity";
    case PHASE_DECELERATION:
      return "Deceleration";
    case PHASE_STOPPED:
      return "Stopped";
    default:
      return "Unknown";
  }
}
