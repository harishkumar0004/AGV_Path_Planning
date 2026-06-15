#include "differential_drive.h"


DifferentialDrive::DifferentialDrive(
  StepGenerator &left_motor,
  StepGenerator &right_motor
) : _left_motor(left_motor), _right_motor(right_motor) {
}


void DifferentialDrive::begin() {
  _left_motor.begin();
  _right_motor.begin();
  disable();
}



void DifferentialDrive::enable() {
  _left_motor.enable();
  _right_motor.enable();
}


void DifferentialDrive::disable() {
  _left_motor.disable();
  _right_motor.disable();
}


void DifferentialDrive::moveForward(uint32_t steps) {
  startMove(steps, true, true);
}


void DifferentialDrive::moveBackward(uint32_t steps) {
  startMove(steps, false, false);
}


void DifferentialDrive::turnLeft(uint32_t steps) {
  startMove(steps, false, true);
}


void DifferentialDrive::turnRight(uint32_t steps) {
  startMove(steps, true, false);
}


void DifferentialDrive::stop() {
  _left_motor.stop();
  _right_motor.stop();
}


void DifferentialDrive::setFrequency(float frequency_hz) {
  _left_motor.setFrequency(frequency_hz);
  _right_motor.setFrequency(frequency_hz);
}


void DifferentialDrive::setMotorFrequencies(
  float left_frequency_hz,
  float right_frequency_hz
) {
  _left_motor.setDirection(true);
  _right_motor.setDirection(true);

  _left_motor.setFrequency(left_frequency_hz);
  _right_motor.setFrequency(right_frequency_hz);

  enable();

  if (!_left_motor.isRunning()) {
    _left_motor.start(0);
  }

  if (!_right_motor.isRunning()) {
    _right_motor.start(0);
  }
}


bool DifferentialDrive::isRunning() const {
  return _left_motor.isRunning() || _right_motor.isRunning();
}


uint32_t DifferentialDrive::getStepCount() const {
  uint32_t left_steps = _left_motor.getStepCount();
  uint32_t right_steps = _right_motor.getStepCount();

  return left_steps > right_steps ? left_steps : right_steps;
}


uint32_t DifferentialDrive::getTargetSteps() const {
  return _left_motor.getTargetSteps();
}


void DifferentialDrive::startMove(
  uint32_t steps,
  bool left_forward,
  bool right_forward
) {
  _left_motor.setDirection(left_forward);
  _right_motor.setDirection(right_forward);
  enable();
  _left_motor.start(steps);
  _right_motor.start(steps);
}
