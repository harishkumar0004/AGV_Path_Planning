#include "DifferentialDrive.h"
#include "RobotConfig.h"

DifferentialDrive::DifferentialDrive(StepperTimerMotor& leftMotor,
                                     StepperTimerMotor& rightMotor)
: _left(leftMotor),
  _right(rightMotor) {}

void DifferentialDrive::begin() {
    _left.begin();
    _right.begin();
}

void DifferentialDrive::stop() {
    _left.stop();
    _right.stop();
}

void DifferentialDrive::setWheelRPM(float leftRpm, float rightRpm) {
    _left.setDirection(leftRpm >= 0.0f);
    _right.setDirection(rightRpm >= 0.0f);

    _left.setFrequencyHz(rpmToHz(fabs(leftRpm)));
    _right.setFrequencyHz(rpmToHz(fabs(rightRpm)));
}

void DifferentialDrive::driveForwardRPM(float rpm) {
    setWheelRPM(rpm, rpm);
}

void DifferentialDrive::driveBackwardRPM(float rpm) {
    setWheelRPM(-rpm, -rpm);
}

// Test these physically.
// If CW/CCW are swapped, change only these two functions.
void DifferentialDrive::rotateClockwiseRPM(float rpm) {
    setWheelRPM(-rpm, rpm);
}

void DifferentialDrive::rotateCounterClockwiseRPM(float rpm) {
    setWheelRPM(rpm, -rpm);
}

void DifferentialDrive::setWheelVelocity(float leftMps, float rightMps) {
    _left.setDirection(leftMps >= 0.0f);
    _right.setDirection(rightMps >= 0.0f);

    _left.setFrequencyHz(mpsToHz(leftMps));
    _right.setFrequencyHz(mpsToHz(rightMps));
}

void DifferentialDrive::setRobotVelocity(float vMps, float wRadps) {
    float vLeft  = vMps - (wRadps * WHEEL_BASE_M / 2.0f);
    float vRight = vMps + (wRadps * WHEEL_BASE_M / 2.0f);

    setWheelVelocity(vLeft, vRight);
}