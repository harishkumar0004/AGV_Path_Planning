#pragma once
#include <Arduino.h>
#include "StepperTimerMotor.h"

class DifferentialDrive {
public:
    DifferentialDrive(StepperTimerMotor& leftMotor,
                      StepperTimerMotor& rightMotor);

    void begin();
    void stop();

    void setWheelRPM(float leftRpm, float rightRpm);

    void driveForwardRPM(float rpm);
    void driveBackwardRPM(float rpm);

    void rotateClockwiseRPM(float rpm);
    void rotateCounterClockwiseRPM(float rpm);

    void setWheelVelocity(float leftMps, float rightMps);
    void setRobotVelocity(float vMps, float wRadps);

private:
    StepperTimerMotor& _left;
    StepperTimerMotor& _right;
};