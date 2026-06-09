#ifndef ESP32_DIFFERENTIAL_DRIVE_H
#define ESP32_DIFFERENTIAL_DRIVE_H

#include <Arduino.h>
#include "step_generator.h"


class DifferentialDrive {
public:
  DifferentialDrive(StepGenerator &left_motor, StepGenerator &right_motor);

  void begin();
  void enable();
  void disable();
  void moveForward(uint32_t steps);
  void moveBackward(uint32_t steps);
  void turnLeft(uint32_t steps);
  void turnRight(uint32_t steps);
  void stop();
  void setFrequency(float frequency_hz);

  bool isRunning() const;
  uint32_t getStepCount() const;
  uint32_t getTargetSteps() const;

private:
  StepGenerator &_left_motor;
  StepGenerator &_right_motor;

  void startMove(
    uint32_t steps,
    bool left_forward,
    bool right_forward
  );
};

#endif
