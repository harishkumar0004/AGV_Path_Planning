#ifndef STEP_GENERATOR_H
#define STEP_GENERATOR_H

#include <Arduino.h>


class StepGenerator {
public:
  StepGenerator(
    uint8_t stepPin,
    uint8_t dirPin,
    uint8_t enablePin,
    bool enableActiveLow = true
  );

  void begin();
  void enable();
  void disable();
  void setDirection(bool forward);
  void generateStep();

private:
  uint8_t _stepPin;
  uint8_t _dirPin;
  uint8_t _enablePin;
  bool _enableActiveLow;
  unsigned int _stepPulseWidthMicros;
};

#endif
