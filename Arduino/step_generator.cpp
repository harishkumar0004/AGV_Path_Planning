#include "step_generator.h"


StepGenerator::StepGenerator(
  uint8_t stepPin,
  uint8_t dirPin,
  uint8_t enablePin,
  bool enableActiveLow
) {
  _stepPin = stepPin;
  _dirPin = dirPin;
  _enablePin = enablePin;
  _enableActiveLow = enableActiveLow;
  _stepPulseWidthMicros = 5;
}


void StepGenerator::begin() {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_enablePin, OUTPUT);

  digitalWrite(_stepPin, LOW);
  digitalWrite(_dirPin, LOW);
  disable();
}


void StepGenerator::enable() {
  digitalWrite(_enablePin, _enableActiveLow ? LOW : HIGH);
}


void StepGenerator::disable() {
  digitalWrite(_enablePin, _enableActiveLow ? HIGH : LOW);
}


void StepGenerator::setDirection(bool forward) {
  digitalWrite(_dirPin, forward ? HIGH : LOW);
}


void StepGenerator::generateStep() {
  digitalWrite(_stepPin, HIGH);
  delayMicroseconds(_stepPulseWidthMicros);
  digitalWrite(_stepPin, LOW);
  delayMicroseconds(_stepPulseWidthMicros);
}
