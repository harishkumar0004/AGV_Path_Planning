#include "step_generator.h"


StepGenerator::StepGenerator(
  uint8_t step_pin,
  uint8_t dir_pin,
  uint8_t enable_pin,
  uint8_t timer_index,
  bool enable_active_low
) : _timer_mux(portMUX_INITIALIZER_UNLOCKED) {
  _step_pin = step_pin;
  _dir_pin = dir_pin;
  _enable_pin = enable_pin;
  _timer_index = timer_index;
  _enable_active_low = enable_active_low;
  _timer = nullptr;
  _running = false;
  _complete = false;
  _step_pin_high = false;
  _step_count = 0;
  _target_steps = 0;
  _frequency_hz = 1.0;
}


void StepGenerator::begin() {
  pinMode(_step_pin, OUTPUT);
  pinMode(_dir_pin, OUTPUT);
  pinMode(_enable_pin, OUTPUT);

  digitalWrite(_step_pin, LOW);
  digitalWrite(_dir_pin, LOW);
  disable();

  // Arduino-ESP32 3.x timer API: create a timer with 1 MHz resolution.
  // That means timerAlarm values are expressed directly in microseconds.
  _timer = timerBegin(1000000);
  timerAttachInterruptArg(_timer, &StepGenerator::onTimer, this);
  timerAlarm(_timer, frequencyToHalfPeriodMicros(_frequency_hz), true, 0);
  timerStop(_timer);
}


void StepGenerator::enable() {
  digitalWrite(_enable_pin, _enable_active_low ? LOW : HIGH);
}


void StepGenerator::disable() {
  digitalWrite(_enable_pin, _enable_active_low ? HIGH : LOW);
}


void StepGenerator::setDirection(bool forward) {
  digitalWrite(_dir_pin, forward ? HIGH : LOW);
}


void StepGenerator::setFrequency(float frequency_hz) {
  if (frequency_hz < 1.0) {
    frequency_hz = 1.0;
  }

  _frequency_hz = frequency_hz;

  if (_timer != nullptr) {
    timerAlarm(_timer, frequencyToHalfPeriodMicros(_frequency_hz), true, 0);
  }
}


void StepGenerator::start(uint32_t target_steps) {
  portENTER_CRITICAL(&_timer_mux);
  _target_steps = target_steps;
  _step_count = 0;
  _complete = false;
  _running = true;
  _step_pin_high = false;
  portEXIT_CRITICAL(&_timer_mux);

  digitalWrite(_step_pin, LOW);
  enable();

  if (_timer != nullptr) {
    timerWrite(_timer, 0);
    timerStart(_timer);
  }
}


void StepGenerator::stop() {
  portENTER_CRITICAL(&_timer_mux);
  _running = false;
  _complete = true;
  _step_pin_high = false;
  portEXIT_CRITICAL(&_timer_mux);

  if (_timer != nullptr) {
    timerStop(_timer);
  }

  digitalWrite(_step_pin, LOW);
  disable();
}


bool StepGenerator::isRunning() const {
  return _running;
}


bool StepGenerator::isComplete() const {
  return _complete;
}


uint32_t StepGenerator::getStepCount() const {
  return _step_count;
}


uint32_t StepGenerator::getTargetSteps() const {
  return _target_steps;
}


void IRAM_ATTR StepGenerator::handleInterrupt() {
  if (!_running) {
    return;
  }

  if (_step_pin_high) {
    digitalWrite(_step_pin, LOW);
    _step_pin_high = false;
    return;
  }

  if (_target_steps > 0 && _step_count >= _target_steps) {
    digitalWrite(_step_pin, LOW);
    _running = false;
    _complete = true;
    return;
  }

  digitalWrite(_step_pin, HIGH);
  _step_pin_high = true;
  _step_count++;
}


uint32_t StepGenerator::frequencyToHalfPeriodMicros(float frequency_hz) const {
  if (frequency_hz < 1.0) {
    frequency_hz = 1.0;
  }

  return (uint32_t)(500000.0 / frequency_hz);
}


void IRAM_ATTR StepGenerator::onTimer(void *arg) {
  StepGenerator *generator = static_cast<StepGenerator *>(arg);
  if (generator != nullptr) {
    generator->handleInterrupt();
  }
}
