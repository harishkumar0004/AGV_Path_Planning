#ifndef ESP32_MOTION_STEP_GENERATOR_H
#define ESP32_MOTION_STEP_GENERATOR_H

#include <Arduino.h>


class StepGenerator {
public:
  StepGenerator(
    uint8_t step_pin,
    uint8_t dir_pin,
    uint8_t enable_pin,
    uint8_t timer_index,
    bool enable_active_low = true
  );

  void begin();
  void enable();
  void disable();
  void setDirection(bool forward);
  void setFrequency(float frequency_hz);
  void start(uint32_t target_steps);
  void stop();

  bool isRunning() const;
  bool isComplete() const;
  uint32_t getStepCount() const;
  uint32_t getTargetSteps() const;

private:
  uint8_t _step_pin;
  uint8_t _dir_pin;
  uint8_t _enable_pin;
  uint8_t _timer_index;
  bool _enable_active_low;

  hw_timer_t *_timer;
  portMUX_TYPE _timer_mux;

  volatile bool _running;
  volatile bool _complete;
  volatile bool _step_pin_high;
  volatile uint32_t _step_count;
  volatile uint32_t _target_steps;
  float _frequency_hz;

  void handleInterrupt();
  uint32_t frequencyToHalfPeriodMicros(float frequency_hz) const;

  static void IRAM_ATTR onTimer(void *arg);
};

#endif
