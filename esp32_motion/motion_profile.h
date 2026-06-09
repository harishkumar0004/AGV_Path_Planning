#ifndef ESP32_MOTION_PROFILE_H
#define ESP32_MOTION_PROFILE_H

#include <Arduino.h>
#include "motion_types.h"


class MotionProfile {
public:
  MotionProfile();

  void configure(
    uint32_t total_steps,
    float max_frequency_hz,
    float acceleration_hz_per_sec
  );
  MotionProfileState calculate(uint32_t current_step_count) const;
  bool isTriangular() const;
  uint32_t getTotalSteps() const;

private:
  uint32_t _total_steps;
  uint32_t _acceleration_steps;
  uint32_t _cruise_steps;
  uint32_t _deceleration_steps;
  float _max_frequency_hz;
  float _acceleration_hz_per_sec;
  bool _triangular;

  float calculateFrequencyForStep(uint32_t phase_step) const;
};

#endif
