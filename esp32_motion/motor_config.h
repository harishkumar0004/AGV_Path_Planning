#ifndef ESP32_MOTION_MOTOR_CONFIG_H
#define ESP32_MOTION_MOTOR_CONFIG_H

#include <Arduino.h>


struct MotorConfig {
  float wheel_diameter_mm;
  float wheel_base_mm;
  uint32_t pulses_per_revolution;
  float max_frequency_hz;
  float max_acceleration_hz_per_sec;
  float min_start_frequency_hz;
  uint32_t direction_change_settling_ms;
};

#endif
