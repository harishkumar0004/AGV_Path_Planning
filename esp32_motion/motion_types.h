#ifndef ESP32_MOTION_TYPES_H
#define ESP32_MOTION_TYPES_H

#include <Arduino.h>


enum MotionPhase {
  MOTION_IDLE,
  MOTION_ACCELERATION,
  MOTION_CRUISE,
  MOTION_DECELERATION,
  MOTION_COMPLETE
};


struct MotionProfileState {
  MotionPhase phase;
  float target_frequency_hz;
  uint32_t current_step_count;
  uint32_t remaining_steps;
};


inline const char *motionPhaseToText(MotionPhase phase) {
  switch (phase) {
    case MOTION_IDLE:
      return "IDLE";
    case MOTION_ACCELERATION:
      return "ACCELERATION";
    case MOTION_CRUISE:
      return "CRUISE";
    case MOTION_DECELERATION:
      return "DECELERATION";
    case MOTION_COMPLETE:
      return "COMPLETE";
    default:
      return "UNKNOWN";
  }
}

#endif
