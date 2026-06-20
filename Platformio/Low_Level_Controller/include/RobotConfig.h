#pragma once
#include <Arduino.h>

// Robot geometry

constexpr float WHEEL_DIAMETER_M = 0.117f;
constexpr float WHEEL_BASE_M = 0.324f;
constexpr float PULSES_PER_REV = 20000.0f;

constexpr float WHEEL_CIRCUMFERENCE_M = PI * WHEEL_DIAMETER_M;
constexpr float PULSES_PER_METER = PULSES_PER_REV / WHEEL_CIRCUMFERENCE_M;

// speed limits for test

constexpr float DEFAULT_TEST_RPM = 1.0f;
constexpr float MAX_TEST_RPM = 30.0f;

// Local AprilTag alignment

constexpr float THETA_SIGN = 1.0f;
constexpr float X_SIGN = -1.0f;
constexpr float Y_SIGN = 1.0f;

constexpr float K_THETA = 0.008f;      // rad/s per degree
constexpr float K_THETA_FINAL = 0.004f;

constexpr float KY = 0.010f;
constexpr float K_X_STEP_YAW = 0.008f;       // yaw control during rotate-to-offset

constexpr float W_ALIGN_MAX = 0.06f;   // slow
constexpr float W_FINAL_MAX = 0.04f;
constexpr float W_X_STEP_MAX = 0.04f;

constexpr float V_ALIGN_MAX = 0.006f;
constexpr float V_X_STEP = 0.002f;

constexpr float THETA_TOL_DEG = 1.0f;
constexpr float ROUGH_THETA_TOL_DEG = 8.0f;
constexpr float X_TOL_NORM = 0.05f;
constexpr float Y_TOL_NORM = 0.05f;
constexpr float X_SAFE_LIMIT = 0.30f;
constexpr float Y_SAFE_LIMIT = 0.25f;

constexpr float X_STEP_THETA_DEG = 4.0f;      // intentional heading offset
constexpr unsigned long X_STEP_CREEP_MS = 180;
constexpr unsigned long X_STEP_SETTLE_MS = 250;

constexpr float TAG_HARD_EDGE_LIMIT = 0.70f;  // stop if abs(x/y) too large
constexpr unsigned long TAG_TIMEOUT_MS = 600;
// Pose-based AprilTag alignment
// Use pose mode for final checkpoint alignment, not for 90 degree route turns.

constexpr float POSE_X_TOL_M = 0.0025f;       // 2.5 mm
constexpr float POSE_Y_TOL_M = 0.0025f;       // 2.5 mm
constexpr float POSE_X_SAFE_M = 0.0080f;      // 8 mm
constexpr float POSE_Y_SAFE_M = 0.0080f;      // 8 mm
constexpr float POSE_YAW_TOL_DEG = 0.5f;
constexpr float POSE_ROUGH_YAW_TOL_DEG = 8.0f;
constexpr float POSE_LOCAL_UNSAFE_M = 0.015f;

constexpr float K_POSE_Y = 0.60f;             // m/s per meter
constexpr float V_POSE_MAX = 0.006f;

constexpr float K_POSE_YAW = 0.008f;
constexpr float W_POSE_MAX = 0.06f;

constexpr float K_POSE_YAW_FINAL = 0.004f;
constexpr float W_POSE_FINAL_MAX = 0.04f;

constexpr float POSE_X_STEP_THETA_DEG = 4.0f;
constexpr float K_POSE_X_STEP_YAW = 0.008f;
constexpr float W_POSE_X_STEP_MAX = 0.04f;
constexpr float V_POSE_X_STEP = 0.002f;
constexpr unsigned long POSE_X_STEP_CREEP_MS = 150;
constexpr unsigned long POSE_X_STEP_SETTLE_MS = 250;
constexpr float POSE_X_STEP_IMPROVE_M = 0.0010f;

// Blended pose tracking alignment

constexpr float POSE_TRACK_XY_TOL_M = 0.0025f;
constexpr float POSE_TRACK_YAW_TOL_DEG = 1.0f;

constexpr float POSE_TRACK_LOOKAHEAD_M = 0.08f;

constexpr float K_POSE_TRACK_Y = 0.50f;
constexpr float K_POSE_TRACK_W = 0.006f;

constexpr float V_POSE_TRACK_MAX = 0.004f;
constexpr float W_POSE_TRACK_MAX = 0.035f;

constexpr float POSE_TRACK_UNSAFE_X_M = 0.018f;
constexpr float POSE_TRACK_UNSAFE_Y_M = 0.018f;
constexpr float POSE_TRACK_UNSAFE_YAW_DEG = 30.0f;

// Observe-plan-act geometric pose alignment

constexpr float POSE_GEO_XY_TOL_M = 0.0025f;          // 2.5 mm
constexpr float POSE_GEO_YAW_TOL_DEG = 1.0f;

constexpr float POSE_GEO_UNSAFE_X_M = 0.018f;
constexpr float POSE_GEO_UNSAFE_Y_M = 0.018f;
constexpr float POSE_GEO_UNSAFE_YAW_DEG = 30.0f;

constexpr float POSE_GEO_MIN_DIST_M = 0.0015f;
constexpr float POSE_GEO_MAX_STEP_DIST_M = 0.0030f;   // 3 mm per observation cycle

constexpr float POSE_GEO_MAX_TURN_DEG = 5.0f;
constexpr float POSE_GEO_MIN_TURN_DEG = 1.0f;

constexpr float POSE_GEO_TURN_KW = 0.010f;
constexpr float POSE_GEO_TURN_W_MAX = 0.035f;

constexpr float POSE_GEO_CREEP_V = 0.0020f;
constexpr unsigned long POSE_GEO_SETTLE_MS = 250;

constexpr float POSE_GEO_FINAL_YAW_KW = 0.004f;
constexpr float POSE_GEO_FINAL_YAW_W_MAX = 0.025f;

// Motion-primitive pose alignment

constexpr float POSE_PRIM_XY_TOL_M = 0.0025f;
constexpr float POSE_PRIM_YAW_TOL_DEG = 1.0f;

constexpr float POSE_PRIM_UNSAFE_X_M = 0.018f;
constexpr float POSE_PRIM_UNSAFE_Y_M = 0.018f;
constexpr float POSE_PRIM_UNSAFE_YAW_DEG = 30.0f;

constexpr float POSE_PRIM_DT_SEC = 0.15f;
constexpr unsigned long POSE_PRIM_EXEC_MS = 150;
constexpr unsigned long POSE_PRIM_SETTLE_MS = 120;

constexpr float POSE_PRIM_V = 0.0020f;
constexpr float POSE_PRIM_W = 0.025f;

constexpr float POSE_PRIM_WX = 4.0f;
constexpr float POSE_PRIM_WY = 4.0f;
constexpr float POSE_PRIM_WYAW = 0.08f;
constexpr float POSE_PRIM_WEDGE = 20.0f;

constexpr float POSE_PRIM_EDGE_X_M = 0.014f;
constexpr float POSE_PRIM_EDGE_Y_M = 0.014f;

// Measured trial-and-observe pose alignment

constexpr float POSE_TRIAL_XY_TOL_M = 0.0025f;
constexpr float POSE_TRIAL_YAW_TOL_DEG = 1.0f;

constexpr float POSE_TRIAL_UNSAFE_X_M = 0.018f;
constexpr float POSE_TRIAL_UNSAFE_Y_M = 0.018f;
constexpr float POSE_TRIAL_UNSAFE_YAW_DEG = 60.0f;

constexpr float POSE_TRIAL_V = 0.0020f;
constexpr float POSE_TRIAL_W = 0.025f;

constexpr unsigned long POSE_TRIAL_EXEC_MS = 120;
constexpr unsigned long POSE_TRIAL_SETTLE_MS = 250;

constexpr float POSE_TRIAL_MIN_IMPROVE = 0.0005f;

constexpr float POSE_TRIAL_WX = 5.0f;
constexpr float POSE_TRIAL_WY = 5.0f;
constexpr float POSE_TRIAL_WYAW_POSITION = 0.02f;
constexpr float POSE_TRIAL_WYAW_FINAL = 0.10f;

constexpr float POSE_TRIAL_POSITION_READY_M = 0.0040f;

// Phase 4A tag-to-tag navigation

constexpr int NAV_FIRST_TAG_ID = 0;
constexpr int NAV_FINAL_TAG_ID = 1;
constexpr float NAV_TAG_SPACING_M = 0.50f;
constexpr float NAV_V_MAX_MPS = 0.050f;
constexpr float NAV_MIN_V_MPS = 0.015f;
constexpr float NAV_ACCEL_MPS2 = 0.050f;
constexpr float NAV_DECEL_MPS2 = 0.050f;
constexpr float NAV_EXPECTED_TAG_SLOW_ZONE_M = 0.15f;
constexpr float NAV_TAG_CAPTURE_SPEED_MPS = 0.030f;

constexpr float NAV_START_GOOD_X_M = 0.004f;
constexpr float NAV_START_GOOD_Y_M = 0.006f;
constexpr float NAV_START_GOOD_YAW_DEG = 2.0f;

constexpr float NAV_START_SAFE_X_M = 0.014f;
constexpr float NAV_START_SAFE_Y_M = 0.014f;
constexpr float NAV_START_SAFE_YAW_DEG = 12.0f;

constexpr unsigned long NAV_START_GOOD_STABLE_MS = 400;
constexpr unsigned long NAV_START_ALIGN_MAX_MS = 6000;

constexpr unsigned long NAV_TAG_STABLE_MS = 150;
constexpr float NAV_TAG_MAX_ABS_X_M = 0.030f;
constexpr float NAV_TAG_MAX_ABS_Y_M = 0.030f;
constexpr float NAV_TAG_MAX_ABS_YAW_DEG = 35.0f;

constexpr float NAV_TAG_YAW_TO_IMU_SIGN = 1.0f;
constexpr float NAV_TAG_X_TO_HEADING_DEG_PER_M = -80.0f;
constexpr float NAV_TAG_YAW_TO_HEADING_GAIN = 0.30f;
constexpr float NAV_TAG_CORRECTION_MAX_DEG = 4.0f;

// ESP32 motor timer
// 20 us interrupt = 50 kHz timer rate

constexpr uint32_t MOTOR_TIMER_TICK_US = 20;

//Pin mapping 

const uint8_t LEFT_PUL_PIN = 4;
const uint8_t LEFT_DIR_PIN = 13;
const uint8_t LEFT_ENA_PIN = 14;

const uint8_t RIGHT_PUL_PIN = 16;
const uint8_t RIGHT_DIR_PIN = 26;
const uint8_t RIGHT_ENA_PIN = 25;

// Direction inversion flags

constexpr bool LEFT_DIR_INVERT = false;
constexpr bool RIGHT_DIR_INVERT = false;

// Enable pin logic

constexpr bool ENABLE_ACTIVE_LOW = true;

// Conversion functions

inline float rpmToHz(float rpm){
    return rpm * PULSES_PER_REV / 60.0f;
}

inline float hzToRpm(float hz){
    return hz * 60.0f / PULSES_PER_REV;
}

inline float mpsToHz(float speed_mps){
    return fabs(speed_mps) * PULSES_PER_METER; 
}

inline long distanceToPulses(float distance_m){
    return static_cast<long>(distance_m * PULSES_PER_METER);
}

inline uint32_t hzToTimerTicks(float hz){
    if(hz <= 0.0f) return 0;

    float period_us = 1000000.0f / hz;
    uint32_t ticks = static_cast<uint32_t>(period_us) / MOTOR_TIMER_TICK_US;

    if(ticks < 2) ticks = 2;
    return ticks;
}
