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



