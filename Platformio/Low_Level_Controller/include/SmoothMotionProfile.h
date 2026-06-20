#pragma once

#include <Arduino.h>


enum SmoothMotionPhase {
    SMOOTH_IDLE,
    SMOOTH_ACCELERATION,
    SMOOTH_CRUISE,
    SMOOTH_DECELERATION,
    SMOOTH_COMPLETE,
};


struct SmoothMotionProfileState {
    SmoothMotionPhase phase = SMOOTH_IDLE;
    float targetFrequencyHz = 0.0f;
    uint32_t currentSteps = 0;
    uint32_t remainingSteps = 0;
};


class SmoothMotionProfile {
public:
    void configure(
        uint32_t totalSteps,
        float maxFrequencyHz,
        float accelerationHzPerSec
    );
    SmoothMotionProfileState calculate(uint32_t currentStepCount) const;

    bool isTriangular() const;
    uint32_t getTotalSteps() const;
    uint32_t getAccelerationSteps() const;

private:
    uint32_t _totalSteps = 0;
    uint32_t _accelerationSteps = 0;
    uint32_t _decelerationSteps = 0;
    float _maxFrequencyHz = 1.0f;
    float _accelerationHzPerSec = 1.0f;
    bool _triangular = false;

    float calculateFrequencyForStep(uint32_t phaseStep) const;
};
