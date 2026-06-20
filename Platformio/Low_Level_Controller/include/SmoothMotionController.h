#pragma once

#include <Arduino.h>

#include "SmoothMotionProfile.h"
#include "SmoothStepGenerator.h"


class SmoothMotionController {
public:
    SmoothMotionController(
        SmoothStepGenerator& left,
        SmoothStepGenerator& right,
        float pulsesPerMeter,
        float accelerationHzPerSec
    );

    void begin();
    void startForwardDistance(float distanceM, float maxFrequencyHz);
    void startForwardContinuous(float maxFrequencyHz);
    void stopSmooth();
    void emergencyStop();
    void update();

    bool isRunning() const;
    float getDistanceMovedM() const;
    float getCurrentFrequencyHz() const;
    const char* getPhaseName() const;
    uint32_t getStepCount() const;
    uint32_t getLeftStepCount() const;
    uint32_t getRightStepCount() const;

private:
    SmoothStepGenerator& _left;
    SmoothStepGenerator& _right;
    SmoothMotionProfile _profile;
    float _pulsesPerMeter;
    float _accelerationHzPerSec;
    bool _running = false;
    bool _continuous = false;
    bool _stopping = false;
    uint32_t _profileStartStep = 0;
    uint32_t _profileStepOffset = 0;
    uint32_t _continuousCruiseStep = 0;
    float _currentFrequencyHz = 0.0f;
    SmoothMotionPhase _phase = SMOOTH_IDLE;

    uint32_t averageStepCount() const;
    void applyFrequency(float frequencyHz);
    void finishMotion();
};
