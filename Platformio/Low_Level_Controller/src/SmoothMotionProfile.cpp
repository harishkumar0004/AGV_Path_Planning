#include "SmoothMotionProfile.h"


void SmoothMotionProfile::configure(
    uint32_t totalSteps,
    float maxFrequencyHz,
    float accelerationHzPerSec
) {
    _totalSteps = totalSteps;
    _maxFrequencyHz = maxFrequencyHz < 1.0f ? 1.0f : maxFrequencyHz;
    _accelerationHzPerSec =
        accelerationHzPerSec < 1.0f ? 1.0f : accelerationHzPerSec;

    float accelerationDistance =
        (_maxFrequencyHz * _maxFrequencyHz) /
        (2.0f * _accelerationHzPerSec);
    _accelerationSteps = (uint32_t)(accelerationDistance + 0.5f);
    _decelerationSteps = _accelerationSteps;

    if (_accelerationSteps + _decelerationSteps >= _totalSteps) {
        _triangular = true;
        _accelerationSteps = _totalSteps / 2;
        _decelerationSteps = _totalSteps - _accelerationSteps;

        float peakFrequency = sqrtf(
            2.0f * _accelerationHzPerSec * _accelerationSteps
        );
        if (peakFrequency >= 1.0f) {
            _maxFrequencyHz = peakFrequency;
        }
        return;
    }

    _triangular = false;
}


SmoothMotionProfileState SmoothMotionProfile::calculate(
    uint32_t currentStepCount
) const {
    SmoothMotionProfileState state;
    state.currentSteps = currentStepCount;
    state.remainingSteps = currentStepCount >= _totalSteps
        ? 0
        : _totalSteps - currentStepCount;

    if (_totalSteps == 0) {
        state.phase = SMOOTH_IDLE;
        return state;
    }

    if (currentStepCount >= _totalSteps) {
        state.phase = SMOOTH_COMPLETE;
        return state;
    }

    if (currentStepCount < _accelerationSteps) {
        state.phase = SMOOTH_ACCELERATION;
        state.targetFrequencyHz = calculateFrequencyForStep(currentStepCount + 1);
        return state;
    }

    if (currentStepCount >= _totalSteps - _decelerationSteps) {
        state.phase = SMOOTH_DECELERATION;
        state.targetFrequencyHz = calculateFrequencyForStep(state.remainingSteps);
        return state;
    }

    state.phase = SMOOTH_CRUISE;
    state.targetFrequencyHz = _maxFrequencyHz;
    return state;
}


bool SmoothMotionProfile::isTriangular() const {
    return _triangular;
}


uint32_t SmoothMotionProfile::getTotalSteps() const {
    return _totalSteps;
}


uint32_t SmoothMotionProfile::getAccelerationSteps() const {
    return _accelerationSteps;
}


float SmoothMotionProfile::calculateFrequencyForStep(uint32_t phaseStep) const {
    float frequency = sqrtf(
        2.0f * _accelerationHzPerSec * phaseStep
    );
    if (frequency < 1.0f) return 1.0f;
    if (frequency > _maxFrequencyHz) return _maxFrequencyHz;
    return frequency;
}
