#include "SmoothMotionController.h"


SmoothMotionController::SmoothMotionController(
    SmoothStepGenerator& left,
    SmoothStepGenerator& right,
    float pulsesPerMeter,
    float accelerationHzPerSec
) :
    _left(left),
    _right(right),
    _pulsesPerMeter(pulsesPerMeter),
    _accelerationHzPerSec(accelerationHzPerSec) {
}


void SmoothMotionController::begin() {
    _left.begin();
    _right.begin();
    applyFrequency(1.0f);
}


void SmoothMotionController::startForwardDistance(
    float distanceM,
    float maxFrequencyHz
) {
    emergencyStop();
    _steeringAngularVelocityRadps = 0.0f;

    uint32_t totalSteps = (uint32_t)(
        fabs(distanceM) * _pulsesPerMeter + 0.5f
    );
    if (totalSteps < 1) totalSteps = 1;

    _profile.configure(totalSteps, maxFrequencyHz, _accelerationHzPerSec);
    SmoothMotionProfileState initialState = _profile.calculate(0);

    _left.setDirection(true);
    _right.setDirection(true);
    applyFrequency(initialState.targetFrequencyHz);
    // The profile stops on average wheel steps. Continuous wheel counters allow
    // a steering differential without forcing both wheels to the same distance.
    _left.startContinuous();
    _right.startContinuous();

    _running = true;
    _continuous = false;
    _stopping = false;
    _profileStartStep = 0;
    _profileStepOffset = 0;
    _continuousCruiseStep = 0;
    _phase = initialState.phase;
}


void SmoothMotionController::startForwardContinuous(float maxFrequencyHz) {
    emergencyStop();
    _steeringAngularVelocityRadps = 0.0f;

    uint32_t accelerationSteps = (uint32_t)(
        (maxFrequencyHz * maxFrequencyHz) /
        (2.0f * _accelerationHzPerSec) + 0.5f
    );
    if (accelerationSteps < 1) accelerationSteps = 1;

    uint32_t profileSteps = accelerationSteps * 3 + 1;
    _profile.configure(profileSteps, maxFrequencyHz, _accelerationHzPerSec);
    SmoothMotionProfileState initialState = _profile.calculate(0);

    _left.setDirection(true);
    _right.setDirection(true);
    applyFrequency(initialState.targetFrequencyHz);
    _left.startContinuous();
    _right.startContinuous();

    _running = true;
    _continuous = true;
    _stopping = false;
    _profileStartStep = 0;
    _profileStepOffset = 0;
    _continuousCruiseStep = _profile.getAccelerationSteps();
    _phase = initialState.phase;
}


void SmoothMotionController::stopSmooth() {
    if (!_running) return;

    uint32_t stopSteps = (uint32_t)(
        (_currentFrequencyHz * _currentFrequencyHz) /
        (2.0f * _accelerationHzPerSec) + 0.5f
    );
    if (stopSteps < 1) stopSteps = 1;

    _profile.configure(
        stopSteps * 2 + 1,
        _currentFrequencyHz,
        _accelerationHzPerSec
    );
    _profileStartStep = averageStepCount();
    _profileStepOffset = stopSteps + 1;
    _continuous = false;
    _stopping = true;
}


void SmoothMotionController::emergencyStop() {
    _left.stop();
    _right.stop();
    _running = false;
    _continuous = false;
    _stopping = false;
    _currentFrequencyHz = 0.0f;
    _leftFrequencyHz = 0.0f;
    _rightFrequencyHz = 0.0f;
    _steeringAngularVelocityRadps = 0.0f;
    _phase = SMOOTH_IDLE;
}


void SmoothMotionController::update() {
    if (!_running) return;

    uint32_t currentSteps = averageStepCount();
    uint32_t profileStep =
        currentSteps - _profileStartStep + _profileStepOffset;

    if (_continuous && profileStep > _continuousCruiseStep) {
        profileStep = _continuousCruiseStep;
    }

    SmoothMotionProfileState state = _profile.calculate(profileStep);
    _phase = state.phase;

    if (state.phase == SMOOTH_COMPLETE ||
        (!_left.isRunning() && !_right.isRunning())) {
        finishMotion();
        return;
    }

    applySteeredFrequencies(state.targetFrequencyHz);
}


void SmoothMotionController::setSteeringCorrection(
    float angularVelocityRadps,
    float wheelBaseM
) {
    _steeringAngularVelocityRadps = angularVelocityRadps;
    _wheelBaseM = wheelBaseM;

    if (_running) {
        applySteeredFrequencies(_currentFrequencyHz);
    }
}


bool SmoothMotionController::isRunning() const {
    return _running;
}


float SmoothMotionController::getDistanceMovedM() const {
    return averageStepCount() / _pulsesPerMeter;
}


float SmoothMotionController::getCurrentFrequencyHz() const {
    return _currentFrequencyHz;
}


float SmoothMotionController::getLeftFrequencyHz() const {
    return _leftFrequencyHz;
}


float SmoothMotionController::getRightFrequencyHz() const {
    return _rightFrequencyHz;
}


const char* SmoothMotionController::getPhaseName() const {
    switch (_phase) {
        case SMOOTH_IDLE: return "IDLE";
        case SMOOTH_ACCELERATION: return "ACCEL";
        case SMOOTH_CRUISE: return "CRUISE";
        case SMOOTH_DECELERATION: return "DECEL";
        case SMOOTH_COMPLETE: return "COMPLETE";
        default: return "UNKNOWN";
    }
}


uint32_t SmoothMotionController::getStepCount() const {
    return averageStepCount();
}


uint32_t SmoothMotionController::getLeftStepCount() const {
    return _left.getStepCount();
}


uint32_t SmoothMotionController::getRightStepCount() const {
    return _right.getStepCount();
}


uint32_t SmoothMotionController::averageStepCount() const {
    return (_left.getStepCount() + _right.getStepCount()) / 2;
}


void SmoothMotionController::applyFrequency(float frequencyHz) {
    if (frequencyHz < 1.0f) frequencyHz = 1.0f;
    _currentFrequencyHz = frequencyHz;
    _leftFrequencyHz = frequencyHz;
    _rightFrequencyHz = frequencyHz;
    _left.setFrequency(_leftFrequencyHz);
    _right.setFrequency(_rightFrequencyHz);
}


void SmoothMotionController::applySteeredFrequencies(float baseFrequencyHz) {
    if (baseFrequencyHz < 1.0f) baseFrequencyHz = 1.0f;

    float baseMps = baseFrequencyHz / _pulsesPerMeter;
    float halfDifferentialMps =
        _steeringAngularVelocityRadps * _wheelBaseM * 0.5f;
    float leftHz = fabs(baseMps + halfDifferentialMps) * _pulsesPerMeter;
    float rightHz = fabs(baseMps - halfDifferentialMps) * _pulsesPerMeter;

    if (leftHz < 1.0f) leftHz = 1.0f;
    if (rightHz < 1.0f) rightHz = 1.0f;

    bool baseChanged = fabs(baseFrequencyHz - _currentFrequencyHz) >= 0.5f;
    bool leftChanged = fabs(leftHz - _leftFrequencyHz) >= 0.5f;
    bool rightChanged = fabs(rightHz - _rightFrequencyHz) >= 0.5f;
    _currentFrequencyHz = baseFrequencyHz;

    if (baseChanged || leftChanged) {
        _leftFrequencyHz = leftHz;
        _left.setFrequency(leftHz);
    }
    if (baseChanged || rightChanged) {
        _rightFrequencyHz = rightHz;
        _right.setFrequency(rightHz);
    }
}


void SmoothMotionController::finishMotion() {
    _left.stop();
    _right.stop();
    _running = false;
    _continuous = false;
    _stopping = false;
    _currentFrequencyHz = 0.0f;
    _leftFrequencyHz = 0.0f;
    _rightFrequencyHz = 0.0f;
    _steeringAngularVelocityRadps = 0.0f;
    _phase = SMOOTH_COMPLETE;
}
