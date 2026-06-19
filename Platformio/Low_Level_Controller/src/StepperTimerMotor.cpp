#include "StepperTimerMotor.h"
#include "RobotConfig.h"

StepperTimerMotor::StepperTimerMotor(uint8_t pulPin,
                                     uint8_t dirPin,
                                     uint8_t enaPin,
                                     bool dirInvert,
                                     bool enableActiveLow)
: _pulPin(pulPin),
  _dirPin(dirPin),
  _enaPin(enaPin),
  _dirInvert(dirInvert),
  _enableActiveLow(enableActiveLow) {}

void StepperTimerMotor::begin() {
    pinMode(_pulPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_enaPin, OUTPUT);

    digitalWrite(_pulPin, LOW);
    setDirection(true);
    enable(false);
    stop();
}

void StepperTimerMotor::enable(bool on) {
    _enabled = on;

    if (_enableActiveLow) {
        digitalWrite(_enaPin, on ? LOW : HIGH);
    } else {
        digitalWrite(_enaPin, on ? HIGH : LOW);
    }
}

void StepperTimerMotor::setDirection(bool forward) {
    _directionForward = forward;

    bool pinState = forward;

    if (_dirInvert) {
        pinState = !pinState;
    }

    digitalWrite(_dirPin, pinState ? HIGH : LOW);
}

void StepperTimerMotor::setFrequencyHz(float hz) {
    if (hz <= 0.0f) {
        stop();
        return;
    }

    float maxHz = rpmToHz(MAX_TEST_RPM);
    if (hz > maxHz) {
        hz = maxHz;
    }

    noInterrupts();
    _targetHz = hz;
    _ticksPerStep = hzToTimerTicks(hz);
    _tickCounter = 0;
    _pulseHigh = false;
    digitalWrite(_pulPin, LOW);
    interrupts();

    enable(true);
}

void StepperTimerMotor::stop() {
    noInterrupts();
    _targetHz = 0.0f;
    _ticksPerStep = 0;
    _tickCounter = 0;
    _pulseHigh = false;
    digitalWrite(_pulPin, LOW);
    interrupts();
}

void IRAM_ATTR StepperTimerMotor::isrUpdate() {
    if (!_enabled || _ticksPerStep == 0) {
        return;
    }

    // Keep PUL HIGH for one timer tick, about 20 us.
    if (_pulseHigh) {
        digitalWrite(_pulPin, LOW);
        _pulseHigh = false;
        return;
    }

    _tickCounter++;

    if (_tickCounter >= _ticksPerStep) {
        _tickCounter = 0;

        digitalWrite(_pulPin, HIGH);
        _pulseHigh = true;

        if (_directionForward) {
            _stepCount++;
        } else {
            _stepCount--;
        }
    }
}

long StepperTimerMotor::getStepCount() const {
    return _stepCount;
}

float StepperTimerMotor::getTargetHz() const {
    return _targetHz;
}

bool StepperTimerMotor::getDirectionForward() const {
    return _directionForward;
}