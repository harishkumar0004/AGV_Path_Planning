#include "SmoothStepGenerator.h"


SmoothStepGenerator* SmoothStepGenerator::_instances[4] = {
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};


SmoothStepGenerator::SmoothStepGenerator(
    uint8_t stepPin,
    uint8_t dirPin,
    uint8_t enablePin,
    uint8_t timerIndex,
    bool dirInvert,
    bool enableActiveLow
) :
    _stepPin(stepPin),
    _dirPin(dirPin),
    _enablePin(enablePin),
    _timerIndex(timerIndex),
    _dirInvert(dirInvert),
    _enableActiveLow(enableActiveLow) {
}


void SmoothStepGenerator::begin() {
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_enablePin, OUTPUT);

    digitalWrite(_stepPin, LOW);
    setDirection(true);
    disable();

    if (_timerIndex > 3) {
        return;
    }

    _instances[_timerIndex] = this;
    _timer = timerBegin(_timerIndex, 80, true);

    switch (_timerIndex) {
        case 0: timerAttachInterrupt(_timer, &SmoothStepGenerator::onTimer0, true); break;
        case 1: timerAttachInterrupt(_timer, &SmoothStepGenerator::onTimer1, true); break;
        case 2: timerAttachInterrupt(_timer, &SmoothStepGenerator::onTimer2, true); break;
        case 3: timerAttachInterrupt(_timer, &SmoothStepGenerator::onTimer3, true); break;
    }

    timerAlarmWrite(_timer, frequencyToHalfPeriodUs(_frequencyHz), true);
    timerAlarmEnable(_timer);
}


void SmoothStepGenerator::enable() {
    digitalWrite(_enablePin, _enableActiveLow ? LOW : HIGH);
}


void SmoothStepGenerator::disable() {
    digitalWrite(_enablePin, _enableActiveLow ? HIGH : LOW);
}


void SmoothStepGenerator::setDirection(bool forward) {
    bool pinState = _dirInvert ? !forward : forward;
    digitalWrite(_dirPin, pinState ? HIGH : LOW);
}


void SmoothStepGenerator::setFrequency(float frequencyHz) {
    if (frequencyHz < 1.0f) {
        frequencyHz = 1.0f;
    }

    _frequencyHz = frequencyHz;
    if (_timer != nullptr) {
        timerAlarmWrite(_timer, frequencyToHalfPeriodUs(_frequencyHz), true);
    }
}


void SmoothStepGenerator::start(uint32_t targetSteps) {
    portENTER_CRITICAL(&_timerMux);
    _targetSteps = targetSteps;
    _stepCount = 0;
    _running = true;
    _stepPinHigh = false;
    portEXIT_CRITICAL(&_timerMux);

    digitalWrite(_stepPin, LOW);
    enable();
}


void SmoothStepGenerator::startContinuous() {
    start(0);
}


void SmoothStepGenerator::stop() {
    portENTER_CRITICAL(&_timerMux);
    _running = false;
    _stepPinHigh = false;
    portEXIT_CRITICAL(&_timerMux);

    digitalWrite(_stepPin, LOW);
    disable();
}


bool SmoothStepGenerator::isRunning() const {
    return _running;
}


uint32_t SmoothStepGenerator::getStepCount() const {
    return _stepCount;
}


float SmoothStepGenerator::getFrequencyHz() const {
    return _frequencyHz;
}


void IRAM_ATTR SmoothStepGenerator::handleInterrupt() {
    if (!_running) {
        return;
    }

    if (_stepPinHigh) {
        digitalWrite(_stepPin, LOW);
        _stepPinHigh = false;
        return;
    }

    if (_targetSteps > 0 && _stepCount >= _targetSteps) {
        digitalWrite(_stepPin, LOW);
        _running = false;
        return;
    }

    digitalWrite(_stepPin, HIGH);
    _stepPinHigh = true;
    _stepCount++;
}


uint64_t SmoothStepGenerator::frequencyToHalfPeriodUs(float frequencyHz) const {
    if (frequencyHz < 1.0f) {
        frequencyHz = 1.0f;
    }
    uint64_t halfPeriodUs = (uint64_t)(500000.0f / frequencyHz);
    return halfPeriodUs < 1 ? 1 : halfPeriodUs;
}


void IRAM_ATTR SmoothStepGenerator::onTimer0() {
    if (_instances[0] != nullptr) _instances[0]->handleInterrupt();
}

void IRAM_ATTR SmoothStepGenerator::onTimer1() {
    if (_instances[1] != nullptr) _instances[1]->handleInterrupt();
}

void IRAM_ATTR SmoothStepGenerator::onTimer2() {
    if (_instances[2] != nullptr) _instances[2]->handleInterrupt();
}

void IRAM_ATTR SmoothStepGenerator::onTimer3() {
    if (_instances[3] != nullptr) _instances[3]->handleInterrupt();
}
