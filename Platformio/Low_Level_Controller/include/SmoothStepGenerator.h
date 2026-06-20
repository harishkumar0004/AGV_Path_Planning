#pragma once

#include <Arduino.h>


class SmoothStepGenerator {
public:
    SmoothStepGenerator(
        uint8_t stepPin,
        uint8_t dirPin,
        uint8_t enablePin,
        uint8_t timerIndex,
        bool dirInvert,
        bool enableActiveLow
    );

    void begin();
    void enable();
    void disable();
    void setDirection(bool forward);
    void setFrequency(float frequencyHz);
    void start(uint32_t targetSteps);
    void startContinuous();
    void stop();

    bool isRunning() const;
    uint32_t getStepCount() const;
    float getFrequencyHz() const;

private:
    uint8_t _stepPin;
    uint8_t _dirPin;
    uint8_t _enablePin;
    uint8_t _timerIndex;
    bool _dirInvert;
    bool _enableActiveLow;

    hw_timer_t* _timer = nullptr;
    portMUX_TYPE _timerMux = portMUX_INITIALIZER_UNLOCKED;

    volatile bool _running = false;
    volatile bool _stepPinHigh = false;
    volatile uint32_t _stepCount = 0;
    volatile uint32_t _targetSteps = 0;
    float _frequencyHz = 1.0f;

    void IRAM_ATTR handleInterrupt();
    uint64_t frequencyToHalfPeriodUs(float frequencyHz) const;

    static SmoothStepGenerator* _instances[4];
    static void IRAM_ATTR onTimer0();
    static void IRAM_ATTR onTimer1();
    static void IRAM_ATTR onTimer2();
    static void IRAM_ATTR onTimer3();
};
