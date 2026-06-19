#pragma once
#include <Arduino.h>

class StepperTimerMotor {
    public:
        StepperTimerMotor(uint8_t pulPin, uint8_t dirPin, 
                         uint8_t enaPin, bool dirInvert, bool enableActiveLow);

        void begin();

        void enable(bool on);
        void setDirection(bool forward);
        void setFrequencyHz(float hz);
        void stop();

        void IRAM_ATTR isrUpdate();


        long getStepCount() const;
        float getTargetHz() const;
        bool getDirectionForward() const;

    private:
        uint8_t _pulPin;
        uint8_t _dirPin;
        uint8_t _enaPin;

        bool _dirInvert;
        bool _enableActiveLow;

        volatile bool _enabled = false;
        volatile bool _directionForward = true;

        volatile bool _pulseHigh = false;
        volatile uint32_t _tickCounter = 0;
        volatile uint32_t _ticksPerStep = 0;

        volatile long _stepCount = 0;

        float _targetHz = 0.0f;
};