#pragma once

#include <Arduino.h>
#include <TeensyTimerTool.h>

class StepDriver
{
    public:
        StepDriver(){};

        void setPins(uint8_t DirectionPin, uint8_t StepPin) // single driver
            { doubleDriver = false; dirPin = DirectionPin; stepPin = StepPin; };
        void setPins(uint8_t DirectionPin, uint8_t StepPin, uint8_t DirectionPin2, uint8_t StepPin2) // double driver
            { doubleDriver = true; dirPin = DirectionPin; stepPin = StepPin; dirPin2 = DirectionPin2; stepPin2 = StepPin2; };

        void setPhysicalConstants(float degreesPerStep, uint64_t microstepResolution)
            { degPerStep = degreesPerStep; microstepRes = microstepResolution; };

        void begin()
        {
            pinMode(dirPin, OUTPUT);
            pinMode(stepPin, OUTPUT);
            if(doubleDriver){
                pinMode(dirPin2, OUTPUT);
                pinMode(stepPin2, OUTPUT);
            }
        };

        // this normally should not be used, just use setVelocityCommand() instead.
        void setDirection(bool forwards)
        {
            digitalWrite(dirPin, forwards);
            if(doubleDriver){
                digitalWrite(dirPin2, forwards);
            }
        };
        
        // if you wish to set the velocity but not start outputting steps, set the second argument to false
        void setVelocityCommand(float setVelocity, bool startAutomatically = true)
        {
            // Convert velocity command to step frequency
            double stepFreq = abs(setVelocity) / (degPerStep / microstepRes);
            bool dir = setVelocity >= 0;
            setDirection(dir);

            updateFrequency(stepFreq);

            if(startAutomatically){ start(); };
        };

        void start() { timer.start(); };

        void stop() { timer.stop(); };


    private:
        TeensyTimerTool::PeriodicTimer timer;

        bool doubleDriver = false;

        uint8_t dirPin = -1;
        uint8_t stepPin = -1;

        uint8_t dirPin2 = -1;
        uint8_t stepPin2 = -1;

        float degPerStep;
        uint64_t microstepRes;

        volatile bool stepPinState = false;

        void updateFrequency(double frequency)
        {
            if(frequency < 1.0){
                stop();
                return;
            }
            // Convert frequency to period in microseconds
            float periodMicros = 1e6 / frequency / 2; // half period (toggle HIGH/LOW)

            /*
            digitalWriteFast can change a pin's state far faster, 
            at the expense of not doing some safety checking/perfect cross-platform compatibility.
            given this code is targeting a specific platform ( Polaris / Teensy 4.0(micromod) ),
            this is an acceptable tradeoff for better performance
            */

            if(doubleDriver){ // two drivers
                timer.begin([this]() {
                    digitalWriteFast(stepPin, stepPinState); 
                    digitalWriteFast(stepPin2, stepPinState);
                    stepPinState = !stepPinState;
                }, periodMicros, false);
            }
            else{ // one driver
                timer.begin([this]() {
                    digitalWriteFast(stepPin, stepPinState);
                    stepPinState = !stepPinState;
                }, periodMicros, false);
            }
        }
};