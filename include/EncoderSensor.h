#pragma once

#include <Arduino.h>
#include <Encoder.h>

#include "Sensor.h"

class EncoderSensor : public Sensor
{
    public:
        EncoderSensor() = default;
        void setSensorPins(uint8_t pin) override {}; // DO NOT USE, WILL NOT BE USED FOR ENCODER
        void setSensorPins(uint8_t PinA, uint8_t PinB, uint8_t LimitPin) override
            { pinA = PinA; pinB = PinB; pinLimit = LimitPin; };

        uint8_t begin() override
        {
            encoder.begin(pinA, pinB);
            pinMode(pinLimit, INPUT_PULLUP);
            zeroed = false;
            return 0;
        };

        // returns 1 if zeroed during loop cycle, 0 otherwise
        uint8_t update() override
        {
            currentPos = encoder.read() - zeroPos;


            bool state = digitalRead(pinLimit);
            if(state != lastState){ // if we aren't in the same thing, then reset
                lastDebounceTime = millis();
            }

            if((millis() - lastDebounceTime) > debounceTime)
            {
                if(state != storedState) // we've actually changed state
                {
                    storedState = state;

                    if(storedState == LOW)
                    {
                        SerialUSB.println("ZEROING");
                        Sensor::setZero(encoder.read());
                        return 1;
                    }
                }
            }
            lastState = state;

            return 0;
        }

        void debugPrint(Stream *printInterface)
        {
            printInterface->print("Current Position: "); printInterface->print(currentPos); printInterface->print(", ");
            printInterface->print("Zero Position: "); printInterface->print(zeroPos); printInterface->print(", ");
            printInterface->print("Encoder Ticks: "); printInterface->print(encoder.read()); printInterface->print(", ");
            printInterface->print("Limit Switch State: "); printInterface->print(digitalRead(pinLimit)); printInterface->print(", ");
            printInterface->print("Debounced State: "); printInterface->print(storedState); printInterface->println("");
        }


    private:
        uint8_t pinA;
        uint8_t pinB;

        uint8_t pinLimit;
        
        uint64_t debounceTime = 10; // ms
        uint64_t lastDebounceTime;
        bool lastState;
        bool storedState;

        Encoder encoder = Encoder();

};