#ifndef EncoderSensor_h
#define EncoderSensor_h

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
            encoder = Encoder(pinA, pinB);
            pinMode(pinLimit, INPUT_PULLUP);
            return 0;
        };

        // returns 1 if zeroed during loop cycle, 0 otherwise
        uint8_t update() override
        {
            currentPos = encoder.read() - zeroPos;
            if(digitalRead(pinLimit) == LOW){
                zeroPos = currentPos;
                return 1; 
            }

            return 0;
        }

    private:
        uint8_t pinA;
        uint8_t pinB;

        uint8_t pinLimit;

        Encoder encoder = Encoder(-1,-1);

};

#endif // EncoderSensor_h