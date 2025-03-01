#ifndef EncoderSensor_h
#define EncoderSensor_h

#include <Arduino.h>
#include <Encoder.h>

#include "Sensor.h"

class EncoderSensor : Sensor
{
    public:
        EncoderSensor()
        {
            Sensor(); 
        };
        void setPins(uint8_t PinA, uint8_t PinB, uint8_t LimitPin) override
        { 
            pinA = PinA; 
            pinB = PinB; 
            pinLimit = LimitPin; 
        };

        uint8_t begin () override
        {
            encoder = Encoder(pinA, pinB);
            pinMode(pinLimit, INPUT_PULLUP);
        };

        uint8_t update() override
        {
            currentPos = encoder.read() - zeroPos;
            if(digitalRead(pinLimit) == LOW){
                zeroPos = currentPos;
            }
        }
        
        int64_t getDistFrom0() override
        {
            return (currentPos - zeroPos);
        }

    private:
        uint8_t pinA;
        uint8_t pinB;

        uint8_t pinLimit;

        Encoder encoder = Encoder(-1,-1);

};

#endif // EncoderSensor_h