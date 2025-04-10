#ifndef PotSensor_h
#define PotSensor_h

#include <Arduino.h>
#include <Encoder.h>

#include "Sensor.h"

class PotSensor : Sensor
{
    public:
        PotSensor()
            { Sensor(); };

        void setPins(uint8_t AnalogPin) override
            { analogPin = AnalogPin; };

        uint8_t begin() override
        {
            pinMode(analogPin, INPUT);
            analogReadResolution(10); // 0-1023, we are specifiying it to be safe
            return 0;
        };

        // returns 0 in all cases
        uint8_t update() override
        {
            currentPos = analogRead(analogPin);
        }

    private:
        uint8_t analogPin;
};

#endif // PotSensor_h