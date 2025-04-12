#ifndef PotSensor_h
#define PotSensor_h

#include <Arduino.h>

#include "Sensor.h"

class PotSensor : public Sensor
{
    public:
        PotSensor() = default;

        void setSensorPins(uint8_t AnalogPin) override
            { analogPin = AnalogPin; };
        void setSensorPins(uint8_t DO, uint8_t NOT, uint8_t USE) override { }; // DON'T USE, DOESN'T WORK FOR POT

        uint8_t begin() override
        {
            /*
            by the below link, setting the pinmode to INPUT_DISABLE puts it in the ideal high impedience state for analog input
            pinMode normally should not be used for analog inputs, 
            as setting something like INPUT or INPUT_PULLUP plays with the physical chip and doesn't present the ideal high input impedience
            https://forum.pjrc.com/index.php?threads/analog-input-impedance-and-pull-up.34319/
            come to think of it, this explains a lot about issues i've(ndebruin) had on previous projects lol (ESP32 but it's the same concept)
            */
            
            pinMode(analogPin, INPUT_DISABLE); 
            
            analogReadResolution(10); // 0-1023, we are specifiying it to be safe
            return 0;
        };

        // returns 0 in all cases
        uint8_t update() override
        {
            currentPos = analogRead(analogPin);
            return 0;
        }

    private:
        uint8_t analogPin;
};

#endif // PotSensor_h