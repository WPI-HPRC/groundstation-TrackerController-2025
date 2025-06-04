#pragma once

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
            as setting something like INPUT or INPUT_PULLUP plays with the physical chip and doesn't present the ideal high Z state
            https://forum.pjrc.com/index.php?threads/analog-input-impedance-and-pull-up.34319/
            come to think of it, this explains a lot about issues i've(ndebruin) had on previous projects lol (ESP32 but it's the same concept)
            */
            
            pinMode(analogPin, INPUT_DISABLE); 
            
            analogReadResolution(10); // 0-1023, we are specifiying it to be safe
            update(); // seed initial value
            zeroed = true;
            
            return 0;
        };

        void debugPrint(Stream *printInterface)
        {
            printInterface->print("Current Position: "); printInterface->print(getDistFrom0()); printInterface->print(", "); 
            printInterface->print("Current Velocity: "); printInterface->print(currentVel, 20); printInterface->print(", "); 
            printInterface->print("Pot reading: "); printInterface->print(currentPos); printInterface->println();
        }

        void updateVelocity() override
        {
            // update velocity
                // convert to actual degrees
            float currentRealPos = ( (currentPos - zeroPos ) * conversionConstant );
            float lastRealPos = ( (lastPos - zeroPos ) * conversionConstant );

            // time math
            unsigned long dt = micros() - lastTime;
            lastTime = micros();

            // SerialUSB.println(dt);

            // velocity math
            currentVel = ( (currentRealPos - lastRealPos) / dt ) * (10e4);
            lastPos = currentPos;
        };

        // returns 0 if updated
        // returns 1 if didn't update a noisy reading
        uint8_t update() override
        {
            float newReading = analogRead(analogPin);

            // SerialUSB.println(abs(newReading-currentPos));
            if(abs(newReading-currentPos) < 3){
                return 1;
            }
            currentPos = newReading;
            
            
            // currentPos = analogRead(analogPin);

            // updateVelocity();

            return 0;
        }

        float getDistFrom0() override
            { return (currentPos - zeroPos) * conversionConstant; }; // sensors should use their raw value internally, and we only convert to the desired unit with the conversion constant before we give it to the user

        float getVelocity() override
            { return currentVel; };

    private:
        uint8_t analogPin;

        int64_t minimumValue;
};