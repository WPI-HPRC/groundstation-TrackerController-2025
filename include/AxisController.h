#ifndef AxisController_h
#define AxisController_h

#include <Arduino.h>

#include "Sensor.h"

#include "StepDriver.h"

#include "sCurveProfiler.h"

class AxisController
{
    public:
        AxisController(uint8_t StepPin=-1, uint8_t DirPin=-1, bool SecondDirPinWanted=false, Sensor* PositionalSensor = nullptr)
        {
            sensor = PositionalSensor;
            stepPin = StepPin;
            dirPin = DirPin;
            usingSecondDir = SecondDirPinWanted;
            if(SecondDirPinWanted){
                secondDirPin = DirPin + 1;
            }
            else{
                secondDirPin = -1;
            }
        };

        uint8_t begin()
        {
            sensor->begin();
            return 0;
        };
        
        uint8_t update()
        {
            return 0;
        };

        void setTarget()
        {

        };

        bool hasReachedGoal()
        {
            return reachedGoal;
        };

    private:
        Sensor *sensor = nullptr;

        uint8_t stepPin = -1;
        uint8_t dirPin = -1;
        uint8_t secondDirPin = -1;
        bool usingSecondDir = false;

        bool reachedGoal = false;
};

#endif // AxisController_h

