#ifndef AxisController_h
#define AxisController_h

#include <Arduino.h>
#include <TeensyStep.h>

#include "Sensor.h"

class AxisController
{
    public:
        AxisController();
        uint8_t begin();
        uint8_t update();
        
        bool reachedGoal = false;

    private:
        Sensor *sensor;


 
};

#endif // AxisController_h