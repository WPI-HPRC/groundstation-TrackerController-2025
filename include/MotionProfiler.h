#pragma once

#include <Arduino.h>

class MotionProfiler
{
    public:
        MotionProfiler(){};
        
        void setLimits(float maxVelocityDPS, float maxAccelDPS2, float maxJerkDPS3)
            { maxVelocityLimit = maxVelocityDPS; maxAccelLimit = maxAccelDPS2; };

        void setTarget(float targetPosDegrees)
            { target = targetPosDegrees; };

        float getDesiredPosition() { return desiredPos; };
        float getDesiredVelocity() { return desiredVel; };

        void update(float timeStep, float currentPosition, float currentVelocity)
        {
            // as this is just a trajectory / motion profile generator, 
            // we don't care about the actual physical position of the system. 
            // we only care about our own internal state vector (desired[Pos,Vel,Accel])
            

        };

    private:
        float maxVelocityLimit; // Degrees per second
        float maxAccelLimit; // degrees per second^2

        float desiredPos = 0; // degrees
        float desiredVel = 0; // degrees per second

        float target; // degrees
};