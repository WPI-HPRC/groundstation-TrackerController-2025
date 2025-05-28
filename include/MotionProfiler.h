#pragma once

#include <Arduino.h>

class MotionProfiler
{
    public:
        MotionProfiler(){};
        
        void setLimits(float maxVelocityDPS, float maxAccelDPS2)
            { maxVelocityLimit = maxVelocityDPS; maxAccelLimit = maxAccelDPS2; };

        void setTarget(float targetPosDegrees, float InitialPos, float InitialVel)
            { targetPos = targetPosDegrees; initialPos = InitialPos; initialVel = InitialVel; };

        float getTarget(){return targetPos; };
        float getDesiredPosition() { return desiredPos; };
        float getDesiredVelocity() { return desiredVel; };
        float getDesiredAccel() { return desiredAccel; };

        void update(float timeStep, float currentPos, float currentVel)
        {
            float posError = targetPos - currentPos;
            float direction = (posError > 0) ? 1.0 : -1.0;
            
            // determine the minimum stopping distance given our current velocity
            float stoppingDist = (fabs(currentVel) * fabs(currentVel)) / (2.0 * maxAccelLimit);

            desiredAccel = 0; // this is the "impulse" we will apply to the current velocity

            if ((currentVel * posError) < 0.0){
                // we're moving away from the goal positon, we need to deaccelerate asap
                desiredAccel = -maxAccelLimit * ((currentVel > 0 ) ? 1 : -1);                
            }
            else if (fabs(posError) < stoppingDist){
                // we've reached the point where we need to begin deaccelerating to land on our position
                desiredAccel = -maxAccelLimit * direction;
            }
            else if (fabs(currentVel) < maxVelocityLimit){
                // we're still moving towards our goal and have room to accelerate
                desiredAccel = maxAccelLimit * direction;
            }
            else{
                // don't make any changes
                desiredAccel = 0; 
            }
            
            // integrate current velocity with our desired accel
            desiredVel = currentVel + (desiredAccel * timeStep);

            // integrate to position
            desiredPos = currentPos + currentVel * timeStep;

            // should already be constrained, but just to be safe
            desiredVel = constrain(desiredVel, -maxVelocityLimit, maxVelocityLimit);         
        };

    private:
        float maxVelocityLimit; // Degrees per second
        float maxAccelLimit; // degrees per second^2

        float desiredPos = 0; // degrees
        float desiredVel = 0; // degrees per second
        float desiredAccel = 0; // degrees per second^2

        float targetPos; // degrees

        float initialPos; // degrees
        float initialVel; // degrees per second
};