#ifndef sCurveProfiler_h
#define sCurveProfiler_h

#include <Arduino.h>

class sCurveProfiler
{
    public:
        sCurveProfiler(){};
        
        void setLimits(float maxVelocityDPS, float maxAccelDPS2, float maxJerkDPS3)
            { maxVelocityLimit = maxVelocityDPS; maxAccelLimit = maxAccelDPS2; maxJerkLimit = maxJerkDPS3; };

        void setTarget(float targetPosDegrees)
            { target = targetPosDegrees; };

        float getDesiredPosition() { return desiredPos; };
        float getDesiredVelocity() { return desiredVel; };
        float getDesiredAcceleration() { return desiredAccel; }; // don't use in most cases, as accel is indirectly controlled via velocity. here for logging / debugging purposes

        void update(float timeStep)
        {
            // as this is just a trajectory / motion profile generator, 
            // we don't care about the actual physical position of the system. 
            // we only care about our own internal state vector (desired[Pos,Vel,Accel])
            
            float error = target - desiredPos;
            int8_t dir = (error >= 0) ? 1 : -1;
            float absError = abs(error);
            float stopDist = (desiredVel * desiredVel) / (2 * maxAccelLimit); // p = 1/2 v^2, hence the factor of 2 in the denominator
            // if it would take us more distance to slow down than our error, start slowing down
            // this profiler should be run with a tight enough timestep to prevent significant overshoot
            bool shouldSlowDown = stopDist >= absError; 

            float desiredJerk = 0;
            // if we're at the point where we need to start slowing down, we want to apply negative jerk 
            // (and by extension negative accel and velocity)
            if(shouldSlowDown){ 
                if(desiredAccel > -maxAccelLimit) // this ensures we aren't already de-accelerating too hard. if we are, this loop cycle we won't apply any additional control
                    desiredJerk = -maxJerkLimit * dir; 
            }
            // this is the acceleration case of above
            else{
                if(desiredAccel < maxAccelLimit) // again, a safety measure to make sure we listen to the maxAccelLimit
                    desiredJerk = maxJerkLimit * dir;
            }

            // incredibly basic riemann sum integration assuming constant value of jerk
            // constant jerk is guaranteed on each loop cycle
            desiredAccel += desiredJerk * timeStep; 
            desiredAccel = constrain(desiredAccel, -maxAccelLimit, maxAccelLimit); // another hard cut to be within our limits

            // we now do the same riemann sum approximation for velocity from accel
            // again, accel is a constant on the timescale of one loop cycle
            desiredVel += desiredAccel * timeStep;
            desiredVel = constrain(desiredVel, -maxVelocityLimit, maxVelocityLimit); // constrain to within our limits

            // riemann approximation, knowing constant velocity on the timestep one more time for position
            desiredPos += desiredVel * timeStep;
        };

    private:
        float maxVelocityLimit; // Degrees per second
        float maxAccelLimit; // degrees per second^2
        float maxJerkLimit; // degrees per second^3

        float desiredPos = 0; // degrees
        float desiredVel = 0; // degrees per second
        float desiredAccel = 0; // degrees per second^2

        float target; // degrees
};

#endif // sCurveProfiler_h