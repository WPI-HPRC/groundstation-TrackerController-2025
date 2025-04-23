#pragma once

#include <Arduino.h>
#include <TeensyTimerTool.h>

#include "Sensor.h"

#include "StepDriver.h"

#include "sCurveProfiler.h"

enum HoldBehavior {coastMode, brakeMode};

class AxisController
{
    public:
        AxisController(StepDriver* StepperDriver = nullptr, uint8_t EnablePin = -1, Sensor* PositionalSensor = nullptr)
            { sensor = PositionalSensor; driver = StepperDriver; enablePin = EnablePin; };

        // set physical limits for this axis. units are degrees per second (seconds squared and cubed where appropriate)
        void setPhysicalLimits(float MaxVelocityLimit, float maxAccelerationLimit, float maxJerkLimit)
            { motionProfiler.setLimits(MaxVelocityLimit, maxAccelerationLimit, maxJerkLimit); maxVelocityLimit = MaxVelocityLimit; };

        // kP, kD, and gravity feedforward compensation are all unitless. acceptable error is in degrees.
        void setTuningParameters(float _kP, float _kD, float gravityFeedFowardCompensation, float AcceptableError)
            { kP = _kP; kD = _kD; gravFFComp = gravityFeedFowardCompensation; acceptableError = AcceptableError; };

        // set how often we want the loop to run. 
        // actually updating the control loop will be handled internally with a timer, to ensure accurate control loop timing.
        // units are microseconds (us)
        void setLoopTimeStep(float timeStepUs)
        { 
            timeStep = timeStepUs;
            // controlLoopTimer.setPeriod(timeStep);
            // controlLoopTimer.setNextPeriod(timeStep); // why do we have to do it twice?? idk, library quirk
        };

        void updateLoop()
        {
            sensor->update(); // run an update loop for our sensor so we chilling on position
            motionProfiler.update(timeStep); // update our motion profiler so we can get the new desired pos/vel data

            float error = motionProfiler.getDesiredPosition() - sensor->getDistFrom0();
            float targetVel = motionProfiler.getDesiredVelocity();

            // generate our commanded velocity
            float velocityCommand = (kP * error) + (kD * targetVel) + getGravityFF();

            // constrain it to be within our maximum allowable velocity
            velocityCommand = constrain(velocityCommand, -maxVelocityLimit, maxVelocityLimit);
            if(enabled){
                driver->setVelocityCommand(velocityCommand);
            }

            reachedGoal = ( fabs(error) < acceptableError);
        };

        uint8_t begin()
        {
            sensor->begin();
            driver->begin();
            pinMode(enablePin, OUTPUT);
            setHoldBehavior(HoldBehavior::coastMode); // begin in a disabled state for on-boot calibration
            applyHoldBehavior(getHoldBehavior());            
            
            controlLoopTimer.begin([this]() { this->updateLoop(); }, timeStep, false); // in µs

            
            return 0;
        };

        void setHoldBehavior(HoldBehavior behavior) { desiredHoldBehavior = behavior; };

        HoldBehavior getHoldBehavior() { return desiredHoldBehavior; };

        // this will take control over the axis using the controller. 
        // this bypasses the hold behavior set by setHoldBehavior()
        // it will act similar to the brakeMode hold behavior, but will actively maintain position.
        void startController()
        {
            enabled = true; // set this safety value on as the very first thing
            // we need to make sure that the controller is enabled ("brake mode") before sending steps
            applyHoldBehavior(HoldBehavior::brakeMode);
            controlLoopTimer.start();
        };

        // this will stop the controller as soon as is feasible, while following the physical limits and deaccelerating smoothly
        // it will then apply the desired hold behavior
        // to work correctly, this function should be called repeatidly until it returns true
        bool smoothStopController()
        {
            // update to have our goal be to stop in the current position (there will be some overshoot with this method but that's fine)
            setTarget(sensor->getDistFrom0());
            
            if(atPosition()){
                driver->setVelocityCommand(0.0, true); // ensure that we stop sending step pulses before doing anything
                applyHoldBehavior(desiredHoldBehavior);
                enabled = false; // set this safety value off as the very last thing
                return true;
            }
            return false;
        };

        // this will stop the controller ASAP, with no regard for the physical limits.
        // should only be used in an emergency.
        // USING THIS COULD RESULT IN MECHANICAL BREAKAGE
        // this will assume usage is for the safety of people, and therefore will try to hold position( "brake mode" )
        void eStopController()
        {
            controlLoopTimer.stop(); // stops trying to command
            applyHoldBehavior(HoldBehavior::brakeMode); // we set this twice just to be extra safe that it won't drop
            driver->setVelocityCommand(0.0, true); // stop the motor
            applyHoldBehavior(HoldBehavior::brakeMode);
            enabled = false; // set this safety value off as the very last thing
        };

        bool controllerEnabled() { return enabled; };

        bool atPosition() { return reachedGoal; };

        void setTarget(float target)
        {
            goalPosition = target;
            motionProfiler.setTarget(goalPosition);
        }

        // temporarily public for tuning
        sCurveProfiler motionProfiler = sCurveProfiler();
    private:
        // physical hardware interfaces
        Sensor *sensor = nullptr;
        StepDriver *driver = nullptr;
        uint8_t enablePin = -1;

        // sCurveProfiler motionProfiler = sCurveProfiler();

        TeensyTimerTool::PeriodicTimer controlLoopTimer;

        bool reachedGoal = false;

        float timeStep; // microseconds (us)

        HoldBehavior desiredHoldBehavior = HoldBehavior::brakeMode;

        bool enabled;

        // tuning parameters
        float kP;
        float kI; // not used / implemented, just in here for possible future use
        float kD;
        float gravFFComp;

        float maxVelocityLimit;

        float acceptableError;

        float goalPosition;

        // gravity feedfoward
        float getGravityFF()
            { return gravFFComp * sin(sensor->getDistFrom0() * DEG_TO_RAD); };

        void applyHoldBehavior(HoldBehavior holdBehavior)
        {
            if(holdBehavior == brakeMode){
                // this enables the driver, which means it runs current through the windings even if it isn't getting steps
                // this acts very similar to "brake mode" on brushed or brushless motor drivers, hence the naming
                digitalWrite(enablePin, HIGH); 
            }
            else if(holdBehavior == coastMode){
                // this disables the driver, which means it just leaves the windings "disconnected", not shorted or anything
                // this acts very similar to "coast mode" on brushed or brushless motor drivers, hence the naming
                digitalWrite(enablePin, LOW); 
            }
        }
};
