#pragma once

#include <Arduino.h>
#include <TeensyTimerTool.h>

#include "Sensor.h"

#include "StepDriver.h"

#include "sCurveProfiler.h"

enum HoldBehavior {coastMode, brakeMode};

enum State {disabled, stopped, running, homing, stopping};

class AxisController
{
    public:
        AxisController(StepDriver* StepperDriver = nullptr, uint8_t EnablePin = -1, Sensor* PositionalSensor = nullptr)
            { sensor = PositionalSensor; driver = StepperDriver; enablePin = EnablePin; };

        // set physical limits for this axis. units are degrees per second (seconds squared and cubed where appropriate)
        void setPhysicalLimits(float MaxVelocityLimit, float maxAccelerationLimit, float maxJerkLimit)
            { motionProfiler.setLimits(MaxVelocityLimit, maxAccelerationLimit, maxJerkLimit); maxVelocityLimit = MaxVelocityLimit; };

        // kP, kD, and gravity feedforward compensation are all unitless. acceptable error is in degrees.
        void setTuningParameters(float _FF, float _kP, float _kD, float gravityFeedFowardCompensation, float AcceptableError, float AcceptableVelocityError)
            { FF = _FF; kP = _kP; kD = _kD; gravFFComp = gravityFeedFowardCompensation; acceptableError = AcceptableError; acceptableVelocityError = AcceptableVelocityError; };

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

            // generate commanded velocity depending on operating state
            switch(state)
            {
                case State::disabled:
                    velocityCommand = 0;
                    break;

                case State::stopped:
                    velocityCommand = 0;
                    break;

                case State::homing:
                    velocityCommand = -0.5;
                    if(sensor->isZeroed()){ state = State::stopped; } // internal state transition to stopped when we have homed
                    break;

                case State::stopping:
                    // TO:DO - change lines to relate to velocity

                    if (reachedGoal){ state = State::disabled; };
                    // if(fabs(sensor->getVelocity()) < acceptableVelocityError){ state = State::disabled; }; // internal state transition when we have reached our destination
                    // we don't care about positional accuracy, just stopping ASAP
                    
                    [[fallthrough]]; // we intentionally fall through here to continue to run the motion profiler while stopping

                case State::running:
                    motionProfiler.update(timeStep); // update our motion profiler so we can get the new desired pos/vel data
                    error = motionProfiler.getDesiredPosition() - sensor->getDistFrom0();
                    
                    // TO:DO - have gravity feedforward act only on upward motion
                    
                    velocityCommand = FF + (kP * error) + (kD * motionProfiler.getDesiredVelocity()) + getGravityFF();
                    
                    // TO:DO - have a velocity requirement for us to have reached the goal, not just position

                    reachedGoal = ( fabs(error) < acceptableError); // && (fabs(sensor->getVelocity()) < acceptableVelocityError);
                    if(reachedGoal){ state = State::stopped; }; 
                    break;
            }

            // constrain it to be within our maximum allowable velocity
            velocityCommand = constrain(velocityCommand, -maxVelocityLimit, maxVelocityLimit);
            if(state != State::disabled) // this check is technically unnecessary, but is another safety check
            {
                // make sure that the controller can actually output
                applyHoldBehavior(HoldBehavior::brakeMode);
                driver->setVelocityCommand(velocityCommand);
            }

            // we only want to apply our hold behavior when we are disabled
            if(state == State::disabled)
            {
                driver->setVelocityCommand(0.0); // safety measure
                applyHoldBehavior(getHoldBehavior()); // make sure we update our hold behavior/enable so we chillin on that
            }
        };

        uint8_t begin()
        {
            sensor->begin();
            driver->begin();
            pinMode(enablePin, OUTPUT);
            setHoldBehavior(HoldBehavior::coastMode); // begin in a disabled state for on-boot calibration
            applyHoldBehavior(getHoldBehavior());            
            
            controlLoopTimer.begin([this]() { this->updateLoop(); }, timeStep); // in µs

            return 0;
        };

        void setHoldBehavior(HoldBehavior behavior) { desiredHoldBehavior = behavior; };

        HoldBehavior getHoldBehavior() { return desiredHoldBehavior; };

        // this will take control over the axis using the controller. 
        // this bypasses the hold behavior set by setHoldBehavior()
        // it will act similar to the brakeMode hold behavior, but will actively maintain position.
        void startController()
        {
            state = State::stopped; // change to an an enabled mode
        };

        void homeController()
        {
            state = State::homing;
        }

        // this will stop the controller as soon as is feasible, while following the physical limits and deaccelerating smoothly
        // it will then apply the desired hold behavior
        bool smoothStopController()
        {
            // update to have our goal be to stop in the current position (there will be some overshoot with this method but that's fine)
            setTarget(sensor->getDistFrom0());

            state = State::stopping;            
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
            state = State::disabled; // set this safety value off as the very last thing
        };

        bool controllerEnabled() { return (state != State::disabled); };

        bool isAtPosition() { return reachedGoal; };

        void setTarget(float target)
        {
            goalPosition = target;
            motionProfiler.setTarget(goalPosition);
        };

        void debugPrint(Stream *printInterface)
        {
            printInterface->print("State: "); printInterface->print(getCurrentState()); printInterface->print(", ");
            printInterface->print("Commanded Velocity: "); printInterface->print(velocityCommand, 3); printInterface->print(", ");
            printInterface->print("Goal Position: "); printInterface->print(goalPosition, 3); printInterface->print(", ");
            printInterface->print("Current Position: "); printInterface->print(sensor->getDistFrom0(), 3); printInterface->print(", ");
            printInterface->print("Error: "); printInterface->print(error, 3); printInterface->print(", ");
            printInterface->println("");
        };

        // temporarily public for tuning
        sCurveProfiler motionProfiler = sCurveProfiler();
    private:
        // physical hardware interfaces
        Sensor *sensor = nullptr;
        StepDriver *driver = nullptr;
        uint8_t enablePin = -1;

        // sCurveProfiler motionProfiler = sCurveProfiler();

        State state;

        TeensyTimerTool::PeriodicTimer controlLoopTimer;

        bool reachedGoal = false;

        float timeStep; // microseconds (us)

        HoldBehavior desiredHoldBehavior = HoldBehavior::brakeMode;

        // tuning parameters
        float FF;
        float kP;
        float kI; // not used / implemented, just in here for possible future use
        float kD;
        float gravFFComp;

        float maxVelocityLimit;

        float acceptableError;
        float acceptableVelocityError;

        float goalPosition;
        
        // these are placed here to gain observability into the system
        float error;
        float velocityCommand;

        // gravity feedfoward
        float getGravityFF()
            { return gravFFComp * sin(sensor->getDistFrom0() * DEG_TO_RAD); };

        void applyHoldBehavior(HoldBehavior holdBehavior)
        {
            if(holdBehavior == brakeMode){
                // this enables the driver, which means it runs current through the windings even if it isn't getting steps
                // this acts very similar to "brake mode" on brushed or brushless motor drivers, hence the naming
                digitalWrite(enablePin, LOW); 
            }
            else if(holdBehavior == coastMode){
                // this disables the driver, which means it just leaves the windings "disconnected", not shorted or anything
                // this acts very similar to "coast mode" on brushed or brushless motor drivers, hence the naming
                digitalWrite(enablePin, HIGH); 
            }
        }

        String getCurrentState()
        {
            switch(state)
            {
                case State::disabled:
                    return "Disabled";
                case State::homing:
                    return "Homing";
                case State::running:
                    return "Running";
                case State::stopped:
                    return "Stopped";
                case State::stopping:
                    return "Stopping";
            }
            return "";
        }
};
