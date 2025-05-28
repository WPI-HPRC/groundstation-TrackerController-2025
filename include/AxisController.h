#pragma once

#include <Arduino.h>
#include <TeensyTimerTool.h>

#include "Sensor.h"

#include "StepDriver.h"

#include "MotionProfiler.h"

enum HoldBehavior {coastMode, brakeMode};

enum State {disabled, stopped, running, homing, stopping};

class AxisController
{
    public:
        AxisController(StepDriver* StepperDriver = nullptr, uint8_t EnablePin = -1, Sensor* PositionalSensor = nullptr)
            { sensor = PositionalSensor; driver = StepperDriver; enablePin = EnablePin; };

        // set physical limits for this axis. units are degrees per second (seconds squared where appropriate)
        void setPhysicalLimits(float MaxVelocityLimit, float maxAccelerationLimit)
            { motionProfiler.setLimits(MaxVelocityLimit, maxAccelerationLimit); maxVelocityLimit = MaxVelocityLimit; };

        // kP, kD, and gravity feedforward compensation are all unitless. acceptable error is in degrees. acceptable velocity error is in deg/s
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
            sensor->update(); // run an update loop for our sensor so we chilling on position and velocity

            // generate commanded velocity depending on operating state
            switch(state)
            {
                case State::disabled: // not actively driving
                    velocityCommand = 0;
                    break;

                case State::stopped: // we've reached our goal position, waiting for new position / drift off of position
                    velocityCommand = 0;
                    break;

                case State::homing: // homing state
                    velocityCommand = -0.5;
                    if(sensor->isZeroed()){ state = State::stopped; } // internal state transition to stopped when we have homed
                    break;

                case State::stopping: // stopping the system to be able to disable it

                    // we don't care about positional accuracy, just stopping ASAP
                    // so as long as we reach the goal of ~0 velocity we're fine
                    reachedGoal = reachedVelGoal;
                    if (reachedVelGoal){ state = State::disabled; };
                    
                    
                    [[fallthrough]]; // we intentionally fall through here to continue to run the motion profiler while stopping

                case State::running: // actually running to desired pose

                    motionProfiler.update(timeStep, sensor->getDistFrom0(), sensor->getVelocity()); // update our motion profiler so we can get the new desired pos/vel data

                    // calculate error from desired position in profile so we can drive our PID controller
                    error = motionProfiler.getDesiredPosition() - sensor->getDistFrom0();
                    velError = motionProfiler.getDesiredVelocity() - sensor->getVelocity();
                    
                    // velocityCommand = FF + (kP * error) + (kD * motionProfiler.getDesiredVelocity()) + getGravityFF();

                    // determine direction to apply feedfowards
                    float dir = (motionProfiler.getDesiredVelocity() > 0) ? 1.0 : -1.0;

                    /*
                    we only want to apply the gravity feedfoward on upwards motion 
                    (which is when we're actually fighting gravity)
                    the two cases where this is true are:
                        position < 90 AND direction > 0 (forward motion)
                        position > 90 AND direction < 0 (backward motion)
                    this could be some complex boolean expression, 
                    or could be some rather clean math

                    (position-90)   is negative when velocity should be positive
                                    and is positive when velocity should be negative

                    in both cases, the product should be negative
                    so when the product is negative, we apply the gravity feedfoward
                    
                    ((position < 90) && (dir > 0)) ||  ((position > 90) && (dir < 0))
                    === (functionally equivalent to)
                    ((position-90)*dir) < 0
                    */
                    float applyGravFF = (((sensor->getDistFrom0()-90)*dir) < 0) ? 1.0 : 0.0;

                    // calculate integral error
                    double dt = (1.0/timeStep);
                    float tempIntegralError = integralError + error * dt;

                    velocityCommand = 
                                    (FF * dir) + // dynamic friction feedfoward
                                    (kP * error) + // proportional controller (on position)
                                    (kI * integralError) + // integral controller (on position * time)
                                    (kD * velError) + // derivative controller (on velocity)
                                    (getGravityFF() * dir * applyGravFF) // gravity feedfoward
                                    ;
            
                    /*
                    this is an anti-windup measure that prevents integration if the actuator is saturated
                    tl:dr: 
                        if the motor is already running at max speed,
                        we don't want to accumulate integral error bc
                        the motor is already doing as much as it can
                    */
                    float clampedVelocityCommand = constrain(velocityCommand, -maxVelocityLimit, maxVelocityLimit);
                    if(clampedVelocityCommand == velocityCommand){
                        // if the motor output isn't being saturated, then integrate the error
                        integralError = tempIntegralError;
                    }
                    
                    // perform a different error calculation to our actual goal pose for these
                    reachedPosGoal = ( fabs(goalPosition - sensor->getDistFrom0()) < acceptableError);
                    reachedVelGoal = ( fabs(goalVelocity - sensor->getVelocity()) < acceptableVelocityError);
                    
                    // we care about both velocity and position for this case
                    reachedGoal = reachedPosGoal && reachedVelGoal;

                    if(reachedPosGoal && reachedVelGoal){ state = State::stopped; }; 

                    break;
            }

            // constrain our command to be within our maximum allowable velocity
            velocityCommand = constrain(velocityCommand, -maxVelocityLimit, maxVelocityLimit);

            if(state != State::disabled) // this check is technically unnecessary, but is another safety check
            {
                // make sure that the controller can actually output
                applyHoldBehavior(HoldBehavior::brakeMode);
                // apply our desired value
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
            // every value is set twice to be extra safe
            state = State::disabled;
            applyHoldBehavior(HoldBehavior::brakeMode);
            driver->setVelocityCommand(0.0, true); // stop the motor
            applyHoldBehavior(HoldBehavior::brakeMode);
            state = State::disabled;
            driver->setVelocityCommand(0.0, true);
        };

        bool controllerEnabled() { return (state != State::disabled); };

        bool isAtPosition() { return reachedPosGoal; };
        bool isAtVelocity() { return reachedVelGoal; };
        bool isAtGoal() { return reachedGoal; };

        void setTarget(float target)
        {
            goalPosition = target;
            motionProfiler.setTarget(goalPosition, sensor->getDistFrom0(), sensor->getVelocity());
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

        // temporarily public for debugging
        MotionProfiler motionProfiler = MotionProfiler();
    private:
        // physical hardware interfaces
        Sensor *sensor = nullptr;
        StepDriver *driver = nullptr;
        uint8_t enablePin = -1;

        State state = State::disabled;

        TeensyTimerTool::PeriodicTimer controlLoopTimer;

        bool reachedPosGoal = false;
        bool reachedVelGoal = false;
        bool reachedGoal = false;

        float timeStep; // microseconds (us)

        HoldBehavior desiredHoldBehavior = HoldBehavior::brakeMode;

        // tuning parameters
        float FF;
        float kP;
        float kI;
        float kD;
        float gravFFComp;

        float maxVelocityLimit;

        float acceptableError;
        float acceptableVelocityError;

        float goalPosition;
        float goalVelocity = 0.0;
        
        // these are placed here to gain observability into the system
        float error;
        float velError;
        float integralError;

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
