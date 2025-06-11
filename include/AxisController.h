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
        void setPhysicalLimits(float MaxVelocityLimit, float MaxAccelerationLimit, float GearRatio)
            { motionProfiler.setLimits(MaxVelocityLimit, MaxAccelerationLimit); maxVelocityLimit = MaxVelocityLimit; maxAccelerationLimit = MaxAccelerationLimit; gearRatio = GearRatio; };

        // kP, kD, and gravity feedforward compensation are all unitless. acceptable error is in degrees. acceptable velocity error is in deg/s
        void setTuningParameters(float _FF, float _kP, float _kI, float _kD, float gravityFeedFowardCompensation, float AcceptableError, float AcceptableVelocityError, float HomingVelocity)
            { FF = _FF; kP = _kP; kI = _kI; kD = _kD; gravFFComp = gravityFeedFowardCompensation; acceptableError = AcceptableError; acceptableVelocityError = AcceptableVelocityError; homingVelocity = HomingVelocity; };

        // set how often we want the loop to run. 
        // actually updating the control loop will be handled internally with a timer, to ensure accurate control loop timing.
        // units are microseconds (us)
        void setLoopTimeStep(float timeStepUs)
        { 
            timeStep = timeStepUs;
            dt = (1.0/timeStepUs);
            // controlLoopTimer.setPeriod(timeStep);
            // controlLoopTimer.setNextPeriod(timeStep); // why do we have to do it twice?? idk, library quirk
        };

        void setLimits(float MinimumAngle, float MaximumAngle)
        {
            minimumAngle = MinimumAngle;
            maximumAngle = MaximumAngle;
        };

        void updateLoop()
        {
            sensor->update(); // run an update loop for our sensor so we chilling on position

            error = goalPosition - sensor->getDistFrom0();
            velError = goalVelocity - sensor->getVelocity();

            // generate commanded velocity depending on operating state
            switch(state)
            {
                case State::disabled: // not actively driving
                    velocityCommand = 0;
                    break;

                    // this is unused lowkey?
                case State::stopped: // we've reached our goal position, waiting for new position / drift off of position
                    velocityCommand = 0;
                    integralError = 0; // reset integral error if we're stopped and in our position
                    if(fabs(error) > acceptableError){
                        state = State::running;
                    }
                    break;

                case State::homing: // homing state
                    velocityCommand = -homingVelocity;
                    if(sensor->isZeroed()){ state = State::stopped; } // internal state transition to stopped when we have homed
                    break;

                case State::stopping: // stopping the system to be able to disable it

                    // we don't care about positional accuracy, just stopping ASAP
                    // so as long as we reach the goal of ~0 velocity we're fine
                    reachedGoal = reachedVelGoal;
                    if (reachedVelGoal){ state = State::disabled; };
                    
                    [[fallthrough]]; // we intentionally fall through here to continue to run the motion profiler while stopping

                case State::running: // actually running to desired pose

                    // motionProfiler.update(timeStep, sensor->getDistFrom0(), sensor->getVelocity()); // update our motion profiler so we can get the new desired pos/vel data

                    // calculate error from desired position in profile so we can drive our PID controller
                    // error = motionProfiler.getDesiredPosition() - sensor->getDistFrom0();
                    // velError = motionProfiler.getDesiredVelocity() - sensor->getVelocity();

                    // determine direction to apply feedfowards
                    float dir = (error > 0) ? 1.0 : -1.0;

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

                    this is arguably less efficent from a clock cycle PoV, but it's not enough to matter. 
                    (subtraction & multiplication VS just a bunch of comparisions)
                    since our loop time is 10ms, we have plenty of time.
                    */
                    float applyGravFF = (((sensor->getDistFrom0()-90)*dir) < 0) ? 1.0 : 0.0;

                    // calculate integral error
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
                    laymans terms: 
                        if the motor is already running at max speed,
                        we don't want to accumulate integral error bc
                        the motor is already doing as much as it can
                    */
                    // the velocity limits we set are for the output of the system. we need to multiply them by the output of the gear ratio to get the correct limits
                    // bc the velocity command is the velocity of the motor output 
                    bool velocityLimitApplied = true;
                    float clampedVelocityCommand = constrain(velocityCommand, -maxVelocityLimit, maxVelocityLimit);
                    if(clampedVelocityCommand == velocityCommand){
                        velocityLimitApplied = false;
                    }
                    velocityCommand = clampedVelocityCommand;
                    if(velocityLimitApplied){
                        SerialUSB.println("VEL LIMIT APPLIED");
                    }
                    // constrain our velocity command to be within the maximum allowed acceleration
                    /*
                        we first figure out what the change in velocity between loop cycles is
                        constrain that change in velocity to our max acceleration
                        then reapply it to our previously stored velocity command
                    */
                    float deltaV = velocityCommand - prevVelocityCommand;
                    float maxDeltaV = maxAccelerationLimit * gearRatio * 0.01;
                    deltaV = constrain(deltaV, -maxDeltaV, maxDeltaV);
                    velocityCommand = prevVelocityCommand + deltaV;
                    
                    bool accelLimitApplied = true;
                    if(velocityCommand == clampedVelocityCommand){
                        accelLimitApplied = false;
                    }
                    if(accelLimitApplied){
                        SerialUSB.println("Accel limiting! Accel limiting! Accel limiting! Accel limiting! Accel limiting! Accel limiting! Accel limiting! Accel limiting! ");
                    }

                    if(!velocityLimitApplied && !accelLimitApplied && !reachedGoal){
                        // if the motor output isn't being saturated, then integrate the error
                        integralError = tempIntegralError;
                    }
                    
                    reachedPosGoal = ( fabs(error) < acceptableError);
                    reachedVelGoal = ( fabs(velError) < acceptableVelocityError);
                    
                    // we care about both velocity and position for this case
                    reachedGoal = reachedPosGoal && reachedVelGoal;

                    if(reachedPosGoal ){ state = State::stopped; }; 

                    break;
            }

            // constrain our command to be within our maximum allowable velocity
            // the velocity limits we set are for the output of the system. we need to multiply them by the output of the gear ratio to get the correct limits
            // bc the velocity command is the velocity of the motor output 
            velocityCommand = constrain(velocityCommand, -maxVelocityLimit*gearRatio, maxVelocityLimit*gearRatio);

            if(state != State::disabled) // this check is technically unnecessary, but is another safety check
            {
                // make sure that the controller can actually output
                applyHoldBehavior(HoldBehavior::brakeMode);
                // apply our desired value
                float newVelocityCommand = constrain(velocityCommand, -maxVelocityLimit*gearRatio, maxVelocityLimit*gearRatio);
                driver->setVelocityCommand(newVelocityCommand);
            }

            // we only want to apply our hold behavior when we are disabled
            if(state == State::disabled)
            {
                driver->setVelocityCommand(0.0); // safety measure
                applyHoldBehavior(getHoldBehavior()); // make sure we update our hold behavior/enable so we chillin on that
            }

            prevVelocityCommand = velocityCommand;
            prevError = error;
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
            state = State::running; // change to an an enabled mode
        };

        void homeController()
        {
            sensor->unsetZero();
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
            target = constrain(target, minimumAngle, maximumAngle);
            goalPosition = target;
            // motionProfiler.setTarget(goalPosition, sensor->getDistFrom0(), sensor->getVelocity());
            integralError = 0;
            state = State::running;
        };

        void debugPrint(Stream *printInterface)
        {
            printInterface->print("State: "); printInterface->print(getCurrentState()); printInterface->print(", ");
            printInterface->print("Commanded Vel: "); printInterface->print(velocityCommand, 3); printInterface->print(", ");
            printInterface->print("Goal Pos: "); printInterface->print(goalPosition, 3); printInterface->print(", ");
            printInterface->print("Current Pos: "); printInterface->print(sensor->getDistFrom0(), 3); printInterface->print(", ");
            printInterface->print("Current Vel: "); printInterface->print(sensor->getVelocity(), 5); printInterface->print(", ");
            printInterface->print("Error: "); printInterface->print(error, 3); printInterface->print(", ");
            printInterface->print("Velocity Error: "); printInterface->print(velError, 3); printInterface->print(", ");
            printInterface->print("Integral Error: "); printInterface->print(integralError, 3); printInterface->print(", ");
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

        #ifdef REAL
        TeensyTimerTool::PeriodicTimer controlLoopTimer;//(TeensyTimerTool::TCK);
        #else
        TeensyTimerTool::PeriodicTimer controlLoopTimer(TeensyTimerTool::TCK);
        #endif

        bool reachedPosGoal = false;
        bool reachedVelGoal = false;
        bool reachedGoal = false;

        float timeStep; // microseconds (us)
        float dt; 

        float gearRatio; // this is output / input 

        HoldBehavior desiredHoldBehavior = HoldBehavior::brakeMode;

        // tuning parameters
        float FF;
        float kP;
        float kI;
        float kD;
        float gravFFComp;

        float maxVelocityLimit;
        float maxAccelerationLimit;

        float acceptableError;
        float acceptableVelocityError;

        float goalPosition;
        float goalVelocity = 0.0;

        float homingVelocity;
        
        // these are placed here to gain observability into the system
        float error;
        float prevError;
        float velError;
        float integralError;

        float velocityCommand;
        float prevVelocityCommand;

        float minimumAngle = -5000;
        float maximumAngle = 5000;

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
