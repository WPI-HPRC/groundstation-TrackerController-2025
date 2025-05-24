#include <Arduino.h>

// CHOOSE DEPLOY PLATFORM HERE ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define REAL
// #define SUBSCALE
// #define SIM // NOTE: this will be implemented in the future at some point. whether it will be HITL is TBD

////////////////////////////////////////////////////////////////////// Includes //////////////////////////////////////////////////////////////////////
 
// platform specific defines
#ifdef REAL
  #include "Constants/ConstantsReal.h"
#elif SUBSCALE
  #include "Constants/ConstantsSubscale.h"
#elif SIM
  #include "Constants/ConstantsSim.h"
#endif // platform specific defines

// axis controller code
#include "AxisController.h"

// stepper driver code
#include "StepDriver.h"

// positional feedback code
#include "EncoderSensor.h"
#include "PotSensor.h"

// IMU code
#include "IMU.h"

// this handles our interfacing to the outside world
#include "StreamInterface.h"

// temporary serial interface code for tuning / bringup
// #include "TunerInterface.h"

////////////////////////////////////////////////////////////////////// Global Objects //////////////////////////////////////////////////////////////////////

// hardware declerations for azimuth axis
Sensor* azimuthSensor = new EncoderSensor();
StepDriver azimuthMotorDriver;

// motion controller defs for the azimuth axis
AxisController azimuthController(&azimuthMotorDriver, azimuthEnable, azimuthSensor);

// hardware declerations for elevation axis
Sensor* elevationSensor = new PotSensor();
StepDriver elevationMotorDriver;

// motion controller defs for the elevation axis
AxisController elevationController(&elevationMotorDriver, elevationEnable, elevationSensor);

// IMU abstraction object over both chips for accel, gyro & mag
IMU imu;

// tuning interface
// TunerInterface tuner(&SerialUSB);

// serial interface
StreamInterface serialInterface(&SerialUSB, azimuthSensor, elevationSensor, &azimuthController, &elevationController, &imu);

TeensyTimerTool::PeriodicTimer interfaceTimer;
TeensyTimerTool::PeriodicTimer blinkTimer;

////////////////////////////////////////////////////////////////////// Local Function Declarations //////////////////////////////////////////////////////////////////////

void interfaceLoop();

void debugPrint();
void interface();

void configureHardware(); // provides pin mappings and tuning parameters to objects

// tuner application related functions
void runTuner();
void calculateVelAccel(float actualPos);
void sendTunerData(float desiredPos, float actualPos, float desiredVel, float actualVelocity, float desiredAcc, float actualAccel);

////////////////////////////////////////////////////////////////////// setup() //////////////////////////////////////////////////////////////////////

void setup() 
{
  SerialUSB.begin(9600);
  while(!SerialUSB){} // wait for connection

  configureHardware(); // setup pins and tuning parameters for controllers

  interfaceTimer.begin(interfaceLoop, 100000); // 100000 in µs = 100ms = 0.1s

  blinkTimer.begin([]{digitalToggle(LED_POLARIS);}, 1000000); // 1000000 in us = 1s blink

  // start actual things
  azimuthController.begin();  
  elevationController.begin();

  // start LEDS
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_POLARIS, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_POLARIS, LOW);

  imu.begin();

  // actual begin code
  delay(10);
  // azimuthMotorDriver.setVelocityCommand(-1);
  // elevationMotorDriver.setVelocityCommand(2);
  azimuthController.homeController();

  // elevationController.homeController();
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////

// select controller for tuning
AxisController tuningController = azimuthController;
Sensor* tuningSensor = azimuthSensor;
// AxisController tuningController = elevationController;
// Sensor* tuningSensor = elevationSensor;

// as everything is run through timers, loop goes unused
void loop() 
{
}

////////////////////////////////////////////////////////////////////// Local Function Definitions //////////////////////////////////////////////////////////////////////

void interfaceLoop()
{
  // debugPrint();
  interface();
}

void debugPrint(){
  // azimuthController.debugPrint(&SerialUSB);
  // elevationController.debugPrint(&SerialUSB);
  // azimuthSensor->debugPrint(&SerialUSB);
  elevationSensor->update();
  elevationSensor->debugPrint(&SerialUSB);
}

void interface(){
  serialInterface.read();
}


void configureHardware()
{
  // configure azimuth hardware
  azimuthSensor->setSensorPins(azimuthEncoderA, azimuthEncoderB, azimuthLimitSwitch);
  azimuthSensor->setPhysicalConversionConstant(azimuthConversionRatio);
  azimuthMotorDriver.setPins(azimuthDirection, azimuthStep);
  azimuthMotorDriver.setPhysicalConstants(DegreesPerStep, microStepResolution);

  // configure azimuth motion controller
  azimuthController.setPhysicalLimits(azimuthMaxVelocity, azimuthMaxAcceleration, azimuthMaxJerk);
  azimuthController.setTuningParameters(azimuthFF, azimuthkP, azimuthkD, 0.0, azimuthAcceptableError, azimuthAcceptableVelocityError);
  azimuthController.setLoopTimeStep(controlLoopTimeStep);

  // configure elevation hardware
  elevationSensor->setSensorPins(elevationPotentiometer);
  elevationSensor->setPhysicalConversionConstant(elevationConversionRatio);
  elevationSensor->setZero(elevationMinimumValue); // we only do this for the elevation, azimuth zeroes itself
  elevationMotorDriver.setPins(elevationDirection, elevationStep, elevationDirection2, elevationStep2);
  elevationMotorDriver.setPhysicalConstants(DegreesPerStep, microStepResolution);

  // configure elevation motion controller
  elevationController.setPhysicalLimits(elevationMaxVelocity, elevationMaxAcceleration, elevationMaxJerk);
  elevationController.setTuningParameters(elevationFF, elevationkP, elevationkD, elevationGravityCompFactor, elevationAcceptableError, elevationAcceptableVelocityError);
  elevationController.setLoopTimeStep(controlLoopTimeStep);

}


float actualVel;
float actualAcc;
/*void runTuner()
{
  // check for tuner updates
  tuner.readSerial();
  if(tuner.newData()){
    while(!tuningController.smoothStopController()){};
    tuningController.setPhysicalLimits(tuner.getMaxVel(), tuner.getMaxAccel(), tuner.getMaxJerk());
    // tuningController.setTuningParameters(tuner.getKP(), tuner.getKD(), tuner.getGravFF(), tuner.getAcceptableError(), tuner.getAcceptableVelocityError());
    tuningController.setHoldBehavior(tuner.getBrake() ? HoldBehavior::brakeMode : HoldBehavior::coastMode);
    // now that we've applied our new stuff, we can re-enable the controller
    tuningController.startController();
    // now give it a target
    tuningController.setTarget(tuner.getTarget());
  }


  // send data to tuner
  // calculateVelAccel(tuningSensor->getDistFrom0());
  sendTunerData(tuningController.motionProfiler.getDesiredPosition(), tuningSensor->getDistFrom0(),
                tuningController.motionProfiler.getDesiredVelocity(), actualVel,
                tuningController.motionProfiler.getDesiredAcceleration(), actualAcc);
}*/

float prevPos;
float prevVel;
unsigned long prevTime = 0;
void calculateVelAccel(float actualPos)
{
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;  // Time difference in seconds

  // Update actual velocity and acceleration
  actualVel = (actualPos - prevPos) / dt;  // Velocity = (position change) / (time change)
  actualAcc = (actualVel - prevVel) / dt;  // Acceleration = (velocity change) / (time change)

  // Update previous values for next calculation
  prevTime = currentTime;
  prevPos = actualPos;
  prevVel = actualVel;
}

void sendTunerData(float desiredPos, float actualPos, float desiredVel, float actualVelocity, float desiredAcc, float actualAccel)
{
  // Create a formatted string with all the data
  String message = "MOTION_DATA,";
  message += String(desiredPos, 4) + ",";
  message += String(actualPos, 4) + ",";
  message += String(desiredVel, 4) + ",";
  message += String(actualVelocity, 4) + ",";
  message += String(desiredAcc, 4) + ",";
  message += String(actualAccel, 4);

  // Send the message over serial
  Serial.println(message);
}