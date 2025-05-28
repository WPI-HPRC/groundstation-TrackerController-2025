#include <Arduino.h>

// CHOOSE DEPLOY PLATFORM BY SELECTING ENVIRONMENT IN PLATFORMIO

////////////////////////////////////////////////////////////////////// Includes //////////////////////////////////////////////////////////////////////
 
// platform specific defines
#ifdef REAL
  #include "Constants/ConstantsReal.h"
#endif
#ifdef SUBSCALE
  #include "Constants/ConstantsSubscale.h"
#endif
#ifdef SIM
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
// #include "StreamInterface.h"

// temporary serial interface code for tuning / bringup
#include "TunerInterface.h"

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
TunerInterface tuner(&SerialUSB);

TeensyTimerTool::PeriodicTimer debugPrintTimer;
TeensyTimerTool::PeriodicTimer blinkTimer;

////////////////////////////////////////////////////////////////////// Local Function Declarations //////////////////////////////////////////////////////////////////////

void debugPrint();

void configureHardware(); // provides pin mappings and tuning parameters to objects

// tuner application related functions
void runTuner();
void calculateVelAccel(float actualPos);
void sendTunerData(float desiredPos, float actualPos, float desiredVel, float actualVelocity, float desiredAcc, float actualAccel);

////////////////////////////////////////////////////////////////////// setup() //////////////////////////////////////////////////////////////////////

void setup() 
{
  SerialUSB.begin(115200);
  while(!SerialUSB){} // wait for connection

  configureHardware(); // setup pins and tuning parameters for controllers

  debugPrintTimer.begin(debugPrint, 100000); // 100000 in µs = 100ms = 0.1s

  blinkTimer.begin([]{digitalToggle(LED_POLARIS);}, 1000000); // 1000000 in us = 1s blink

  // start actual things
  azimuthController.begin();  
  elevationController.begin();

  // start LEDS
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_POLARIS, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_POLARIS, LOW);

  // imu.begin();

  // actual begin code
  delay(10);
  // azimuthMotorDriver.setVelocityCommand(-1);
  // elevationMotorDriver.setVelocityCommand(2);
  // azimuthController.homeController();
  // elevationController.homeController();
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////

// as everything is run through timers, loop goes unused
void loop() 
{
}


////////////////////////////////////////////////////////////////////// Local Function Definitions //////////////////////////////////////////////////////////////////////

void debugPrint()
{
  // azimuthController.debugPrint(&SerialUSB);
  // elevationController.debugPrint(&SerialUSB);
  // azimuthSensor->debugPrint(&SerialUSB);
  elevationSensor->update();
  elevationSensor->debugPrint(&SerialUSB);
}


void configureHardware()
{
  // configure azimuth hardware
  azimuthSensor->setSensorPins(azimuthEncoderA, azimuthEncoderB, azimuthLimitSwitch);
  azimuthSensor->setPhysicalConversionConstant(azimuthConversionRatio);
  azimuthMotorDriver.setPins(azimuthDirection, azimuthStep);
  azimuthMotorDriver.setPhysicalConstants(DegreesPerStepAzimuth, microStepResolution);

  // configure azimuth motion controller
  azimuthController.setPhysicalLimits(azimuthMaxVelocity, azimuthMaxAcceleration);
  azimuthController.setTuningParameters(azimuthFF, azimuthkP, azimuthkD, 0.0, azimuthAcceptableError, azimuthAcceptableVelocityError);
  azimuthController.setLoopTimeStep(controlLoopTimeStep);

  // configure elevation hardware
  elevationSensor->setSensorPins(elevationPotentiometer);
  elevationSensor->setPhysicalConversionConstant(elevationConversionRatio);
  elevationSensor->setZero(elevationMinimumValue); // we only do this for the elevation, azimuth zeroes itself
  elevationMotorDriver.setPins(elevationDirection, elevationStep, elevationDirection2, elevationStep2);
  elevationMotorDriver.setPhysicalConstants(DegreesPerStepElevation, microStepResolution);

  // configure elevation motion controller
  elevationController.setPhysicalLimits(elevationMaxVelocity, elevationMaxAcceleration);
  elevationController.setTuningParameters(elevationFF, elevationkP, elevationkD, elevationGravityCompFactor, elevationAcceptableError, elevationAcceptableVelocityError);
  elevationController.setLoopTimeStep(controlLoopTimeStep);

}