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

// serial interface
StreamInterface serialInterface(&SerialUSB, azimuthSensor, elevationSensor, &azimuthController, &elevationController, &imu);

// tuning interface
// TunerInterface tuner(&SerialUSB);

TeensyTimerTool::PeriodicTimer interfaceTimer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer debugPrintTimer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer blinkTimer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer sensorVelocityTimer(TeensyTimerTool::TCK);


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

  debugPrintTimer.begin(debugPrint, 100ms); // thank you std::chrono for readable units
  interfaceTimer.begin(interfaceLoop, 10ms); // 100000 in µs = 100ms = 0.1s

  blinkTimer.begin([]{digitalToggle(LED_POLARIS);}, 1s); // thank you std::chrono for readable units

  sensorVelocityTimer.begin([]{azimuthSensor->updateVelocity();elevationSensor->updateVelocity();}, 100ms); // thank you std::chrono for readable units

  // start actual things
  azimuthController.begin();  
  elevationController.begin();

  // start LEDS
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_POLARIS, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_POLARIS, LOW);

#ifdef REAL // subscale doesn't have an IMU
  imu.begin();
#endif

  // actual begin code
  delay(10);

  // elevationMotorDriver.begin();
  // elevationSensor->begin();

  // pinMode(azimuthEnable, OUTPUT); digitalWrite(azimuthEnable, LOW);
  // pinMode(elevationEnable, OUTPUT); digitalWrite(elevationEnable, LOW);
  
  // azimuthMotorDriver.setVelocityCommand(5);
  // elevationMotorDriver.setVelocityCommand(-100);
  
  // azimuthController.homeController();
  // elevationController.homeController();

  // while(!azimuthSensor->isZeroed()){delay(1);};

  // elevationController.setTarget(30);
  // elevationController.setTarget(90);
  // elevationController.setTarget(60);

  // azimuthController.setTarget(40);
  // azimuthController.setTarget(90);
  // azimuthController.setTarget(10);
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////

// as everything is run through timers, loop goes unused
void loop() 
{
  // this is called implicitly
  // yield(); 
}

////////////////////////////////////////////////////////////////////// Local Function Definitions //////////////////////////////////////////////////////////////////////

void interfaceLoop()
{
  interface();
  // debugPrint();
}

void debugPrint(){
  azimuthController.debugPrint(&SerialUSB);
  // elevationController.debugPrint(&SerialUSB);
  // azimuthSensor->debugPrint(&SerialUSB);
  // elevationSensor->update();
  // elevationSensor->debugPrint(&SerialUSB);
}

void interface(){
  serialInterface.read();
}


void configureHardware()
{
  // configure azimuth hardware
  azimuthSensor->setSensorPins(azimuthEncoderA, azimuthEncoderB, azimuthLimitSwitch);
  azimuthSensor->setPhysicalConversionConstant(azimuthConversionRatio);
  azimuthSensor->setZeroOffset(azimuthZeroOffsetDegrees);
  azimuthMotorDriver.setPins(azimuthDirection, azimuthStep);
  azimuthMotorDriver.setPhysicalConstants(DegreesPerStepAzimuth, microStepResolution);

  // configure azimuth motion controller
  azimuthController.setPhysicalLimits(azimuthMaxVelocity, azimuthMaxAcceleration, azimuthGearRatio);
  azimuthController.setTuningParameters(azimuthFF, azimuthkP, azimuthkI, azimuthkD, 0.0, azimuthAcceptableError, azimuthAcceptableVelocityError, homingVelocity);
  azimuthController.setLoopTimeStep(controlLoopTimeStep);

  // configure elevation hardware
  elevationSensor->setSensorPins(elevationPotentiometer);
  elevationSensor->setPhysicalConversionConstant(elevationConversionRatio);
  elevationSensor->setZero(elevationMinimumValue); // we only do this for the elevation, azimuth zeroes itself
  elevationMotorDriver.setPins(elevationDirection, elevationStep, elevationDirection2, elevationStep2);
  elevationMotorDriver.setPhysicalConstants(DegreesPerStepElevation, microStepResolution);

  // configure elevation motion controller
  elevationController.setPhysicalLimits(elevationMaxVelocity, elevationMaxAcceleration, elevationGearRatio);
  elevationController.setTuningParameters(elevationFF, elevationkP, elevationkI, elevationkD, elevationGravityCompFactor, elevationAcceptableError, elevationAcceptableVelocityError, homingVelocity);
  elevationController.setLoopTimeStep(controlLoopTimeStep);
  elevationController.setLimits(elevationMinimumAngle, elevationMaximumAngle);
}