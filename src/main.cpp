#include <Arduino.h>

// CHOOSE DEPLOY PLATFORM HERE ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define REAL
// #define SUBSCALE
// #define SIM // NOTE: this will be implemented in the future at some point. whether it will be HITL is TBD

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

// positional feedback code
#include "EncoderSensor.h"

// this handles our interfacing to the outside world
#include "StreamInterface.h"



// hardware declerations for azimuth axis
// Sensor encoderSensor = EncoderSensor();
// encoderSensor.setPins(azimuthEncoderA, azimuthEncoderB, azimuthLimitSwitch);
    // TO:DO fix "inaccessible base"


void setup() {
}

void loop() {
  // put your main code here, to run repeatedly:
}
