#pragma once

////////////////////////////////////////////////////////////////////// Tuning Parameters //////////////////////////////////////////////////////////////////////

const float azimuthMaxVelocity      = 0.0; // degrees per second
const float azimuthMaxAcceleration  = 0.0; // degrees per second^2
const float azimuthMaxJerk          = 0.0; // degrees per second^3

const float azimuthAcceptableError = 0.0; // degrees

const float azimuthkP   = 0.0;
const float azimuthkD   = 0.0;


const float elevationMaxVelocity      = 0.0; // degrees per second
const float elevationMaxAcceleration  = 0.0; // degrees per second^2
const float elevationMaxJerk          = 0.0; // degrees per second^3

const float elevationAcceptableError = 0.0; // degrees

const float elevationkP   = 0.0;
const float elevationkD   = 0.0;
const float elevationGravityCompFactor = 0.0;


const float timeStep = 10000; // 10000us = 10ms = 100Hz loop update rate

////////////////////////////////////////////////////////////////////// Physical Constants //////////////////////////////////////////////////////////////////////

const int azimuthMainGearTeeth = 480; // gear teeth
const int azimuthMotorPinionTeeth = 40; // gear teeth
const int azimuthEncoderPinionTeeth = 40; // gear teeth

const int elevationGearboxReduction = 20; // ratio
const int elevationChainReduction = 3; // ratio

const int elevationTotalReduction = elevationGearboxReduction*elevationChainReduction; // ratio

const int azimuthEncoderActualTicksPerRev = 600; // encoder ticks per rev
const int azimuthEncoderTicksPerRev = azimuthEncoderActualTicksPerRev*4; // quadurature counting means 4 counts per pulse

const int microStepResolution = 40000;
const float DegreesPerStep = 1.8f;

////////////////////////////////////////////////////////////////////// Sensor Calibration Values //////////////////////////////////////////////////////////////////////

// must be determined emperically 
const uint16_t elevationMinimumValue = 110; // ADC reading value
const uint16_t elevationMaximumValue = 826; // ADC reading value

// must be determined/defined
constexpr float elevationMinimumAngle = 0.0f; // degrees
constexpr float elevationMaximumAngle = 180.0f; // degrees

// this assumes a linear relationship (which should be true since we bought linear pots)
        // TO:DO - new pot is non-linear. we need to handle that. lookup table?
constexpr double elevationConversionRatio ( (elevationMaximumAngle-elevationMinimumAngle) / (elevationMaximumValue-elevationMinimumValue) ); // adc readings to degrees

// 360 degrees / ticks per revolution * gear ratio
constexpr double azimuthConversionRatio ( (360/azimuthEncoderTicksPerRev) * (azimuthEncoderPinionTeeth / azimuthMainGearTeeth) ); // encoder ticks to degrees


////////////////////////////////////////////////////////////////////// Pins //////////////////////////////////////////////////////////////////////

#define LED_POLARIS 6

const uint8_t azimuthEncoderA = 32;
const uint8_t azimuthEncoderB = 33;

const uint8_t azimuthLimitSwitch = 9;

const uint8_t azimuthStep = 11;
const uint8_t azimuthDirection = 12;
const uint8_t azimuthEnable = 10;

const uint8_t elevationPotentiometer = 17;

const uint8_t elevationStep = 25;
const uint8_t elevationDirection = 24;
const uint8_t elevationStep2 = 8;
const uint8_t elevationDirection2 = 20;
const uint8_t elevationEnable = 7;