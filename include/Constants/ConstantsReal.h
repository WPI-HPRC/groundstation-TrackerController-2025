#pragma once

#include <Arduino.h> // removes errors on uints

////////////////////////////////////////////////////////////////////// Tuning Parameters //////////////////////////////////////////////////////////////////////

constexpr float azimuthMaxVelocity      = 10.0; // degrees per second
constexpr float azimuthMaxAcceleration  = 10.0; // degrees per second^2

constexpr float azimuthAcceptableError = 0.0; // degrees
constexpr float azimuthAcceptableVelocityError = 0.0; // degrees per second

constexpr float azimuthFF   = 0.0; // technically units of velocity
constexpr float azimuthkP   = 0.0; // unitless
constexpr float azimuthkD   = 0.0; // unitless


constexpr float elevationMaxVelocity      = 10.0; // degrees per second
constexpr float elevationMaxAcceleration  = 10.0; // degrees per second^2

constexpr float elevationAcceptableError = 0.0; // degrees
constexpr float elevationAcceptableVelocityError = 0.0; // degrees per second

constexpr float elevationFF   = 0.0; // technically units of velocity
constexpr float elevationkP   = 0.0; // unitless
constexpr float elevationkD   = 0.0; // unitless
constexpr float elevationGravityCompFactor = 0.0; // technically not unitless, but determined emperically, not by calculation

constexpr float controlLoopTimeStep = 10000; // us - 10000us = 10ms = 100Hz control loop update rate

////////////////////////////////////////////////////////////////////// Physical Constants //////////////////////////////////////////////////////////////////////

constexpr float azimuthMainGearTeeth = 480; // gear teeth
constexpr float azimuthMotorPinionTeeth = 40; // gear teeth
constexpr float azimuthEncoderPinionTeeth = 40; // gear teeth

constexpr float elevationGearboxReduction = 20; // ratio
constexpr float elevationChainReduction = 3; // ratio

constexpr float elevationTotalReduction = elevationGearboxReduction*elevationChainReduction; // ratio

constexpr int azimuthEncoderActualTicksPerRev = 600; // encoder ticks per rev
constexpr float azimuthEncoderTicksPerRev = azimuthEncoderActualTicksPerRev*4; // quadurature counting means 4 counts per pulse

constexpr int microStepResolution = 40000;
constexpr float DegreesPerStep = 1.8f;
constexpr float DegreesPerStepAzimuth = DegreesPerStep;
constexpr float DegreesPerStepElevation = DegreesPerStep;

////////////////////////////////////////////////////////////////////// Sensor Calibration Values //////////////////////////////////////////////////////////////////////

// must be determined emperically 
constexpr uint16_t elevationMinimumValue = 1; // Old value- 110; // ADC reading value
constexpr uint16_t elevationMaximumValue = 621; // Old value- 826; // ADC reading value

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

constexpr uint8_t azimuthEncoderA = 32;
constexpr uint8_t azimuthEncoderB = 33;

constexpr uint8_t azimuthLimitSwitch = 9;

constexpr uint8_t azimuthStep = 11;
constexpr uint8_t azimuthDirection = 12;
constexpr uint8_t azimuthEnable = 10;

constexpr uint8_t elevationPotentiometer = 17;

constexpr uint8_t elevationStep = 25;
constexpr uint8_t elevationDirection = 24;
constexpr uint8_t elevationEnable = 7;
constexpr uint8_t elevationStep2 = 8;
constexpr uint8_t elevationDirection2 = 20;
