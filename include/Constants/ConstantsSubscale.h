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

constexpr float azimuthMainGearTeeth = 110; // gear teeth
constexpr float azimuthMotorPinionTeeth = 20; // gear teeth
constexpr float azimuthEncoderPinionTeeth = 20; // gear teeth

constexpr float elevationMainGearTeeth = 96; // ratio
constexpr float elevationMotorPinion = 12; // ratio

constexpr int azimuthEncoderActualTicksPerRev = 20; // encoder ticks per rev
constexpr float azimuthEncoderTicksPerRev = azimuthEncoderActualTicksPerRev*4; // quadurature counting means 4 counts per pulse

constexpr int microStepResolution = 16;
constexpr float DegreesPerStepAzimuth = 1.8f;
constexpr float DegreesPerStepElevation = 7.5f;

////////////////////////////////////////////////////////////////////// Sensor Calibration Values //////////////////////////////////////////////////////////////////////

// must be determined emperically 
constexpr uint16_t elevationMinimumValue = 1; // ADC reading value
constexpr uint16_t elevationMaximumValue = 2; // ADC reading value

// must be determined/defined
constexpr float elevationMinimumAngle = 0.0f; // degrees
constexpr float elevationMaximumAngle = 180.0f; // degrees

// this assumes a linear relationship (which should be true since we bought linear pots)
        // TO:DO - new pot is non-linear. we need to handle that. lookup table?
constexpr double elevationConversionRatio = ( (elevationMaximumAngle-elevationMinimumAngle) / (elevationMaximumValue-elevationMinimumValue) ); // adc readings to degrees

// 360 degrees / ticks per revolution * gear ratio
constexpr double azimuthConversionRatio ( (360.0/azimuthEncoderTicksPerRev) * (azimuthEncoderPinionTeeth / azimuthMainGearTeeth) ); // encoder ticks to degrees

////////////////////////////////////////////////////////////////////// Pins //////////////////////////////////////////////////////////////////////

#define LED_POLARIS 13

constexpr uint8_t azimuthEncoderA = 15;
constexpr uint8_t azimuthEncoderB = 16;

constexpr uint8_t azimuthLimitSwitch = 0;

constexpr uint8_t azimuthStep = 30;
constexpr uint8_t azimuthDirection = 14;
constexpr uint8_t azimuthEnable = 39;

constexpr uint8_t elevationPotentiometer = 33;

constexpr uint8_t elevationStep = 35;
constexpr uint8_t elevationDirection = 38;
constexpr uint8_t elevationEnable = 36;
constexpr uint8_t elevationStep2 = -1;
constexpr uint8_t elevationDirection2 = -1;
