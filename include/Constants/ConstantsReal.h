#ifndef ConstantsReal_h
#define ConstantsReal_h

////////////////////////////////////////////////////////////////////// Physical Constants //////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////// Physical Constants //////////////////////////////////////////////////////////////////////

#define azimuthMainGearTeeth 480 // gear teeth
#define azimuthMotorPinionTeeth 40 // gear teeth
#define azimuthEncoderPinionTeeth 40 // gear teeth

#define elevationGearboxReduction 20 // ratio
#define elevationChainReduction 3 // ratio

#define elevationTotalReduction elevationGearboxReduction*elevationChainReduction // ratio

#define azimuthEncoderActualTicksPerRev 600 // encoder ticks per rev
#define azimuthEncoderTicksPerRev azimuthEncoderActualTicksPerRev*4 // quadurature counting means 4 counts per pulse

#define microStepResolution 40000 // steps per rev

////////////////////////////////////////////////////////////////////// Sensor Calibration Values //////////////////////////////////////////////////////////////////////

#define elevationMinimumValue 1024 // ADC reading value
#define elevationMaximumValue 0 // ADC reading value

#define elevationMaximumAngle 90 // degrees
#define elevationMinimumAngle -90 // degrees

constexpr int elevationZeroValue = (elevationMaximumValue - elevationMinimumValue) / 2 ;// ADC reading value

// this assumes a linear relationship (which should be true since we bought linear pots)
constexpr double elevationConversionRatio ( (elevationMaximumAngle-elevationMinimumAngle) / (elevationMaximumValue-elevationMinimumValue) ); // adc readings to degrees


////////////////////////////////////////////////////////////////////// Pins //////////////////////////////////////////////////////////////////////

#define azimuthEncoderA 1
#define azimuthEncoderB 2

#define azimuthLimitSwitch 3

#define azimuthStep 4
#define azimuthDirection 5



#define elevationPotentiometer 9

#define elevationStep 6
#define elevationDirection 7
#define elevationDirectionNOT 8


#endif // ConstantsReal_h