#pragma once

#include <Arduino.h>

class Sensor
{
    public:
        Sensor(){};
        virtual void setSensorPins(uint8_t){};
        virtual void setSensorPins(uint8_t, uint8_t, uint8_t){};

        void setPhysicalConversionConstant(double ConversionConstant)
            { conversionConstant = ConversionConstant; };

        void setZero(int64_t ZeroPos)
            {zeroPos = ZeroPos; zeroed = true; };

        virtual uint8_t begin();
        virtual uint8_t update();

        bool isZeroed() { return zeroed; };
        
        // returns in the unit specified by your conversion constant
        virtual float getDistFrom0()
            {return currentPos * conversionConstant; };

        virtual void debugPrint(Stream* printInterface);

        virtual ~Sensor() {};

    protected:
        int64_t currentPos; // use integers internally to prevent any floating point weirdness
        int64_t zeroPos = 0;

        float conversionConstant;

        bool zeroed = false;
};
