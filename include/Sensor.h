#ifndef Sensor_h
#define Sensor_h

#include <Arduino.h>

class Sensor
{
    public:
        Sensor(){};
        virtual void setSensorPins(uint8_t){};
        virtual void setSensorPins(uint8_t, uint8_t, uint8_t){};

        void setPhysicalConversionConstant(double ConversionConstant)
            { conversionConstant = ConversionConstant; };

        virtual uint8_t begin();
        virtual uint8_t update();
        
        // returns in the unit specified by your conversion constant
        float getDistFrom0()
            { return currentPos * conversionConstant; }; // sensors should use their raw value internally, and we only convert to the desired unit with the conversion constant before we give it to the user

        virtual ~Sensor() {};

    protected:
        int64_t currentPos; // use integers internally to prevent any floating point weirdness
        int64_t zeroPos;

        float conversionConstant;
};

#endif // Sensor_h