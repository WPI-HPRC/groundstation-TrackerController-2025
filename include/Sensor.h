#ifndef Sensor_h
#define Sensor_h

#include <Arduino.h>

class Sensor
{
    public:
        Sensor();
        virtual void setPins(uint8_t);
        virtual void setPins(uint8_t, uint8_t, uint8_t);

        virtual uint8_t begin();
        virtual uint8_t update();
        
        virtual int64_t getDistFrom0();

    protected:
        int64_t currentPos;
        int64_t zeroPos;
};

#endif // Sensor_h