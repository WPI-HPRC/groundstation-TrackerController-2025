#pragma once

#include <Arduino.h>
#include "IMU.h"

struct Orientation // all values are in radians
{
    float pitch; // radians
    float roll; // radians
    float yaw; // radians
};

class OrientationEstimator
{
    public:
        OrientationEstimator(IMU* Imu) : imu(Imu) {};

        void setCalibrationOffsets(float XMagOffset, float YMagOffset)
            { xMagOffset = XMagOffset; yMagOffset = YMagOffset; };

        bool begin() { return imu->begin(); };

        Orientation update()
        {
            imu->update(); // update the IMU data

            derive(); // actually derive the orientation 

            return orientation; // return orientation
        };

        Orientation getOrientation() { return orientation; };

        void debugPrint(Stream* printInterface)
        {
            printInterface->print("Pitch: "); printInterface->print(orientation.pitch, 4); printInterface->print(", ");
            printInterface->print("Roll: "); printInterface->print(orientation.roll, 4); printInterface->print(", ");
            printInterface->print("Yaw: "); printInterface->print(orientation.yaw, 4); printInterface->println();
        }


    private:
        IMU *imu;

        Orientation orientation;

        float xMagOffset, yMagOffset;

        void derive()
        {
            IMUData rawData = imu->getData();
            // normalize acceleration (shouldn't be needed but just in case)
            float normalAccel = sqrtf(rawData.xAcc*rawData.xAcc + rawData.yAcc*rawData.yAcc + rawData.zAcc*rawData.zAcc);
            rawData.xAcc = rawData.xAcc / normalAccel;
            rawData.yAcc = rawData.yAcc / normalAccel;
            rawData.zAcc = rawData.zAcc / normalAccel;

            // computing pitch and roll from accelerometer
            orientation.pitch = asin(-rawData.xAcc);
            orientation.roll = atan2(rawData.yAcc, rawData.zAcc);

            // perform tilt compensation for the magenetometer (from the accelerometer)
            // we don't do anything to zMag as it isn't relevent for our use case (magnetic heading of a mostly level body)
            float xMagComp = rawData.xMag * cos(orientation.pitch) + rawData.zMag * sin(orientation.pitch);
            float yMagComp = (
                                rawData.xMag * sin(orientation.roll) * sin(orientation.pitch) +
                                rawData.yMag * cos(orientation.roll) -
                                rawData.zMag * sin(orientation.roll) * cos(orientation.pitch)
            );

            float xMagCorrected = xMagComp - xMagOffset;
            float yMagCorrected = yMagComp - yMagOffset;

            // compute actual yaw (heading)
            orientation.yaw = atan2(-yMagCorrected, xMagCorrected);
        }
};