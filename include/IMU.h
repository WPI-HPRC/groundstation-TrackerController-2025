#pragma once

#include <Arduino.h>
#include <ICM42688.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

struct IMUData {
    double xAcc;
    double yAcc;
    double zAcc;

    double xGyr;
    double yGyr;
    double zGyr;

    float xMag;
    float yMag;
    float zMag;
};

class IMU
{
    public:
        IMU() : icm(Wire, 0x68), mag() {};
        // IMU() : icm(Wire, 0x68) {};

        IMUData getData() { return data; };

        void debugPrint(Stream* printInterface)
        {
            printInterface->print("xAcc: "); printInterface->print(data.xAcc, 4); printInterface->print(", ");
            printInterface->print("yAcc: "); printInterface->print(data.yAcc, 4); printInterface->print(", ");
            printInterface->print("zAcc: "); printInterface->print(data.zAcc, 4); printInterface->print(", ");

            printInterface->print("xGyr: "); printInterface->print(data.xGyr, 4); printInterface->print(", ");
            printInterface->print("yGyr: "); printInterface->print(data.yGyr, 4); printInterface->print(", ");
            printInterface->print("zGyr: "); printInterface->print(data.zGyr, 4); printInterface->print(", ");

            printInterface->print("xMag: "); printInterface->print(data.xMag, 4); printInterface->print(", ");
            printInterface->print("yMag: "); printInterface->print(data.yMag, 4); printInterface->print(", ");
            printInterface->print("zMag: "); printInterface->print(data.zMag, 4); printInterface->println();
        }

        void update(){ poll(); };
        bool begin(){ return init_impl(); };

    private:
        IMUData data;

        // hardware objects
        ICM42688 icm;
        SFE_MMC5983MA mag;

        bool init_impl(){
            bool magStatus = mag.begin();
            if(magStatus){
                mag.softReset();
            }
            return icm.begin() > 0 
                && icm.setAccelFS(ICM42688::gpm16) > 0 
                && icm.setGyroFS(ICM42688::dps2000) > 0
                ;
                // && magStatus;
        }

        void poll() {
            // accel/gyro
            icm.getAGT();
            

            data.xAcc = icm.accX();
            data.yAcc = icm.accY();
            data.zAcc = icm.accZ(); 

            data.xGyr = icm.gyrX();
            data.yGyr = icm.gyrY();
            data.zGyr = icm.gyrZ();
            
            
            // magnetometer
            
            uint32_t rawX = mag.getMeasurementX();
            uint32_t rawY = mag.getMeasurementY();
            uint32_t rawZ = mag.getMeasurementZ();
      
            double scaledX = (double)rawX - 131072.0;
            scaledX /= 131072.0;
            double scaledY = (double)rawY - 131072.0;
            scaledY /= 131072.0;
            double scaledZ = (double)rawZ - 131072.0;
            scaledZ /= 131072.0;
      
            data.xMag = scaledX * 8.0;
            data.yMag = scaledY * 8.0;
            data.zMag = scaledZ * 8.0;
      
            return;
        }
};

