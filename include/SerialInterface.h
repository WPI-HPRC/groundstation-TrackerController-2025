#pragma once
#include <Arduino.h>
#include "Sensor.h"
#include "IMU.h"
#include "AxisController.h"

#define READ_BUFFER_LENGTH 64 // Arbitrary length, but is longer than any currently-defined messages

class SerialInterface
{
public:
    SerialInterface(Sensor *azimuthSensor, Sensor *elevationSensor, IMU *imu, AxisController *azimuthController, AxisController *elevationController): 
        azimuthSensor(azimuthSensor), elevationSensor(elevationSensor), imu(imu), azimuthController(azimuthController), elevationController(elevationController) {}

    void send(const char *buffer, size_t length_bytes)
    {
        Serial.write(buffer, length_bytes);
    }

    void read()
    {
        // Read bytes until there aren't any more to be read
        do 
        {
            size_t bytesRead = Serial.readBytesUntil('E', &readBuffer[readBufferIndex], READ_BUFFER_LENGTH - readBufferIndex);

            if(readBuffer[readBufferIndex+bytesRead == 'E'])
            {
                // Now we know a packet has been read completely, so handle it here
                handleMessage(readBuffer);
                readBufferIndex = 0;
                // Optionally, break here if we only want to read/handle one packet maximum per loop
            }
            else
            {
                readBufferIndex += bytesRead;
            }
        } while (Serial.peek() != -1);
    }


private:
    Sensor *azimuthSensor;
    Sensor *elevationSensor;
    IMU *imu;

    AxisController *azimuthController;
    AxisController *elevationController;

    char readBuffer[READ_BUFFER_LENGTH];
    uint readBufferIndex = 0;

private:
    // Takes UTF-8 encoded digits and converts them into an integer. Assumes the most significant digit comes first
    static int utf8DigitsToInt(const char *encodedDigits, size_t length_bytes)
    {
        int intValue = 0;

        for(int i = 0; i < length_bytes; i++)
        {
            char currentChar = encodedDigits[i];

            if('0' <= currentChar && currentChar <= '9') // Make sure the byte we're reading is a valid UTF-8-encoded integer
            {
                // To find the magnitude of the current digit, take the total number of digits minus 1 and subtract from it the index of the current digit 
                // Raise 10 to that power and multiply it by the digit's integer value, which is the encoded number as an integer minus the character '0' as an integer
                intValue += pow(10, (length_bytes - 1 - i)) * (currentChar - '0');
            }
            else
            {
                return -1;
            }
        }
        return intValue;
    }
    
    static int bytesUntilSemicolon(const char *buffer)
    {
        // We assume there is actually a semicolon in the buffer
        int i = 0;
        while(buffer[i] != ';') { i++; }
        return i;
    }

    void handleSetter_pose(const char *buffer)
    {
        int buffer_index = bytesUntilSemicolon(buffer);

        float azimuth_degrees = (float)utf8DigitsToInt(buffer, buffer_index) / 100;

        char *remainingBuffer = (char *)&buffer[buffer_index + 1];

        buffer_index = bytesUntilSemicolon(remainingBuffer);

        float elevation_degrees = (float)utf8DigitsToInt(remainingBuffer, buffer_index) / 100;
        
        azimuthController->setTarget(azimuth_degrees);
        elevationController->setTarget(elevation_degrees);
        
        String response = "R;P;E";
        send(response.c_str(), response.length());
    }

    void handleSetter(const char *buffer)
    {
        char setterType = buffer[0];
        switch(setterType)
        {
            case 'P':
            {
                // Skip the semicolon in the message
                handleSetter_pose(&buffer[2]);
                break;
            }
            default:
            {
                return;
            }
        }
    }

    void handleGetter_pose()
    {
        int azimuth_degrees = round(azimuthSensor->getDistFrom0() * 100);
        int elevation_degrees = round(elevationSensor->getDistFrom0() * 100);

        String str = "D;L;";
        str += String(azimuth_degrees) + ";";
        str += String(elevation_degrees)  + ";";
        str += 'E';

        send(str.c_str(), str.length());
    }

    void handleGetter_gps()
    {
        // TODO: Add the GPS values once GPS is implemented
        int gpsLat_decimal = round(0 * 100);
        int gpsLong_decimal = round(0 * 100);

        String str = "D;G;";
        str += String(gpsLat_decimal) + ";";
        str += String(gpsLong_decimal)  + ";";
        str += 'E';

        send(str.c_str(), str.length());
    }

    void handleGetter_imu()
    {
        IMUData imuData = imu->getData();
        int gravityX_gs = round(imuData.xAcc * 100);
        int gravityY_gs = round(imuData.xAcc * 100);
        int gravityZ_gs = round(imuData.xAcc * 100);

        // TODO: implement heading calculation
        int heading_degrees = round(0 * 100);

        String str = "D;I;";
        str += String(gravityX_gs) + ";";
        str += String(gravityY_gs)  + ";";
        str += String(gravityZ_gs)  + ";";
        str += String(heading_degrees)  + ";";
        str += 'E';

        send(str.c_str(), str.length());
    }

    void handleEstop_brake()
    {
        azimuthController->eStopController();
        elevationController->eStopController();
        Serial.write("D;B;E");
    }

    void handleEstop_coast()
    {
        azimuthController->smoothStopController();
        elevationController->smoothStopController();
        Serial.write("D;C;E");
    }

    void handleGetter(const char *buffer)
    {
        char getterType = buffer[0];
        switch(getterType)
        {
            case 'L':
            {
                handleGetter_pose();
                break;
            }
            case 'G':
            {
                handleGetter_gps();
                break;
            }
            case 'I':
            {
                handleGetter_imu();
                break;
            }
            case 'B':
            {
                handleEstop_brake();
                break;
            }
            case 'C':
            {
                handleEstop_coast();
                break;
            }
            default:
            {
                // Should never happen
                return;
            }
        };
    }

    void handleMessage(const char *buffer)
    {
        // For now, we will just read the first byte since all message types are given by a single byte. In the future, we can change this
        char messageType = buffer[0];
        switch(messageType)
        {
            case 'S': // Setter
            {
                // Skip the semicolon in the message
                handleSetter(&buffer[2]);
                break;
            }
            case 'G': // Getter
            {
                handleSetter(&buffer[2]);
                break;
            }
            default:
            {
                // Should not happen
                return;
            }
        }
    }
};