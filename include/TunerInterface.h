#pragma once

#include <Arduino.h>

class TunerInterface {
public:
    TunerInterface(Stream *StreamInterface){streamInterface = StreamInterface; };

    void readSerial() {
        String command = Serial.readStringUntil('\n'); // Read the entire line
    
        if (command.startsWith("APPLY,")) {
            // Strip off the "APPLY," prefix
            command.remove(0, 6);
            apply = true;
            
            // Split the command into separate values by commas
            int startIndex = 0;
            int endIndex = command.indexOf(',');

            // Parse each value from the string
            maxVel = command.substring(startIndex, endIndex).toFloat();
            startIndex = endIndex + 1;
            endIndex = command.indexOf(',', startIndex);
            maxAccel = command.substring(startIndex, endIndex).toFloat();
            startIndex = endIndex + 1;
            endIndex = command.indexOf(',', startIndex);
            maxJerk = command.substring(startIndex, endIndex).toFloat();
            startIndex = endIndex + 1;
            endIndex = command.indexOf(',', startIndex);
            kP = command.substring(startIndex, endIndex).toFloat();
            startIndex = endIndex + 1;
            endIndex = command.indexOf(',', startIndex);
            kD = command.substring(startIndex, endIndex).toFloat();
            startIndex = endIndex + 1;
            endIndex = command.indexOf(',', startIndex);
            gravityFF = command.substring(startIndex, endIndex).toFloat();
            startIndex = endIndex + 1;
            endIndex = command.indexOf(',', startIndex);
            error = command.substring(startIndex, endIndex).toFloat();
            startIndex = endIndex + 1;
            endIndex = command.indexOf(',', startIndex);
            target = command.substring(startIndex, endIndex).toFloat();
            startIndex = endIndex + 1;
            brake = command.substring(startIndex).toInt() == 1;
            return;
        }
        apply = false;
    }

    float getKP(){ return kP; }
    float getKD(){ return kD; }
    float getGravFF(){ return gravityFF; }
    float getMaxVel(){ return maxVel; }
    float getMaxAccel(){ return maxAccel; }
    float getMaxJerk(){ return maxJerk; }
    float getTarget(){ return target; }
    bool getBrake(){ return brake; }
    bool newData(){ return apply; };
    float getAcceptableError(){ return error; };

private:
    Stream *streamInterface;

    float kP;
    float kD;
    float gravityFF;
    float maxVel;
    float maxAccel;
    float maxJerk;
    bool brake;
    bool apply;
    float target;
    float error;
};