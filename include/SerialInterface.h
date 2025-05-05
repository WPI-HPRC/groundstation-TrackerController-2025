#pragma once
#include <Arduino.h>

#define READ_BUFFER_LENGTH 32 // Arbitrary length, but is longer than any currently-defined messages

class SerialInterface
{
    void read()
    {
        // Read 
        do 
        {
            size_t bytesRead = Serial.readBytesUntil('E', &readBuffer[readBufferIndex], READ_BUFFER_LENGTH);

            if(readBuffer[readBufferIndex+bytesRead == 'E'])
            {
                // Now we know a packet has been read completely, so handle it here
            }
            else
            {
                readBufferIndex = bytesRead;
            }
        } while (Serial.peek() != -1);
    }

    private:
        char readBuffer[READ_BUFFER_LENGTH];
        uint readBufferIndex = 0;
}