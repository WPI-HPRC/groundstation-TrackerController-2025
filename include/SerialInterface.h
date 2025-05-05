#pragma once
#include <Arduino.h>

#define READ_BUFFER_LENGTH 32 // Arbitrary length, but is longer than any currently-defined messages

class SerialInterface
{
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

    void read()
    {
        // Read bytes until there aren't any more to be read
        do 
        {
            size_t bytesRead = Serial.readBytesUntil('E', &readBuffer[readBufferIndex], READ_BUFFER_LENGTH);

            if(readBuffer[readBufferIndex+bytesRead == 'E'])
            {
                // Now we know a packet has been read completely, so handle it here
                handleMessage();
                readBufferIndex = 0;
                // Optionally, break here if we only want to read/handle one packet maximum per loop
            }
            else
            {
                readBufferIndex += bytesRead;
            }
        } while (Serial.peek() != -1);
    }

    void handleSetter_pose(const char *buffer)
    {
        int buffer_index = bytesUntilSemicolon(buffer);

        float azimuth_degrees = (float)utf8DigitsToInt(buffer, buffer_index) / 100;

        char *remainingBuffer = (char *)&buffer[buffer_index + 1];

        buffer_index = bytesUntilSemicolon(remainingBuffer);

        float elevation_degrees = (float)utf8DigitsToInt(remainingBuffer, buffer_index) / 100;
        

        // Actually set the values
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
        }
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
                // Handle getter
                break;
            }
        }
    }

    private:
        char readBuffer[READ_BUFFER_LENGTH];
        uint readBufferIndex = 0;
};