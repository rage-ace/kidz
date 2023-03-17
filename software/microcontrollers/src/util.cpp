#include "util.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <cmath>

// Ensures that the angle is between 0.0F and 360.0F.
float angleMod(float dividend) {
    const auto r = fmodf(dividend, 360.0);
    return r < 0 ? r + 360.0 : r;
}

// Returns the difference between two angles, between 0.0F and 360.0F.
float angleDiff(float leftAngle, float rightAngle) {
    return angleMod(rightAngle - leftAngle);
}

// Returns the smaller of two angle differences, between 0.0F and 180.0F.
float smallerAngleDiff(float leftAngle, float rightAngle) {
    const auto angle = angleDiff(leftAngle, rightAngle);
    return fmin(angle, 360 - angle);
}

// Returns the midpoint of two angles, between 0.0F and 360.0F.
float angleMidpoint(float leftAngle, float rightAngle) {
    return angleMod(leftAngle + angleDiff(leftAngle, rightAngle) / 2.0);
}

// Reads the last packet from the serial port if there is any in the buffer.
// Return if a packet has been read.
bool readPacket(Stream &serial, void *writeTo, unsigned int packetSize,
                byte startByte, byte endByte) {
    bool read = false;
    while (serial.available() >= (signed int)packetSize) {
        const auto syncByte = serial.read();
        if (syncByte == startByte) {
            // start of packet is valid, proceed to read rest of packet
            char buf[packetSize - 1];
            serial.readBytes(buf, packetSize - 1);
            if (buf[packetSize - 2] == endByte) {
                // end of packet is valid, proceed to save payload
                memcpy(writeTo, buf, packetSize - 2);
                read = true;
            }
        }
    }
    return read;
}

// Sends a packet over the serial port.
void sendPacket(Stream &serial, void *readFrom, unsigned int packetSize,
                byte startByte, byte endByte) {
    serial.write(startByte);
    serial.write((const uint8_t *)readFrom, packetSize - 2);
    serial.write(endByte);
}

// Scans the I2C bus for devices.
void scanI2C(Stream &serial, TwoWire wire) {
    // Adapted from
    // https://github.com/stm32duino/Arduino_Core_STM32/blob/main/libraries/Wire/examples/i2c_scanner/i2c_scanner.ino
    byte error, address;
    uint8_t deviceCount = 0;

    serial.println("Scanning...");

    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.

        wire.beginTransmission(address);
        error = wire.endTransmission();

        if (error == 0) {
            serial.printf("I2C device found at address 0x%02x\n", address);
            ++deviceCount;
        } else if (error == 4) {
            serial.printf("Unknown error at address 0x%02x\n", address);
        }
    }

    if (deviceCount == 0)
        serial.println("No I2C devices found\n");
    else
        serial.printf("Found %d I2C devices\n", deviceCount);
}

// Wipes the EEPROM.
void wipeEEPROM() {
    Serial.println("Wiping EEPROM...");
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
        Serial.printf("%4d / %d\n", i, EEPROM.length());
    }
    Serial.println("Done");
}
