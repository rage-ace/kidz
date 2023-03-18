#ifndef UTIL_H
#define UTIL_H

#include "Wire.h"

#define SIN45 0.70710678F
#define COS45 0.70710678F

float angleDiff(float leftAngle, float rightAngle);
float smallerAngleDiff(float leftAngle, float rightAngle);
float angleMidpoint(float leftAngle, float rightAngle);

bool readPacket(Stream &serial, void *writeTo, unsigned int packetSize,
                byte startByte, byte endByte);
void sendPacket(Stream &serial, void *readFrom, unsigned int packetSize,
                byte startByte, byte endByte);

void scanI2C(Stream &serial, TwoWire wire);

void wipeEEPROM();

uint32_t printLoopTime(uint16_t sampleCount = 50U);

#endif
