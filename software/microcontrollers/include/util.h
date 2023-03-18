#ifndef UTIL_H
#define UTIL_H

#include "Wire.h"

#define SIN45 0.70710678F
#define COS45 0.70710678F

float clipBearing(float dividend);
float bearingDiff(float leftAngle, float rightAngle);
float smallerBearingDiff(float leftAngle, float rightAngle);
float bearingMidpoint(float leftAngle, float rightAngle);

int16_t bearingToAngle(float bearing);
int16_t clipAngle(int32_t dividend);

bool readPacket(Stream &serial, void *writeTo, unsigned int packetSize,
                byte startByte, byte endByte);
void sendPacket(Stream &serial, void *readFrom, unsigned int packetSize,
                byte startByte, byte endByte);

void scanI2C(Stream &serial, TwoWire wire);

void wipeEEPROM();

uint32_t printLoopTime(uint16_t sampleCount = 50U);

#endif
