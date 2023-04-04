#ifndef UTIL_H
#define UTIL_H

#include "Wire.h"

void scanI2C(Stream &serial, TwoWire wire);

void wipeEEPROM();

uint32_t printLoopTime(uint16_t sampleCount = 50);

#endif
