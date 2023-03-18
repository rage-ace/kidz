#ifndef UTIL_H
#define UTIL_H

#include "Wire.h"

void scanI2C(Stream &serial, TwoWire wire);

void wipeEEPROM();

#endif
