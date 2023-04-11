#ifndef MAIN_H
#define MAIN_H

#include <PacketSerial.h>

#include "teensy/include/movement.h"
#include "teensy/include/sensors.h"

// Serial
extern PacketSerial muxSerial;
extern PacketSerial tofSerial;
extern PacketSerial imuSerial;
extern PacketSerial coralSerial;

// IO
extern Sensors sensors;
extern Movement movement;

// striker.cpp
void runStriker();

// goalie.cpp
void runGoalie();

// calibrate.cpp
void performCalibration();

// debug.cpp
void performSetupDebug();
void performLoopDebug();

#endif
