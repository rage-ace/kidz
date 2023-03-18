#ifndef ANGLE_H
#define ANGLE_H

#include <cstdint>

#define SIN45 0.70710678F
#define COS45 0.70710678F

float clipBearing(float dividend);
float bearingDiff(float leftAngle, float rightAngle);
float smallerBearingDiff(float leftAngle, float rightAngle);
float bearingMidpoint(float leftAngle, float rightAngle);

int16_t bearingToAngle(float bearing);
int16_t clipAngle(int32_t dividend);

#endif
