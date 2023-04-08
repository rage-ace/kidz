#ifndef ANGLE_H
#define ANGLE_H

#include <cstdint>

#define SIN45 0.70710678F
#define COS45 0.70710678F

float clipBearing(float dividend);
float bearingDiff(float leftAngle, float rightAngle);
float smallerBearingDiff(float leftAngle, float rightAngle);
float bearingMidpoint(float leftAngle, float rightAngle);

float bearingToAngle(float bearing);
float clipAngle(float dividend);

float sinfd(float x);
float cosfd(float x);
float atan2fd(float x, float y);

#endif
