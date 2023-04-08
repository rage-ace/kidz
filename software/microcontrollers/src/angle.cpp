#include "angle.h"

#include <cmath>
#include <cstdint>
#include <wiring.h>

// Bearings are easier to do math with...

// Ensures that the bearing is between 0.0F and 360.0F.
float clipBearing(float dividend) {
    const auto r = fmodf(dividend, 360.0);
    return r < 0 ? r + 360.0 : r;
}

// Returns the difference between two angles, between 0.0F and 360.0F.
float bearingDiff(float leftAngle, float rightAngle) {
    return clipBearing(rightAngle - leftAngle);
}

// Returns the smaller of two angle differences, between 0.0F and 180.0F.
float smallerBearingDiff(float leftAngle, float rightAngle) {
    const auto angle = bearingDiff(leftAngle, rightAngle);
    return fmin(angle, 360 - angle);
}

// Returns the midpoint of two angles, between 0.0F and 360.0F.
float bearingMidpoint(float leftAngle, float rightAngle) {
    return clipBearing(leftAngle + bearingDiff(leftAngle, rightAngle) / 2.0);
}

// Angles are easier for the robot to understand...

float bearingToAngle(float bearing) {
    return bearing > 180 ? bearing - 360 : bearing;
}

// Ensures that the angle is between -179.99F and +180.00F.
float clipAngle(float dividend) {
    const auto r = fmod(dividend, 360);
    return r <= -180 ? r + 360 : r > 180 ? r - 360 : r;
}

// Keep everything in degrees...

float sinfd(float x) { return sin(x * DEG_TO_RAD); }

float cosfd(float x) { return cos(x * DEG_TO_RAD); }

float atan2fd(float x, float y) { return clipAngle(atan2(x, y) * RAD_TO_DEG); }
