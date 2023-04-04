#include "angle.h"

#include <cmath>
#include <cstdint>

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

int16_t bearingToAngle(float bearing) {
    const auto angle = roundf(bearing * 100);
    return angle > 18000 ? angle - 36000 : angle;
}

// Ensures that the angle is between -179(.)99 and +180(.)00.
int16_t clipAngle(int32_t dividend) {
    const auto r = dividend % 36000;
    return r <= -18000 ? r + 36000 : r > 18000 ? r - 36000 : r;
}
