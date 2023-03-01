#include "util.h"

#include <cmath>

float angleMod(float dividend) {
    const auto r = fmodf(dividend, 360.0);
    return r < 0 ? r + 360.0 : r;
}

float angleDiff(float leftAngle, float rightAngle) {
    return angleMod(rightAngle - leftAngle);
}

float smallerAngleDiff(float leftAngle, float rightAngle) {
    const auto angle = angleDiff(leftAngle, rightAngle);
    return fmin(angle, 360 - angle);
}

float angleMidpoint(float leftAngle, float rightAngle) {
    return angleMod(leftAngle + angleDiff(leftAngle, rightAngle) / 2.0);
}
