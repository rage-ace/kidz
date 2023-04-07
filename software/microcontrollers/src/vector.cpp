#include "vector.h"

#include <Arduino.h>
#include <cmath>

Vector::Vector(float angle, float distance)
    : angle(angle), distance(distance) {}

Vector Vector::fromPoint(Point point) {
    return {atan2f(point.y, point.x),
            sqrtf(point.x * point.x + point.y * point.y)};
}

Vector &Vector::operator=(const Vector &other) {
    this->angle = other.angle;
    this->distance = other.distance;
    return *this;
}

Vector Vector::operator+(const Vector &other) const {
    float x = distance * cos(angle) + other.distance * cos(other.angle);
    float y = distance * sin(angle) + other.distance * sin(other.angle);
    return {atan2f(y, x), sqrtf(x * x + y * y)};
}

Vector Vector::operator-(const Vector &other) const {
    float x = distance * cos(angle) - other.distance * cos(other.angle);
    float y = distance * sin(angle) - other.distance * sin(other.angle);
    return {atan2f(y, x), sqrtf(x * x + y * y)};
}

Vector Vector::operator*(const float other) const {
    return {angle, distance * other};
}

Vector Vector::operator/(const float other) const {
    return {angle, distance / other};
}
