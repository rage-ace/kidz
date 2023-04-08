#include "vector.h"

#include <Arduino.h>
#include <cmath>

#include "angle.h"

Vector::Vector(float angle, float distance)
    : angle(angle), distance(distance) {}

Vector Vector::fromPoint(Point point) {
    return {atan2fd(point.y, point.x),
            sqrtf(point.x * point.x + point.y * point.y)};
}

Vector &Vector::operator=(const Vector &other) {
    this->angle = other.angle;
    this->distance = other.distance;
    return *this;
}

Vector Vector::operator+(const Vector &other) const {
    float x = distance * cosfd(angle) + other.distance * cosfd(other.angle);
    float y = distance * sinfd(angle) + other.distance * sinfd(other.angle);
    return {atan2fd(y, x), sqrtf(x * x + y * y)};
}

Vector Vector::operator-(const Vector &other) const {
    float x = distance * cosfd(angle) - other.distance * cosfd(other.angle);
    float y = distance * sinfd(angle) - other.distance * sinfd(other.angle);
    return {atan2fd(y, x), sqrtf(x * x + y * y)};
}

Vector Vector::operator*(const float other) const {
    return {angle, distance * other};
}

Vector Vector::operator/(const float other) const {
    return {angle, distance / other};
}
