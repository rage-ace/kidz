#include "pid.h"

#include <Arduino.h>

// A simple PID controller.
PIDController::PIDController(const float kp, const float ki, const float kd,
                             const float min, const float max,
                             const float setpoint)
    : _kp(kp), _ki(ki), _kd(kd), _min(min), _max(max), _setpoint(setpoint) {}

// Update controller,
float PIDController::advance(const float input) {
    // Find dt
    const auto now = micros();
    const auto dt = now - _lastTime;
    _lastTime = now;

    // Find PID components
    const auto error = _setpoint - input;
    _integral += error * dt;
    const auto derivative = (error - _lastError) / dt;
    _lastError = error;

    // Combine components to get output
    const auto output = _kp * error + _ki * _integral + _kd * derivative;
    return constrain(output, _min, _max);
}

// Update setpoint.
void PIDController::updateSetpoint(const float setpoint) {
    _setpoint = setpoint;
}

// Update limits.
void PIDController::updateLimits(const float min, const float max) {
    _min = min;
    _max = max;
}

// Update gains.
void PIDController::updateGains(const float kp, const float ki,
                                const float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}