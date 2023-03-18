#include "pid.h"

#include <Arduino.h>

// A simple PID controller.
PIDController::PIDController(float kp, float ki, float kd, float min, float max,
                             float setpoint)
    : _kp(kp), _ki(ki), _kd(kd), _min(min), _max(max), _setpoint(setpoint) {}

// Update controller,
float PIDController::advance(float input) {
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
void PIDController::updateSetpoint(float setpoint) { _setpoint = setpoint; }

// Update limits.
void PIDController::updateLimits(float min, float max) {
    _min = min;
    _max = max;
}

// Update gains.
void PIDController::updateGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}