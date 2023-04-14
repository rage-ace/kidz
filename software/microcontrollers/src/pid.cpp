#include "pid.h"

#include <Arduino.h>

#include "angle.h"

// A simple PID controller.
PIDController::PIDController(const float targetSetpoint, const float min,
                             const float max, const float kp, const float ki,
                             const float kd, const uint32_t minDt,
                             const float maxi, const float maxSetpointChange)
    : _targetSetpoint(targetSetpoint), _min(min), _max(max), _kp(kp), _ki(ki),
      _kd(kd), _minDt(minDt), _maxi(maxi / ki),
      _maxSetpointChange(maxSetpointChange) {}

// Update controller,
float PIDController::advance(const float input, const float scaler,
                             const bool isAngle) {
    // If this is the first iteration, don't advance the controller yet
    if (_justStarted) {
        _justStarted = false;
        return 0;
    }

    // Try to move setpoint towards target setpoint
    const auto dsetpoint = constrain(_targetSetpoint - _setpoint,
                                     -_maxSetpointChange, _maxSetpointChange);
    _setpoint += dsetpoint;

    // If dt is too short, don't advance the controller yet
    if (micros() - _lastTime < _minDt) return _lastOutput;

    // Find dt
    const auto now = micros();
    const auto dt = now - _lastTime;
    _lastTime = now;

    // Find PID components
    auto error = _setpoint - input;

    // TODO: Consider whether this is beneficial
    // // Correct for errors that are angles
    // if (isAngle) {
    //     // If it goes beyond ±180º ∓ 90º, we try correcting in the other
    //     // direction instead of going all the way around
    //     if (error > 270.0F) {
    //         error = -90.0F;
    //         reset();
    //     } else if (error < -270.0F) {
    //         error = 90.0F;
    //         reset();
    //     }
    // }

    _integral += error * dt;
    _integral = constrain(_integral, -_maxi, _maxi);
    const auto p = (_kp * scaler) * error;
    const auto i = (_ki * _kp / dt * scaler) * _integral;
    const auto d =
        (_kd * _kp * dt * powf(scaler, 2)) * (error - _lastError) / dt;

    // Combine components to get output
    const auto output = constrain(p + i + d, _min, _max);

    // For debugging
    _lastInput = input;
    _lastP = p;
    _lastI = i;
    _lastD = d;
    _lastDt = dt;

    // For next iteration
    _lastOutput = output;
    _lastError = error;

    return output;
}

void PIDController::reset() {
    _integral = 0.0F;
    _lastError = 0.0F;
    _lastTime = 0;
    _lastOutput = 0.0F;
    _justStarted = true;
}

// Update setpoint.
void PIDController::updateSetpoint(const float value) {
    _targetSetpoint = value;
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

void PIDController::debugPrint(const char *name, Stream &serial) {
    const auto printFloat = [&serial](const float value) {
        serial.printf("%5d.%02d", (int)value, abs((int)(value * 100) % 100));
    };

    if (name != nullptr) serial.printf("[%s] ", name);
    serial.printf("Setpoint: ");
    printFloat(_targetSetpoint);
    serial.printf(" | Input: ");
    printFloat(_lastInput);
    serial.printf(" | Error: ");
    printFloat(_lastError);
    serial.printf(" | Output: ");
    printFloat(_lastOutput);
    serial.printf(" | P: ");
    printFloat(_lastP);
    serial.printf(" | I: ");
    printFloat(_lastI);
    serial.printf(" | D: ");
    printFloat(_lastD);
    serial.printf(" | dt: %4d", _lastDt);
    serial.println();
}