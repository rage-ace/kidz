#include "pid.h"

#include <Arduino.h>

// A simple PID controller.
PIDController::PIDController(const float setpoint, const float min,
                             const float max, const float kp, const float ki,
                             const float kd, const uint32_t minDt)
    : _setpoint(setpoint), _min(min), _max(max), _kp(kp), _ki(ki), _kd(kd),
      _minDt(minDt) {}

// Update controller,
float PIDController::advance(const float input) {
    // If this is the first iteration, don't advance the controller yet
    if (_justStarted) {
        _justStarted = false;
        return 0;
    }

    // If dt is too short, don't advance the controller yet
    if (micros() - _lastTime < _minDt) return _lastOutput;

    // Find dt
    const auto now = micros();
    const auto dt = now - _lastTime;
    _lastTime = now;

    // Find PID components
    const auto error = _setpoint - input;
    _integral += error * dt;
    const auto p = _kp * error;
    const auto i = _ki * _integral;
    const auto d = _kd * (error - _lastError) / dt;

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

void PIDController::debugPrint(const char *name, Stream &serial) {
    const auto printFloat = [&serial](const float value) {
        serial.printf("%5d.%02d", (int)value, abs((int)(value * 100) % 100));
    };

    if (name != nullptr) serial.printf("[%s] ", name);
    serial.printf("Setpoint: ");
    printFloat(_setpoint);
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
    serial.printf(" | kP");
    printFloat(_kp);
    serial.printf(" | kI");
    printFloat(_ki);
    serial.printf(" | kD");
    printFloat(_kd);
    serial.println();
}