#include "counter.h"

#include <Arduino.h>
#include <assert.h>

// Creates a counter
Counter::Counter() {}

// Check if the specified number of iterations have elapsed
bool Counter::countElapsed(const uint32_t dcount) {
    // If the counter has not started, reset the values
    if (!started) {
        started = true;
        _lastDcount = dcount;
        return false;
    }

    // Check if the interval has passed and return accordingly
    if (dcount != UINT32_MAX) {
        if (_count++ >= dcount) {
            _count = 0;
            return true;
        } else
            return false;
    }

    // If no interval was specified, return true
    return true;
}

// Check if the specified number of milliseconds have elapsed
bool Counter::millisElapsed(const uint32_t dmillis) {
    // If the counter has not started, reset the values
    if (!started) {
        started = true;
        _lastDmillis = dmillis;
        return false;
    }

    // Check if the interval has passed and return accordingly
    if (dmillis != UINT32_MAX) {
        if (millis() - _lastTime >= dmillis) {
            _lastTime = millis();
            return true;
        } else
            return false;
    }

    // If no interval was specified, return true
    return true;
}

// Check if the specified number of microseconds have elapsed
bool Counter::microsElapsed(const uint32_t dmicros) {
    // If the counter has not started, reset the values
    if (!started) {
        started = true;
        _lastDmicros = dmicros;
        return false;
    }

    // Check if the interval has passed and return accordingly
    if (dmicros != UINT32_MAX) {
        if (micros() - _lastTime >= dmicros) {
            _lastTime = micros();
            return true;
        } else
            return false;
    }

    // If no interval was specified, return true
    return true;
}

// Reset the counter
void Counter::reset() { started = false; }
