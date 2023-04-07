#include "movement.h"

#include <Arduino.h>
#include <cstdint>

#include "angle.h"
#include "teensy/include/config.h"
#include "vector.h"

Movement::Movement() {}

void Movement::init() {
    // Values from https://www.pjrc.com/teensy/td_pulse.html
    // (based on F_CPU_ACTUAL = 600 MHz)
    // analogWriteResolution(12); // TODO: Debug with Multimeter
    // analogWriteFrequency(PIN_MOTOR_FL_PWM, 36621);
    // analogWriteFrequency(PIN_MOTOR_FR_PWM, 36621);
    // analogWriteFrequency(PIN_MOTOR_BL_PWM, 36621);
    // analogWriteFrequency(PIN_MOTOR_BR_PWM, 36621);
    analogWriteResolution(10);

    // Initialise pins
    pinMode(PIN_MOTOR_FL_DIR, OUTPUT);
    pinMode(PIN_MOTOR_FR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_BL_DIR, OUTPUT);
    pinMode(PIN_MOTOR_BR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_FL_PWM, OUTPUT);
    pinMode(PIN_MOTOR_FR_PWM, OUTPUT);
    pinMode(PIN_MOTOR_BL_PWM, OUTPUT);
    pinMode(PIN_MOTOR_BR_PWM, OUTPUT);

    analogWriteFrequency(PIN_DRIBBLER_PWM, 1000);
    pinMode(PIN_DRIBBLER_PWM, OUTPUT);

    // Arm dribbler
    analogWrite(PIN_DRIBBLER_PWM, 128);
    delay(3000);
}

void Movement::updateHeadingController(const float angle) {
    _angularVelocity = _headingController.advance(angle);
    _actualHeading = angle;
}

void Movement::setMoveTo(const Point &destination, const float targetHeading,
                         const Goals &goals) {
    // We use an algorithm that allows us to minimise error propagated by goal
    // distance and rely more on goal angle

    // Compute reference vectors
    const auto center = (goals.offensive + goals.defensive) / 2;
    const auto centerToOffensiveGoal = goals.offensive - center;

    // Compute destination vector with reference vectors
    const auto realDestination = Vector::fromPoint(destination);
    const auto scalingFactor =
        centerToOffensiveGoal.distance / HALF_GOAL_SEPARATION;
    const auto scaledDestination = realDestination * scalingFactor;
    const auto relativeDestination = center + scaledDestination;

    // Update move to state
    if (_lastDestination == nullptr || *_lastDestination != destination) {
        // A new move to routine just started
        _lastDestination = new Point(destination);
        _initialDistance = relativeDestination.distance;
        _moveToController.reset();
    }

    // Pack it into instructions for our update function
    angle = relativeDestination.angle;
    if (relativeDestination.distance > MOVE_TO_PRECISION) {
        // The destination hasn't been reached
        velocity = _moveToController.advance(relativeDestination.distance);
        heading = (targetHeading - _actualHeading) *
                  fmin(relativeDestination.distance / _initialDistance, 1.0);
    } else {
        // The destination has basically been reached
        velocity = 0;
        heading = targetHeading;
    }
}

// Writes the current movement data.
void Movement::update() {
    if (stop) {
        // Stop the motors
        analogWrite(PIN_MOTOR_FL_PWM, 0);
        analogWrite(PIN_MOTOR_FR_PWM, 0);
        analogWrite(PIN_MOTOR_BL_PWM, 0);
        analogWrite(PIN_MOTOR_BR_PWM, 0);
        stop = false;
        return;
    }

    // Convert polar to cartesian
    const auto x = sinf(angle * M_PI / 180);
    const auto y = cosf(angle * M_PI / 180);

    // Compute the speeds of the individual motors
    const auto transformSpeed = [this](float velocity_,
                                       float angularComponent) {
        return (int16_t)roundf(velocity_ * velocity + angularComponent);
    };
    // Find angular component
    const auto angular = 0.25F * _angularVelocity;
    // Compute speeds
    const int16_t FLSpeed = transformSpeed(x * COS45 + y * SIN45, angular);
    const int16_t FRSpeed = transformSpeed(x * -COS45 + y * SIN45, -angular);
    const int16_t BLSpeed = transformSpeed(x * -COS45 + y * SIN45, +angular);
    const int16_t BRSpeed = transformSpeed(x * COS45 + y * SIN45, -angular);

    // Constrain motor speed
    auto constrainSpeed = [this](int16_t speed) {
        // If the speed is below the stall speed, don't bother moving
        if (abs(speed) < DRIVE_STALL_SPEED) return 0;
        return min(abs(speed), DRIVE_MAX_SPEED);
    };

    // Set the motor directions and speeds
    digitalWriteFast(PIN_MOTOR_FL_DIR,
                     FLSpeed > 0 ? MOTOR_FL_REVERSED : !MOTOR_FL_REVERSED);
    digitalWriteFast(PIN_MOTOR_FR_DIR,
                     FRSpeed > 0 ? MOTOR_FR_REVERSED : !MOTOR_FR_REVERSED);
    digitalWriteFast(PIN_MOTOR_BL_DIR,
                     BLSpeed > 0 ? MOTOR_BL_REVERSED : !MOTOR_BL_REVERSED);
    digitalWriteFast(PIN_MOTOR_BR_DIR,
                     BRSpeed > 0 ? MOTOR_BR_REVERSED : !MOTOR_BR_REVERSED);
    analogWrite(PIN_MOTOR_FL_PWM, constrainSpeed(FLSpeed));
    analogWrite(PIN_MOTOR_FR_PWM, constrainSpeed(FRSpeed));
    analogWrite(PIN_MOTOR_BL_PWM, constrainSpeed(BLSpeed));
    analogWrite(PIN_MOTOR_BR_PWM, constrainSpeed(BRSpeed));

    // Set dribbler motor speed
    analogWrite(PIN_DRIBBLER_PWM,
                dribble ? DRIBBLER_SPEED : DRIBBLER_ARM_SPEED);

    // Deactivate solenoid if duration has elapsed
    if (_kickerActivated && millis() - _kickTime > KICKER_ACTIVATION_DURATION) {
        digitalWriteFast(PIN_KICKER, LOW);
        _kickerActivated = false;
    }

    // Invalidate move to routine (it can be reactivated by calling setMoveTo)
    if (!_moveToActive) {
        delete _lastDestination;
        _lastDestination = nullptr;
    }
    _moveToActive = false;
}

void Movement::kick() {
    if (_kickerActivated || millis() - _kickTime < KICKER_COOLDOWN_DURATION)
        // Don't kick if the solenoid is already activated or if the cooldown
        // period is not over
        return;

    digitalWriteFast(PIN_KICKER, HIGH);
    _kickTime = millis();
    _kickerActivated = true;
}
