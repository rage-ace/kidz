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
    analogWriteResolution(10);
    analogWriteFrequency(PIN_MOTOR_FL_PWM, 146484);
    analogWriteFrequency(PIN_MOTOR_FR_PWM, 146484);
    analogWriteFrequency(PIN_MOTOR_BL_PWM, 146484);
    analogWriteFrequency(PIN_MOTOR_BR_PWM, 146484);

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

#ifndef DISABLE_DRIBBLER
    // Arm dribbler
    analogWrite(PIN_DRIBBLER_PWM, 128);
    delay(3000);
#endif
}

void Movement::updateHeadingController(const float angle) {
    _actualHeading = angle;
}

void Movement::setStop(bool maintainHeading) {
    if (maintainHeading) {
        // Since we want to maintain heading, let's just set velocity to 0
        velocity = 0;
    } else {
        // Since we don't need to maintain heading, we can brake the drivers
        _brake = true;
    }
}

// Sets the robot to move to a certain cartesian position on the field given the
// position of the two goals. We are also able to move to a target heading.
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
        moveToController.reset();
    }

    // Pack it into instructions for our update function
    _moveToActive = true;
    angle = relativeDestination.angle;
    if (relativeDestination.distance > MOVE_TO_PRECISION) {
        // The destination hasn't been reached
        // We square the error to make it decelerate linearly
        const auto controllerError = -powf(relativeDestination.distance, 2);
        velocity = moveToController.advance(controllerError);
        heading = (targetHeading - _actualHeading) *
                  fmin(relativeDestination.distance / _initialDistance, 1.0);
    } else {
        // The destination has basically been reached
        velocity = 0;
        heading = targetHeading;
    }
}

// Sets the velocity to decrease such that deceleration is linear from the start
// speed to the end speed, using a multiplier between 0.0 and 1.0.
void Movement::setLinearDecelerate(const int16_t startSpeed,
                                   const int16_t endSpeed,
                                   const float multiplier) {
    velocity =
        (startSpeed - endSpeed) * powf(fmin(multiplier, 1.0), 2) + endSpeed;
}

// Writes the current movement data.
void Movement::update() {
    if (_brake) {
        // Stop the motors
        // (I'd like to brake the drivers LOW if implemented in hardware)
        analogWrite(PIN_MOTOR_FL_PWM, 0);
        analogWrite(PIN_MOTOR_FR_PWM, 0);
        analogWrite(PIN_MOTOR_BL_PWM, 0);
        analogWrite(PIN_MOTOR_BR_PWM, 0);
        _brake = false;
        return;
    }

    // Convert polar to cartesian
    const auto x = sinfd(angle);
    const auto y = cosfd(angle);

    // Compute the speeds of the individual motors
    const auto transformSpeed = [this](float velocity_,
                                       float angularComponent) {
        return (int16_t)roundf(velocity_ * velocity + angularComponent);
    };
    // Find angular component
    headingController.setpoint = heading; // Update if target heading changed
    const auto angularVelocity = headingController.advance(_actualHeading);
    const auto angular = 0.25F * angularVelocity;
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

#ifndef DISABLE_DRIBBLER
    // Set dribbler motor speed
    analogWrite(PIN_DRIBBLER_PWM,
                dribble ? DRIBBLER_SPEED : DRIBBLER_ARM_SPEED);
#endif

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
