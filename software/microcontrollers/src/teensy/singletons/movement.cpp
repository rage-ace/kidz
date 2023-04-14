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

    // Iniialise motor pins
    pinMode(PIN_MOTOR_FL_DIR, OUTPUT);
    pinMode(PIN_MOTOR_FR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_BL_DIR, OUTPUT);
    pinMode(PIN_MOTOR_BR_DIR, OUTPUT);
    analogWriteFrequency(PIN_MOTOR_FL_PWM, 146484);
    analogWriteFrequency(PIN_MOTOR_FR_PWM, 146484);
    analogWriteFrequency(PIN_MOTOR_BL_PWM, 146484);
    analogWriteFrequency(PIN_MOTOR_BR_PWM, 146484);
    pinMode(PIN_MOTOR_FL_PWM, OUTPUT);
    pinMode(PIN_MOTOR_FR_PWM, OUTPUT);
    pinMode(PIN_MOTOR_BL_PWM, OUTPUT);
    pinMode(PIN_MOTOR_BR_PWM, OUTPUT);

    // Initialise kicker pin
    pinMode(PIN_KICKER, OUTPUT);

#ifndef DISABLE_DRIBBLER
    // Initialise dribbler pins
    analogWriteFrequency(PIN_DRIBBLER_PWM, 1000);
    pinMode(PIN_DRIBBLER_PWM, OUTPUT);

    // Arm dribbler
    analogWrite(PIN_DRIBBLER_PWM, DRIBBLER_ARM_SPEED);
    delay(DRIBBLER_ARM_DURATION);
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
void Movement::setMoveTo(const Vector &robot, const Point &destination,
                         const float targetHeading) {
    const auto relativeDestination = -robot + Vector::fromPoint(destination);

    // Update move to state
    if (_lastDestination == nullptr || *_lastDestination != destination) {
        // A new move to routine just started
        delete _lastDestination;
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

void Movement::setLineTrack(const float lineDepth, const float targetLineAngle,
                            const float targetLineDepth,
                            const bool trackRightSide) {
    // Update line track state
    if (_lastTargetLineAngle == NAN ||
        _lastTargetLineAngle != targetLineAngle) {
        // A new line track routine just started
        _lastTargetLineAngle = targetLineAngle;
        lineTrackController.reset();
    }

    // Compute error to use in control loop
    const auto trackingForwards =
        targetLineAngle >= -90 && targetLineAngle <= 90;
    const auto controllerError = trackingForwards ^ trackRightSide
                                     ? lineDepth - targetLineDepth
                                     : targetLineDepth - lineDepth;

    // Pack it into instructions for our update function
    _lineTrackActive = true;
    // Scale the controller output linearly to velocity with a reference
    // velocity of 300 (we tune it at that)
    angle = targetLineAngle - _actualHeading +
            lineTrackController.advance(controllerError, (float)velocity / 300,
                                        true);
}

void Movement::setMoveOnLineToBall(const float lineDepth, const Vector &ball,
                                   const float targetLineDepth,
                                   const bool trackRightSide) {
    // Update move on line to ball state
    if (_lastTrackBallRightSide == nullptr ||
        *_lastTrackBallRightSide != trackRightSide) {
        // A new move on line to ball routine just started
        delete _lastTrackBallRightSide;
        _lastTrackBallRightSide = new bool(trackRightSide);
        moveOnLineToBallController.reset();
    }

    // Take a robot approaching the left boundary line as reference:
    // 1. If the ball is to the left/behind the robot, line track
    // backwards. Note that even if the ball is to the right of the
    // robot, we choose to line track if it's too far back, as the
    // robot's attempt to curve behind the ball would likely lie
    // beyond the line.
    // 2. If the ball is to the left/in front of the robot, line track
    // forwards.
    if (trackRightSide
            ? (ball.angle <= 0 || ball.angle > LINE_TRACKING_NO_CURVE_THRESHOLD)
            : (ball.angle >= 0 ||
               ball.angle < -LINE_TRACKING_NO_CURVE_THRESHOLD)) {
        // We should be line tracking here

        const auto adjustedBallAngle = clipAngle(ball.angle - _actualHeading);
        const auto ballDistance =
            (trackRightSide
                 ? (adjustedBallAngle > -90 && adjustedBallAngle <= 0)
                 : (adjustedBallAngle < 90 && adjustedBallAngle >= 0))
                ? ball.distance
                : -ball.distance;
        const auto correction =
            moveOnLineToBallController.advance(ballDistance);
        velocity = abs(correction);
        setLineTrack(lineDepth, correction < 0 ? 0 : 180, targetLineDepth,
                     trackRightSide);
    } else {
        // Since the ball is within a range where we can feasible curve
        // to it, let's just do that.

        // Don't override ball curve values, so don't do anything
    }

    _moveOnLineToBallActive = true;
}

// Sets the velocity to decrease such that deceleration is linear from the start
// speed to the end speed, using a multiplier between 0.0 and 1.0.
void Movement::setLinearDecelerate(const int16_t startSpeed,
                                   const int16_t endSpeed,
                                   const float multiplier,
                                   const bool replaceVelocity) {
    const auto newVelocity =
        (startSpeed - endSpeed) * powf(fmin(multiplier, 1.0), 2) + endSpeed;
    velocity = replaceVelocity ? newVelocity : fmin(velocity, newVelocity);
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
    headingController.updateSetpoint(heading);
    // Scale the controller output linearly to velocity with a reference
    // velocity of 300 (we tune it at that)
    auto scaler =
        velocity == 0 ? STATIONARY_SCALER_ROBOT_ANGLE : (float)velocity / 300;
    const auto angularVelocity =
        headingController.advance(_actualHeading, scaler, true);
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

    // Invalidate line track routine (it can be reactivated by calling
    // setLineTrack)
    if (!_lineTrackActive) { _lastTargetLineAngle = NAN; }
    _lineTrackActive = false;

    // Invalidate move on line to ball routine (it can be reactivated by calling
    // setMoveOnLineToBall)
    if (!_moveOnLineToBallActive) {
        delete _lastTrackBallRightSide;
        _lastTrackBallRightSide = nullptr;
    }
    _moveOnLineToBallActive = false;
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
