#include <Arduino.h>

#include "teensy/include/config.h"
#include "teensy/include/main.h"

void moveBehindBall() {
    // We can't just move straight at the ball. We need to move
    // behind it in a curve. Here, we calculate the angle offset we
    // need to achieve this.
    const auto angleOffset =
        // The angle offset is directly related to the angle of the
        // ball, but we constrain it to 90º because the robot should
        // never move in a range greater than 90º away from the
        // ball, as it would be moving away from the ball.
        constrain(sensors.ball.value.angle, -90, 90) *
        // The angle offset undergoes exponential decay. As the ball
        // gets closer to the robot, the robot moves more directly
        // at the ball.
        fmin(powf(exp(1), BALL_MOVEMENT_DECAY * (BALL_MOVEMENT_MAX_CURVE -
                                                 sensors.ball.value.distance)),
             1.0);

    // Then, we pack it into instructions for our update function.
    // Try to keep straight as much as possible to ensure the robot has to
    // leave the field the least if the ball is near the boundary.
    movement.heading =
        constrain(sensors.ball.value.angle, -BALL_MOVEMENT_MAX_HEADING,
                  BALL_MOVEMENT_MAX_HEADING);
    movement.angle = sensors.ball.value.angle + angleOffset;
    // We have to decelerate as we approach the ball to not overshoot it, as
    // the FPS of the camera would not be able to keep up with the robot's
    // movement.
    movement.setLinearDecelerate(
        BALL_MOVEMENT_START_SPEED, BALL_MOVEMENT_END_SPEED,
        sensors.ball.value.distance / BALL_MOVEMENT_START_DECELERATING);
    movement.dribble = false;
}

void moveToGoal() {
    // Move to the goal if we have the ball

    // We use a similar curve algorithm for tracking the goal, but
    // with a constant multiplier instead of an exponential decay
    // multiplier as we want the robot to take a big orbit path,
    // such that it has more space to position itself before shooting.
    const auto angleOffset = constrain(sensors.goals.offensive.angle, -90, 90) *
                             GOAL_MOVEMENT_MULTIPLIER;
    // Then, we pack it into instructions for our update function.
    movement.heading = sensors.goals.offensive.angle;
    movement.angle = sensors.goals.offensive.angle + angleOffset;
    movement.setLinearDecelerate(
        GOAL_MOVEMENT_START_SPEED, GOAL_MOVEMENT_END_SPEED,
        sensors.goals.offensive.distance / GOAL_MOVEMENT_START_DECELERATING);
    movement.dribble = true;

    // If we're close enough to the goal, shoot
    if (sensors.goals.offensive.distance < 50) {
        movement.dribble = false;
        movement.kick(); // this function has a cooldown built in, so it'll
                         // activate periodically if called repeatedly
    }
}

void avoidWall() {
    // Slow down near the wall
    if ((sensors.bounds.left.valid() &&
         sensors.bounds.left.value <= WALL_AVOIDANCE_THRESHOLD) ||
        (sensors.bounds.right.valid() &&
         sensors.bounds.right.value <= WALL_AVOIDANCE_THRESHOLD)) {
        const auto distanceToWall =
            fmin(sensors.bounds.left.value, sensors.bounds.right.value);
        movement.setLinearDecelerate(WALL_AVOIDANCE_START_SPEED,
                                     WALL_AVOIDANCE_END_SPEED,
                                     distanceToWall / WALL_AVOIDANCE_THRESHOLD);
    }
}

void avoidLine() {
    if (sensors.line.exists()) {
        if (sensors.line.depth < LINE_AVOIDANCE_THRESHOLD) {
            // Stop inside the line to prevent jerking
            movement.setStop();
            // TODO: Line track towards the ball
        } else {
            // Move away from the line
            movement.angle = sensors.line.angleBisector;
            movement.velocity =
                fmin(sensors.line.depth * LINE_AVOIDANCE_SPEED_MULTIPLIER,
                     LINE_AVOIDANCE_MAX_SPEED);
        }
    }
}

void runStriker() {
    if (sensors.ball.value.exists() && !sensors.hasBall) {
        // Move behind the ball if we can see it somewhere else on the field
        moveBehindBall();
    } else if (sensors.hasBall) {
        // Move to the goal if we have the ball
        moveToGoal();
    } else {
        // We can't find the ball
        if (sensors.robot.position.exists()) {
            // Return to center if we can determine our position
            movement.setMoveTo(sensors.robot.position.value,
                               NEUTRAL_SPOT_CENTER, 0);
        } else {
            // Stay put
            movement.setStop();
            // TODO: think of a better idea
        }

        movement.dribble = false;
    }

    // Slow down near the wall
    avoidWall();

    // Avoid the lines
    avoidLine();

    // Write to bluetooth
    // TODO: Update payload accordingly
    auto bluetoothOutboundPayload =
        BluetoothPayload::create(true, {0, 0}, {0, 0});
    byte buf[sizeof(TOFRXPayload)];
    memcpy(buf, &bluetoothOutboundPayload, sizeof(bluetoothOutboundPayload));
    tofSerial.send(buf, sizeof(buf));
}