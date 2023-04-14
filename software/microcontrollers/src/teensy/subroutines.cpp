#include <Arduino.h>

#include "teensy/include/config.h"
#include "teensy/include/main.h"

void updateHeadingLoop() {
    if (sensors.robot.angle.newData)
        movement.updateHeadingController(sensors.robot.angle.value);
}

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
    if (sensors.ball.value.distance <= BALL_MOVEMENT_FACE_BALL_DISTANCE) {
        movement.heading =
            constrain(sensors.ball.value.angle, -BALL_MOVEMENT_MAX_HEADING,
                      BALL_MOVEMENT_MAX_HEADING);
    } else {
        movement.heading = 0;
    }
    movement.angle = sensors.ball.value.angle + angleOffset;
    // We have to decelerate as we approach the ball to not overshoot it, as
    // the FPS of the camera would not be able to keep up with the robot's
    // movement.
    movement.setLinearDecelerate(
        BALL_MOVEMENT_START_SPEED, BALL_MOVEMENT_END_SPEED,
        sensors.ball.value.distance / BALL_MOVEMENT_START_DECELERATING, true);
    movement.dribble = false;
}

void moveToOffensiveGoal() {
    // We use a similar curve algorithm for tracking the goal, but
    // with a constant multiplier instead of an exponential decay
    // multiplier as we want the robot to take a big orbit path,
    // such that it has more space to position itself before shooting.
    const auto angleOffset = constrain(sensors.goals.offensive.angle, -90, 90) *
                             GOAL_MOVEMENT_MULTIPLIER;
    // Take the distance to the goal as the minimum of that detected by the
    // camera and TOF, as the TOF can be read at a higher frequency, increasing
    // response time
    const auto goalDistance = sensors.bounds.front.valid()
                                  ? fminf(sensors.goals.offensive.distance,
                                          sensors.bounds.front.value)
                                  : sensors.goals.offensive.distance;

    // Then, we pack it into instructions for our update function.
    movement.heading = sensors.goals.offensive.angle;
    movement.angle = sensors.goals.offensive.angle + angleOffset;
    movement.setLinearDecelerate(
        GOAL_MOVEMENT_START_SPEED, GOAL_MOVEMENT_END_SPEED,
        goalDistance / GOAL_MOVEMENT_START_DECELERATING, true);
    movement.dribble = true;

    // If we're close enough to the goal, shoot
    if (sensors.goals.offensive.distance < GOAL_MOVEMENT_KICK_DISTANCE) {
        movement.dribble = false;
        movement.kick(); // this function has a cooldown built in, so it'll
                         // activate periodically if called repeatedly
    }

    // Besides avoiding the line, we'd also like to try to use the TOF to stop
    // us from going into the penalty area
    if (sensors.bounds.front.valid() &&
        sensors.bounds.front.value < PENALTY_AVOIDANCE_TOF_THRESHOLD) {
        movement.setLinearDecelerate(
            PENALTY_AVOIDANCE_START_SPEED, PENALTY_AVOIDANCE_END_SPEED,
            (sensors.bounds.front.value -
             (PENALTY_AVOIDANCE_TOF_THRESHOLD - PENALTY_AVOIDANCE_TOF_AREA)) /
                PENALTY_AVOIDANCE_TOF_AREA,
            true);
    }
}

void avoidSideWalls() {
    // Slow down near the left or right wall
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
        if (sensors.line.depth > LINE_AVOIDANCE_THRESHOLD) {
            // We're too far into the line, move away quickly
            movement.angle = sensors.line.angleBisector;
            movement.velocity =
                fmin(sensors.line.depth * LINE_AVOIDANCE_SPEED_MULTIPLIER,
                     LINE_AVOIDANCE_MAX_SPEED);
        } else if (sensors.ball.value.exists() && !sensors.hasBall) {
            // We're reasonably within the line, so let's try to line track
            // towards the ball

            bool approachingLeftBounds =
                sensors.bounds.left.value < sensors.bounds.right.value;
            movement.setMoveOnLineToBall(sensors.line.depth, sensors.ball.value,
                                         MOVE_ON_LINE_TO_BALL_TARGET_LINE_DEPTH,
                                         approachingLeftBounds);
        }
    } else if (!sensors.hasBall) {
        // Start line tracking a bit earlier within a TOF threshold near line
        if ((sensors.bounds.left.valid() &&
             sensors.bounds.left.value <= MOVE_ON_LINE_TO_BALL_TOF_THRESHOLD) &&
            (sensors.ball.value.exists() &&
             sensors.ball.value.angle + sensors.robot.angle.value < 0)) {
            movement.setMoveOnLineToBall(0, sensors.ball.value,
                                         MOVE_ON_LINE_TO_BALL_TARGET_LINE_DEPTH,
                                         true);
        }
        if ((sensors.bounds.right.valid() &&
             sensors.bounds.right.value <=
                 MOVE_ON_LINE_TO_BALL_TOF_THRESHOLD) &&
            (sensors.ball.value.exists() &&
             sensors.ball.value.angle + sensors.robot.angle.value > 0)) {
            movement.setMoveOnLineToBall(0, sensors.ball.value,
                                         MOVE_ON_LINE_TO_BALL_TARGET_LINE_DEPTH,
                                         false);
        }
    }
}