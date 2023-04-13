#include <Arduino.h>

#include "teensy/include/main.h"

PIDController goalieTrackController = PIDController(
    0,                                                 // Target angle offset
    -GOALIE_TRACK_MAX_SPEED, GOALIE_TRACK_MAX_SPEED,   // Output limits
    KP_GOALIE_TRACK, KI_GOALIE_TRACK, KD_GOALIE_TRACK, // Gains
    MIN_DT_GOALIE_TRACK);

void runGoalie() {
    // Always face the ball whenever possible
    if (sensors.ball.value.exists())
        movement.heading = sensors.ball.value.angle;

    if (sensors.line.exists()) {
        // Since we're on the penalty area line, we line track towards the ball

        const auto error =
            sensors.ball.value.exists()
                // Move to the ball if we can see it
                ? -sensors.ball.value.angle
                // Otherwise, move to the center of the goal
                : -clipAngle(sensors.goals.defensive.angle - 180);
        const auto velocity = goalieTrackController.advance(error);
        movement.setLineTrack(sensors.line.depth, velocity > 0 ? 90 : -90,
                              GOALIE_TRACK_TARGET_LINE_DEPTH, false);
        movement.velocity = abs(velocity);
    } else {
        // We're no longer on the penalty area line, move back towards it

        if (sensors.goals.defensive.exists()) {
            // Move to the defensive goal
            if (sensors.goals.defensive.distance >= GOALIE_DISTANCE_THRESHOLD) {
                // If we're too far, move back towards the line slowly
                movement.angle = sensors.goals.defensive.angle;
                movement.setLinearDecelerate(GOALIE_RETURN_START_SPEED,
                                             GOALIE_RETURN_END_SPEED,
                                             (sensors.goals.defensive.distance -
                                              GOALIE_DISTANCE_THRESHOLD) /
                                                 GOALIE_DISTANCE_THRESHOLD);
            } else {
                // If we're too near, move back towards the line QUICKLY!
                movement.angle = sensors.goals.defensive.angle;
                movement.setLinearDecelerate(
                    GOALIE_QUICK_RETURN_START_SPEED,
                    GOALIE_QUICK_RETURN_END_SPEED,
                    (GOALIE_DISTANCE_THRESHOLD -
                     sensors.goals.defensive.distance) /
                        GOALIE_DISTANCE_THRESHOLD);
            }
        } else {
            // We can't find the defensive goal, so just move to the center
            movement.setMoveTo(sensors.robot.position.value, HOME, 0);

            // TODO: We should check with the other robot and coordinate role
            // switching accordingly
        }
    }
}
