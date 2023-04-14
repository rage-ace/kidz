#include <Arduino.h>

#include "teensy/include/main.h"

void runStriker() {
    if (sensors.ball.value.exists() && !sensors.hasBall) {
        // Move behind the ball if we can see it somewhere else on the field
        moveBehindBall();
    } else if (sensors.hasBall) {
        // Move to the goal if we have the ball
        moveToOffensiveGoal();
    } else {
        // We can't find the ball
        if (sensors.robot.position.exists()) {
            // Return to center if we can determine our position
            movement.setMoveTo(sensors.robot.position.value, HOME, 0);
        } else {
            // Stay put
            movement.setStop();
            // TODO: think of a better idea
        }

        movement.dribble = false;
    }

    // Slow down near the wall
    avoidSideWalls();

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