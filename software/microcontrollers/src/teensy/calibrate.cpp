#include <Arduino.h>

#include "counter.h"
#include "teensy/include/main.h"
#include "teensy/include/movement.h"

void performCalibration() {
#ifdef CALIBRATE_IMU
    // CALIBRATE IMU
    // Set STM32 IMU to calibration mode
    const bool calibrating = true;
    byte buf[sizeof(IMURXPayload)];
    memcpy(buf, &calibrating, sizeof(calibrating));
    // Send 100 times to ensure it gets through
    for (int i = 0; i < 100; i++) imuSerial.send(buf, sizeof(buf));
    Serial.println("Calibrating");

    movement.velocity = 100;
    auto rotateCounter = Counter();
    auto speedCounter = Counter();
    while (1) {
        // Redirect IMU Serial to monitor
        while (IMU_SERIAL.available() > 0) Serial.write(IMU_SERIAL.read());

        // Drive motors to create magnetometer noise
        if (speedCounter.millisElapsed(100)) {
            movement.velocity = -movement.velocity;
        }
        if (rotateCounter.millisElapsed(1)) {
            if (movement.angle == 18000)
                movement.angle = -17000;
            else
                movement.angle += 100;
        }
        movement.update();
    }
#endif
#ifdef CALIBRATE_MUX
    // CALIBRATE MUX
    // Set STM32 MUX to calibration mode
    const bool calibrating = true;
    byte buf[sizeof(MUXRXPayload)];
    memcpy(buf, &calibrating, sizeof(calibrating));
    // Send 100 times to ensure it gets through
    for (int i = 0; i < 100; i++) muxSerial.send(buf, sizeof(buf));

    while (1) {
        // Redirect MUX Serial to monitor
        while (MUX_SERIAL.available() > 0) Serial.write(MUX_SERIAL.read());
    }
#endif
#ifdef CALIBRATE_ROBOT_ANGLE
    while (1) {
        // Read all sensor values
        sensors.read();

        // Update heading loop
        updateHeadingLoop();
        if (sensors.robot.angle.newData)
            movement.headingController.debugPrint("Heading");

        // Tune while stationary (rough tuning)
        movement.heading = 0;
        movement.setStop(true);
        // // Tune while moving (fine tuning)
        // movement.velocity = 300;
        // movement.angle = fmod(millis() / 5, 360);
        // // Tune while moving and turning (finer tuning)
        // movement.velocity = 300;
        // movement.angle = fmod(millis() / 5, 360);
        // const auto heading = fmod(millis() / 100, 360);
        // movement.heading = heading > 180 ? heading - 360 : heading;

        // Actuate outputs
        movement.update();
    }
#endif
#ifdef CALIBRATE_BALL_CURVE
    while (1) {
        // Read all sensor values
        sensors.read();

        // Update heading loop
        updateHeadingLoop();

        if (sensors.ball.value.exists()) {
            // Move behind the ball if we can see it somewhere else on the field
            moveBehindBall();
            // Don't go past the line
            avoidLine();

            // Print debug output
            Serial.print("Ball Distance: ");
            Serial.print(sensors.ball.value.distance);
            Serial.print(" | Angle : ");
            Serial.print(sensors.ball.value.angle);
            Serial.print(" | Move in ");
            Serial.print(movement.angle);
            Serial.print("º at ");
            Serial.print(movement.velocity);
            Serial.print(" facing ");
            Serial.print(movement.heading);
            Serial.print("º");
            Serial.println();

        } else {
            // Stop if we can't see the ball
            movement.heading = 0;
            movement.setStop();

            // Print debug output
            Serial.println("Ball not found");
        }

        // Actuate outputs
        movement.update();
    }
#endif
#ifdef CALIBRATE_AVOIDANCE
    while (1) {
        // Read all sensor values
        sensors.read();

        // Update heading loop
        updateHeadingLoop();

        if (sensors.ball.value.exists()) {
            // Move behind the ball if we can see it somewhere else on the field
            moveBehindBall();
        } else {
            // Stop if we can't see the ball
            movement.heading = 0;
            movement.setStop();
        }

        // Slow down near the left or right wall
        avoidSideWalls();
        // Don't go past the line
        avoidLine();

        Serial.print("Ball: ");
        Serial.print(sensors.ball.value.angle);
        Serial.print("º ");
        Serial.print(sensors.ball.value.distance);
        Serial.print(" cm | Line: ");
        Serial.print(sensors.line.angleBisector);
        Serial.print("º ");
        Serial.print(sensors.line.depth);
        Serial.print(" | Move in ");
        Serial.print(movement.angle);
        Serial.print("º at ");
        Serial.print(movement.velocity);
        Serial.print(" facing ");
        Serial.print(movement.heading);
        Serial.print("º");
        Serial.println();

        // movement.moveOnLineToBallController.debugPrint("Move to Ball");

        // Actuate outputs
        movement.update();
    }
#endif
#ifdef CALIBRATE_LINE_TRACK
    while (1) {
        // Read all sensor values
        sensors.read();

        // Update heading loop
        updateHeadingLoop();

        // Track the line
        movement.setLineTrack(sensors.line.depth, 0,
                              LINE_AVOIDANCE_THRESHOLD / 2, false);
        movement.velocity = 300;

        // Print debug output
        Serial.print("Move in ");
        Serial.print(movement.angle);
        Serial.print("º at ");
        Serial.print(movement.velocity);
        Serial.print(" facing ");
        Serial.print(movement.heading);
        Serial.print("º ");
        movement.lineTrackController.debugPrint("Line Track");

        // Actuate outputs
        movement.update();
    }
#endif
#ifdef CALIBRATE_GOAL_MOVEMENT
    while (1) {
        // Read all sensor values
        sensors.read();

        // Update heading loop
        updateHeadingLoop();

        if (sensors.goals.offensive.exists()) {
            // Move to the goal if we have the ball
            moveToOffensiveGoal();

            // Don't go past the line
            avoidLine();

            // Print debug output
            Serial.print("Goal: ");
            Serial.print(sensors.goals.offensive.distance);
            Serial.print(" cm at ");
            Serial.print(sensors.goals.offensive.angle);
            Serial.print("º | Front TOF: ");
            Serial.print(sensors.bounds.front.value);
            Serial.print(" cm | Move in ");
            Serial.print(movement.angle);
            Serial.print("º at ");
            Serial.print(movement.velocity);
            Serial.print(" facing ");
            Serial.print(movement.heading);
            Serial.print("º");
            Serial.println();
        } else {
            movement.dribble = false;

            // Print debug output
            Serial.println("Goal not found");
        }

        // Actuate outputs
        movement.update();
    }
#endif
}