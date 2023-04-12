#include <Arduino.h>

#include "teensy/include/main.h"

void performCalibration() {
#ifdef CALIBRATE_IMU
    // CALIBRATE IMU
    // Set STM32 IMU to calibration mode
    const bool calibrating = true;
    byte buf[sizeof(IMURXPayload)];
    memcpy(buf, &calibrating, sizeof(calibrating));
    // Send 100 times to ensure it gets through
    for (int i = 0; i < 100; i++) imuSerial.send(buf, sizeof(buf));

    movement.speed = 100;
    auto rotateCounter = Counter();
    auto speedCounter = Counter();
    while (1) {
        // Redirect IMU Serial to monitor
        while (imuSerial.available() > 0) Serial.write(imuSerial.read());

        // Drive motors to create magnetometer noise
        if (speedCounter.millisElapsed(100)) {
            movement.speed = -movement.speed;
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
        if (sensors.robot.angle.newData) {
            movement.updateHeadingController(sensors.robot.angle.value);
            // Print controller info
            movement.headingController.debugPrint("Heading");
        }

        // // Tune while stationary (rough tuning)
        // movement.heading = 0;
        // movement.setStop(true);
        // // Tune while moving (fine tuning)
        // movement.velocity = 300;
        // movement.angle = fmod(millis() / 5, 360);
        // Tune while moving and turning (finer tuning)
        movement.velocity = 300;
        movement.angle = fmod(millis() / 5, 360);
        const auto heading = fmod(millis() / 10, 360);
        movement.heading = heading > 180 ? heading - 360 : heading;

        // Actuate outputs
        movement.update();
    }
#endif
#ifdef CALIBRATE_BALL_CURVE
    while (1) {
        // Read all sensor values
        sensors.read();

        // Update heading loop
        if (sensors.robot.angle.newData)
            movement.updateHeadingController(sensors.robot.angle.value);

        if (sensors.ball.value.exists()) {
            // Move behind the ball if we can see it somewhere else on the field
            moveBehindBall();

            // Print debug output
            Serial.print("Ball Distance: ");
            Serial.print(sensors.ball.value.distance);
            Serial.print(" | Angle : ");
            Serial.print(sensors.ball.value.angle);
            Serial.print(" | Movement angle: ");
            Serial.print(movement.angle);
            Serial.print(" | Movement velocity: ");
            Serial.print(movement.velocity);
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
        if (sensors.robot.angle.newData)
            movement.updateHeadingController(sensors.robot.angle.value);

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

        Serial.print("Ball Angle: ");
        Serial.print(sensors.ball.value.angle);
        Serial.print(" | Line angle: ");
        Serial.print(sensors.line.angleBisector);
        Serial.print(" | Line depth: ");
        Serial.print(sensors.line.depth);
        Serial.print(" | Movement angle: ");
        Serial.print(movement.angle);
        Serial.print(" | Movement velocity: ");
        Serial.print(movement.velocity);
        Serial.println();

        // Actuate outputs
        movement.update();
    }
#endif
#ifdef CALIBRATE_GOAL_MOVEMENT
    while (1) {
        // Read all sensor values
        sensors.read();

        // Update heading loop
        if (sensors.robot.angle.newData)
            movement.updateHeadingController(sensors.robot.angle.value);

        if (sensors.goals.offensive.exists()) {
            // Move to the goal if we have the ball
            moveToOffensiveGoal();

            // Don't go past the line
            avoidLine();

            // Print debug output
            Serial.print("Goal Distance: ");
            Serial.print(sensors.goals.offensive.distance);
            Serial.print(" | Angle : ");
            Serial.print(sensors.goals.offensive.angle);
            Serial.print(" | Movement angle: ");
            Serial.print(movement.angle);
            Serial.print(" | Movement velocity: ");
            Serial.print(movement.velocity);
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