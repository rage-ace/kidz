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

        // Keep straight
        if (sensors.robot.newData) {
            movement.updateHeadingController(sensors.robot.angle);
            // Print controller info
            movement.headingController.debugPrint("Heading");
        }

        // // Tune while stationary (rough tuning)
        // movement.angle = 0;
        // movement.heading = 0;
        // // Tune while moving (fine tuning)
        // movement.velocity = 300;
        // movement.angle = fmod(millis() / 5, 360);
        // Tune while moving and turning (finer tuning, reduce period for finer)
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

        // Keep straight
        if (sensors.robot.newData)
            movement.updateHeadingController(sensors.robot.angle);

        if (sensors.ball.exists()) {
            // Move behind the ball if we can see it somewhere else on the field
            const auto multiplier =
                fmin(powf(exp(1),
                          BALL_MOVEMENT_DECAY * (BALL_MOVEMENT_MAX_CURVE -
                                                 sensors.ball.value.distance)),
                     1.0);
            const auto angleOffset =
                constrain(sensors.ball.value.angle, -90, 90) * multiplier;
            movement.heading =
                constrain(sensors.ball.value.angle, -BALL_MOVEMENT_MAX_HEADING,
                          BALL_MOVEMENT_MAX_HEADING);
            movement.angle = sensors.ball.value.angle + angleOffset;
            movement.setLinearDecelerate(
                BALL_MOVEMENT_START_SPEED, BALL_MOVEMENT_END_SPEED,
                sensors.ball.value.distance / BALL_MOVEMENT_START_DECELERATING);

            // Print debug output
            Serial.print("Ball Distance: ");
            Serial.print(sensors.ball.value.distance);
            Serial.print(" | Multiplier: ");
            Serial.print(multiplier);
            Serial.print(" | Angle : ");
            Serial.print(sensors.ball.value.angle);
            Serial.print(" | Angle Offset: ");
            Serial.print(angleOffset);
            Serial.print(" | Movement angle: ");
            Serial.print(movement.angle);
            Serial.print(" | Movement velocity: ");
            Serial.print(movement.velocity);
            Serial.println();

        } else {
            // Stop if we can't see the ball
            movement.setStop();

            // Print debug output
            Serial.println("Ball not found");
        }

        // Actuate outputs
        movement.update();
    }
#endif
}