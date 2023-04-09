#include <Arduino.h>
#include <PacketSerial.h>

#include "counter.h"
#include "pid.h"
#include "shared_config.h"
#include "teensy/include/config.h"
#include "teensy/include/movement.h"
#include "teensy/include/sensors.h"

// Serial
PacketSerial muxSerial;
PacketSerial tofSerial;
PacketSerial imuSerial;
PacketSerial coralSerial;

// IO
auto sensors = Sensors(muxSerial, tofSerial, imuSerial, coralSerial);
auto movement = Movement();

// Counters
auto debugPrintCounter = Counter();

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
            const auto multiplier = fmin(
                powf(exp(1), BALL_MOVEMENT_DECAY * (BALL_MOVEMENT_MAX_CURVE -
                                                    sensors.ball.distance)),
                1.0);
            const auto angleOffset =
                constrain(sensors.ball.angle, -90, 90) * multiplier;
            movement.heading =
                constrain(sensors.ball.angle, -BALL_MOVEMENT_MAX_HEADING,
                          BALL_MOVEMENT_MAX_HEADING);
            movement.angle = sensors.ball.angle + angleOffset;
            movement.setLinearDecelerate(
                BALL_MOVEMENT_START_SPEED, BALL_MOVEMENT_END_SPEED,
                sensors.ball.distance / BALL_MOVEMENT_START_DECELERATING);

            // Print debug output
            Serial.print("Ball Distance: ");
            Serial.print(sensors.ball.distance);
            Serial.print(" | Multiplier: ");
            Serial.print(multiplier);
            Serial.print(" | Angle : ");
            Serial.print(sensors.ball.angle);
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

void performDebug() {
#ifdef DEBUG_MUX
    // Redirect MUX Serial to monitor
    while (MUX_SERIAL.available() > 0) Serial.write(MUX_SERIAL.read());
#endif
#ifdef DEBUG_TOF
    // Redirect TOF Serial to monitor
    while (TOF_SERIAL.available() > 0) Serial.write(TOF_SERIAL.read());
#endif
#ifdef DEBUG_IMU
    // Redirect IMU Serial to monitor
    while (IMU_SERIAL.available() > 0) Serial.write(IMU_SERIAL.read());
#endif
#ifdef DEBUG_CORAL
    // Redirect Coral Serial to monitor
    while (CORAL_SERIAL.available() > 0) Serial.write(CORAL_SERIAL.read());
#endif

#ifdef DEBUG_TEENSY
    // Test motors
    // movement.dribble = true;
    // movement.angle = 0;
    // movement.velocity = 400;
    // movement.heading = 0;
    // movement.updateHeadingController(0);
    // movement.update();
    // while (1) {
    //     analogWrite(PIN_MOTOR_FL_PWM, 190);
    //     analogWrite(PIN_MOTOR_FR_PWM, 190);
    //     analogWrite(PIN_MOTOR_BL_PWM, 190);
    //     analogWrite(PIN_MOTOR_BR_PWM, 190);
    // }

    // // Line Track
    // TODO

    if (debugPrintCounter.millisElapsed(100)) {
        // Print debug data
        Serial.printf("Line ");
        if (sensors.line.exists())
            Serial.printf("%4d.%02dº ", (int)sensors.line.angleBisector,
                          abs((int)(sensors.line.angleBisector * 100) % 100));
        else
            Serial.printf("         ");
        Serial.printf("%01d.%02d | ", (int)sensors.line.depth,
                      abs((int)(sensors.line.depth * 100) % 100));
        if (sensors.robot.established())
            Serial.printf("Robot Angle %4d.%02dº | ", (int)sensors.robot.angle,
                          abs((int)(sensors.robot.angle * 100) % 100));
        else
            Serial.printf("Robot Angle          | ");
        // Serial.print("Bounds ");
        // if (sensors.bounds.front.established())
        //     Serial.printf("F: %3d.%1d cm ",
        //     (int)sensors.bounds.front.value,
        //                   abs((int)(sensors.bounds.front.value * 10)
        //                   % 10));
        // else
        //     Serial.printf("F:          ");
        if (sensors.bounds.back.established())
            Serial.printf("B: %3d.%1d cm ", (int)sensors.bounds.back.value,
                          abs((int)(sensors.bounds.back.value * 10) % 10));
        else
            Serial.printf("B:          ");
        // if (sensors.bounds.left.established())
        //     Serial.printf("L: %3d.%1d cm ",
        //     (int)sensors.bounds.left.value,
        //                   abs((int)(sensors.bounds.left.value * 10) %
        //                   10));
        // else
        //     Serial.printf("L:          ");
        // if (sensors.bounds.right.established())
        //     Serial.printf("R: %3d.%1d cm | ",
        //     (int)sensors.bounds.right.value,
        //                   abs((int)(sensors.bounds.right.value * 10)
        //                   % 10));
        // else
        //     Serial.printf("R:          | ");
        if (sensors.ball.exists())
            Serial.printf("Ball %4d.%02dº %4d.%02d cm | ",
                          (int)sensors.ball.angle,
                          abs((int)(sensors.ball.angle * 100) % 100),
                          (int)sensors.ball.distance,
                          abs((int)(sensors.ball.distance * 100) % 100));
        else
            Serial.printf("Ball                     | ");
        Serial.printf("Has Ball: %d | ", sensors.hasBall);
        // if (sensors.goals.exist())
        //     Serial.printf(
        //         "Goal O %4d.%02dº %4d.%02d cm D %4d.%02dº %4d.%02d cm
        //         |
        //         ", (int)sensors.goals.offensive.angle,
        //         abs((int)(sensors.goals.offensive.angle * 100) %
        //         100), (int)sensors.goals.offensive.distance,
        //         abs((int)(sensors.goals.offensive.distance * 100) %
        //         100), (int)sensors.goals.defensive.angle,
        //         abs((int)(sensors.goals.defensive.angle * 100) %
        //         100), (int)sensors.goals.defensive.distance,
        //         abs((int)(sensors.goals.defensive.distance * 100) %
        //         100));
        // else
        //     Serial.printf(
        //         "Goal                                             |
        //         ");
        Serial.printf("Other Robot: 0x%02X | ", sensors.otherRobot.testByte);
        Serial.printf("Drive %4d.%02dº at %4d.%02d | ", (int)movement.angle,
                      abs((int)(movement.angle * 100) % 100),
                      (int)movement.velocity,
                      abs((int)(movement.velocity * 100) % 100));
        Serial.println();
    }

    // // Print loop time
    // printLoopTime();

    // // Figure out lightgate threshold
    // Serial.println(analogRead(PIN_LIGHTGATE));
#endif
}

// ------------------------------ MAIN CODE START ------------------------------
void setup() {
    // Turn on the debug LED
    pinMode(PIN_LED_DEBUG, OUTPUT);
    digitalWriteFast(PIN_LED_DEBUG, HIGH);

    // Initialise monitor serial
    Serial.begin(MONITOR_BAUD_RATE);
    Serial.println("Initialising...");

    // Initialise motors and sensors and wait for completion
    movement.init();
    sensors.init();
#ifndef DEBUG_MUX
    muxSerial.setPacketHandler(
        [](const byte *buf, size_t size) { sensors.onMuxPacket(buf, size); });
#endif
#ifndef DEBUG_TOF
    tofSerial.setPacketHandler(
        [](const byte *buf, size_t size) { sensors.onTofPacket(buf, size); });
#endif
#ifndef DEBUG_IMU
    imuSerial.setPacketHandler(
        [](const byte *buf, size_t size) { sensors.onImuPacket(buf, size); });
#endif
#ifndef DEBUG_CORAL
    coralSerial.setPacketHandler(
        [](const byte *buf, size_t size) { sensors.onCoralPacket(buf, size); });
#endif
#ifndef DONT_WAIT_FOR_SUBPROCESSOR_INIT
    sensors.waitForSubprocessorInit();
#endif

    // Turn off the debug LED
    Serial.println("Initialisation complete");
    digitalWriteFast(PIN_LED_DEBUG, LOW);

#ifdef CALIBRATE
    // Runs the corresponding calibration if flag is defined
    performCalibration(); // this would probably be blocking
#endif
}

void loop() {
    // Read all sensor values
    sensors.read();

    // Keep straight
    if (sensors.robot.newData)
        movement.updateHeadingController(sensors.robot.angle);

    if (sensors.ball.exists() && !sensors.hasBall) {
        // Move behind the ball if we can see it somewhere else on the field

        // We can't just move straight at the ball. We need to move
        // behind it in a curve. Here, we calculate the angle offset we
        // need to achieve this.
        const auto angleOffset =
            // The angle offset is directly related to the angle of the
            // ball, but we constrain it to 90º because the robot should
            // never move in a range greater than 90º away from the
            // ball, as it would be moving away from the ball.
            constrain(sensors.ball.angle, -90, 90) *
            // The angle offset undergoes exponential decay. As the ball
            // gets closer to the robot, the robot moves more directly
            // at the ball.
            fmin(powf(exp(1), BALL_MOVEMENT_DECAY * (BALL_MOVEMENT_MAX_CURVE -
                                                     sensors.ball.distance)),
                 1.0);

        // Then, we pack it into instructions for our update function.
        // Try to keep straight as much as possible to ensure the robot has to
        // leave the field the least if the ball is near the boundary.
        movement.heading =
            constrain(sensors.ball.angle, -BALL_MOVEMENT_MAX_HEADING,
                      BALL_MOVEMENT_MAX_HEADING);
        movement.angle = sensors.ball.angle + angleOffset;
        // We have to decelerate as we approach the ball to not overshoot it, as
        // the FPS of the camera would not be able to keep up with the robot's
        // movement.
        movement.setLinearDecelerate(
            BALL_MOVEMENT_START_SPEED, BALL_MOVEMENT_END_SPEED,
            sensors.ball.distance / BALL_MOVEMENT_START_DECELERATING);
        movement.dribble = false;
    } else if (sensors.hasBall) {
        // Move to the goal if we have the ball

        // We use a similar curve algorithm for tracking the goal, but
        // with a constant multiplier instead of an exponential decay
        // multiplier as we want the robot to take a big orbit path,
        // such that it has more space to position itself before shooting.
        const auto angleOffset =
            constrain(sensors.goals.offensive.angle, -90, 90) *
            GOAL_MOVEMENT_MULTIPLIER;
        // Then, we pack it into instructions for our update function.
        movement.heading = sensors.goals.offensive.angle;
        movement.angle = sensors.goals.offensive.angle + angleOffset;
        movement.velocity = 400;
        // movement.dribble = true; // TODO: enable when dribbler works

        // If we're close enough to the goal, shoot
        if (sensors.goals.offensive.distance < 50) {
            movement.dribble = false;
            movement.kick(); // this function has a cooldown built in
        }
    } else {
        // We can't find the ball
        if (sensors.goals.exist()) {
            // Return to center if we can see both goals
            movement.setMoveTo(NEUTRAL_SPOT_CENTER, 0, sensors.goals);
        } else {
            // Stay put
            // TODO: think of a better idea
        }

        movement.dribble = false;
    }

    // Slow down near the wall
    if ((sensors.bounds.left.established() &&
         sensors.bounds.left.value <= WALL_AVOIDANCE_THRESHOLD) ||
        (sensors.bounds.right.established() &&
         sensors.bounds.right.value <= WALL_AVOIDANCE_THRESHOLD)) {
        const auto distanceToWall =
            fmin(sensors.bounds.left.value, sensors.bounds.right.value);
        movement.setLinearDecelerate(WALL_AVOIDANCE_START_SPEED,
                                     WALL_AVOIDANCE_END_SPEED,
                                     distanceToWall / WALL_AVOIDANCE_THRESHOLD);
    }

    // Avoid the lines
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

#ifdef DEBUG
    // Runs any debug code if the corresponding flag is defined
    performDebug();
#endif

    // Actuate outputs
    movement.update();

    // Write to bluetooth
    auto bluetoothOutboundPayload = BluetoothPayload::create(0x01);
    byte buf[sizeof(TOFRXPayload)];
    memcpy(buf, &bluetoothOutboundPayload, sizeof(bluetoothOutboundPayload));
    tofSerial.send(buf, sizeof(buf));

    // Mark all sensor data as old
    sensors.markAsRead();
}
// ------------------------------- MAIN CODE END -------------------------------
