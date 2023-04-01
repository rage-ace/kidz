#include <Arduino.h>

#include "config.h"
#include "counter.h"
#include "pid.h"
#include "teensy/include/config.h"
#include "teensy/include/movement.h"
#include "teensy/include/sensors.h"

// Serial managers
auto MUXSerial = SerialManager(MUX_SERIAL, TEENSY_MUX_BAUD_RATE,
                               MUX_TX_PACKET_SIZE, MUX_TX_SYNC_START_BYTE,
                               MUX_TX_SYNC_END_BYTE, MUX_RX_PACKET_SIZE,
                               MUX_RX_SYNC_START_BYTE, MUX_RX_SYNC_END_BYTE);
auto TOFSerial = SerialManager(TOF_SERIAL, TEENSY_TOF_BAUD_RATE,
                               TOF_TX_PACKET_SIZE, TOF_TX_SYNC_START_BYTE,
                               TOF_TX_SYNC_END_BYTE, TOF_RX_PACKET_SIZE,
                               TOF_RX_SYNC_START_BYTE, TOF_RX_SYNC_END_BYTE);
auto IMUSerial = SerialManager(IMU_SERIAL, TEENSY_IMU_BAUD_RATE,
                               IMU_TX_PACKET_SIZE, IMU_TX_SYNC_START_BYTE,
                               IMU_TX_SYNC_END_BYTE, IMU_RX_PACKET_SIZE,
                               IMU_RX_SYNC_START_BYTE, IMU_RX_SYNC_END_BYTE);
auto CoralSerial = SerialManager(
    CORAL_SERIAL, TEENSY_CORAL_BAUD_RATE, CORAL_TX_PACKET_SIZE,
    CORAL_TX_SYNC_START_BYTE, CORAL_TX_SYNC_END_BYTE, CORAL_RX_PACKET_SIZE,
    CORAL_RX_SYNC_START_BYTE, CORAL_RX_SYNC_END_BYTE);

// IO
auto sensors = Sensors(MUXSerial, IMUSerial, TOFSerial, CoralSerial);
auto movement = Movement();

// Counters
auto debugPrintCounter = Counter();

void performCalibration() {
#ifdef CALIBRATE_IMU
    // CALIBRATE IMU
    // Set STM32 IMU to calibration mode
    const bool calibrating = true;
    char buf[sizeof(IMURXPayload)];
    memcpy(buf, &calibrating, sizeof(calibrating));
    IMUSerial.sendPacket(buf);

    movement.speed = 100;
    auto rotateCounter = Counter();
    auto speedCounter = Counter();
    while (1) {
        // Redirect IMU Serial to monitor
        IMUSerial.redirectBuffer(Serial);

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
        movement.drive();
    }
#endif

#ifdef CALIBRATE_ROBOT_ANGLE_CONTROLLER
    auto circleCounter = Counter();
    auto rotateCounter = Counter();
    auto controller =
        PIDController(0,         // Target angle
                      -256, 256, // Output limits
                      KP_ROBOT_ANGLE, KI_ROBOT_ANGLE, KD_ROBOT_ANGLE, // Gains
                      MIN_DT_ROBOT_ANGLE                              // min dt
        );
    while (10) {
        // Read all sensor values
        senors.read();

        // Keep straight
        if (sensors.robot.newData) {
            movement.angularVelocity = controller.advance(sensors.robot.angle);

            // Print controller info
            controller.debugPrint("Heading");
        }

        // Spin in a circle
        movement.speed = 80;
        if (circleCounter.millisElapsed(4)) {
            if (movement.angle == 18000)
                movement.angle = -17000;
            else
                movement.angle += 100;
        }
        if (rotateCounter.millisElapsed(50)) {
            if (controller.setpoint == 18000)
                controller.setpoint = -17000;
            else
                controller.setpoint += 100;
        }

        // Actuate outputs
        movement.drive();
    }
#endif
}

void performDebug() {
#ifdef DEBUG_MUX
    // Redirect MUX Serial to monitor
    MUXSerial.redirectBuffer(Serial);
#endif

#ifdef DEBUG_IMU
    // Redirect IMU Serial to monitor
    IMUSerial.redirectBuffer(Serial);
#endif

#ifdef DEBUG_TOF
    // Redirect TOF Serial to monitor
    TOFSerial.redirectBuffer(Serial);
#endif
#ifdef DEBUG_CORAL
    // Redirect Coral Serial to monitor
    CoralSerial.redirectBuffer(Serial);
#endif

#ifdef DEBUG_TEENSY
    if (debugPrintCounter.millisElapsed(100)) {
        // Print debug data
        if (sensors.line.exists())
            Serial.printf("Line %4d.%02dº %01d.%02d | ",
                          (int)sensors.line.angle,
                          abs((int)(sensors.line.angle * 100) % 100),
                          (int)sensors.line.depth,
                          abs((int)(sensors.line.depth * 100) % 100));
        else
            Serial.printf("Line             | ");
        if (sensors.robot.exists())
            Serial.printf("Robot Angle %4d.%02dº | ", (int)sensors.robot.angle,
                          abs((int)(sensors.robot.angle * 100) % 100));
        else
            Serial.printf("Robot Angle          | ");
        Serial.print("Bounds ");
        if (sensors.bounds.front != NO_BOUNDS)
            Serial.printf("F: %3d.%1d cm ", (int)sensors.bounds.front,
                          abs((int)(sensors.bounds.front * 10) % 10));
        else
            Serial.printf("F:          ");
        if (sensors.bounds.back != NO_BOUNDS)
            Serial.printf("B: %3d.%1d cm ", (int)sensors.bounds.back,
                          abs((int)(sensors.bounds.back * 10) % 10));
        else
            Serial.printf("B:          ");
        if (sensors.bounds.left != NO_BOUNDS)
            Serial.printf("L: %3d.%1d cm ", (int)sensors.bounds.left,
                          abs((int)(sensors.bounds.left * 10) % 10));
        else
            Serial.printf("L:          ");
        if (sensors.bounds.right != NO_BOUNDS)
            Serial.printf("R: %3d.%1d cm | ", (int)sensors.bounds.right,
                          abs((int)(sensors.bounds.right * 10) % 10));
        else
            Serial.printf("R:          | ");
        if (sensors.ball.exists())
            Serial.printf("Ball %4d.%02dº %4d.%02d cm | ",
                          (int)sensors.ball.angle,
                          abs((int)(sensors.ball.angle * 100) % 100),
                          (int)sensors.ball.distance,
                          abs((int)(sensors.ball.distance * 100) % 100));
        else
            Serial.printf("Ball                     | ");
        Serial.printf("Has Ball: %d | ", sensors.hasBall);
        Serial.printf("Other Robot: 0x%02X | ", sensors.otherRobot.testByte);
        Serial.println();
    }

    // // Print loop time
    // printLoopTime();

    // // Test motors
    // movement.angle = 0;
    // movement.speed = 0;
    // movement.angularVelocity = 200;

    // // Line Track
    // TODO
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

    // Move about the balls
    if (sensors.ball.exists()) {
        // We can't just move straight at the ball. We need to move behind it in
        // a curve. Here, we calculate the angle offset we need to achieve this.
        const auto angleOffset =
            // The angle offset is directly related to the angle of the ball,
            // but we constrain it to 90º because the robot should never move
            // in a range greater than 90º away from the ball, as it would be
            // moving away from the ball.
            constrain(sensors.ball.angle, -9000, 9000) *
            // The angle offset undergoes exponential decay. As the ball gets
            // closer to the robot, the robot moves more directly at the ball.
            fmin(BALL_MOVEMENT_A *
                     pow(exp(1), BALL_MOVEMENT_B * sensors.ball.distance),
                 1.0);
        // Then, we pack it into instructions for our drive function.
        movement.angle = sensors.ball.angle + angleOffset;
        // movement.speed = hasBall ? 280 : 240;
        movement.velocity = 100;
    } else {
        // If we don't see the ball, we don't move.
        // TODO: Find a better course of action.
        movement.angle = 0;
        movement.velocity = 0;
    }

    // Slow down near the wall
    // TODO

    // Avoid the lines
    // TODO

#ifdef DEBUG
    // Runs any debug code if the corresponding flag is defined
    performDebug();
#endif

    // Actuate outputs
    movement.drive();

    // Write to bluetooth
    auto bluetoothOutboundPayload = BluetoothPayload::create(0x01);
    char buf[sizeof(TOFRXPayload)];
    memcpy(buf, &bluetoothOutboundPayload, sizeof(bluetoothOutboundPayload));
    TOFSerial.sendPacket(buf);

    // Mark all sensor data as old
    sensors.markAsRead();
}
// ------------------------------- MAIN CODE END -------------------------------
