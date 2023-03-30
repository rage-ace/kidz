#include <Arduino.h>

#include "angle.h"
#include "config.h"
#include "counter.h"
#include "pid.h"
#include "teensy/include/config.h"

// State
Line line;                // Read from STM32 MUX
RobotAngle robotAngle;    // Read from STM32 IMU, -179(.)99° to +180(.)00°
int16_t robotAngleOffset; // Reset on Teensy init
Bounds bounds;            // Read from STM32 TOF, 000(.)0 cm to 400(.)0 cm
BluetoothPayload bluetoothInboundPayload; // Read from STM32 TOF
Ball ball;                                // Read from coral
bool hasBall = false;
struct Movement {
    int16_t angle = 0;           // -179(.)99º to +180(.)00º
    int16_t speed = 0;           // ±35 to ±256
    int16_t angularVelocity = 0; // ~±140 to ~±1024
} movement;

// Controllers
auto robotAngleController =
    PIDController(0,         // Target angle
                  -256, 256, // Output limits
                  KP_ROBOT_ANGLE, KI_ROBOT_ANGLE, KD_ROBOT_ANGLE, // Gains
                  MIN_DT_ROBOT_ANGLE                              // min dt
    );

// Counters
auto debugPrintCounter = Counter();

// Writes the current movement data to the motors.
void drive() {
    // Convert polar to cartesian
    const auto x = sinf((float)movement.angle / 100 * DEG_TO_RAD);
    const auto y = cosf((float)movement.angle / 100 * DEG_TO_RAD);

    // Compute the speeds of the individual motors
    const auto transformSpeed = [](float speed, float angularComponent) {
        return (int16_t)roundf(speed * movement.speed + angularComponent);
    };
    // Find angular component
    const auto angular = ANGULAR_VELOCITY_MULTIPLIER * movement.angularVelocity;
    // Compute speeds
    const int16_t FLSpeed = transformSpeed(x * COS45 + y * SIN45, angular);
    const int16_t FRSpeed = transformSpeed(x * -COS45 + y * SIN45, -angular);
    const int16_t BLSpeed = transformSpeed(x * -COS45 + y * SIN45, +angular);
    const int16_t BRSpeed = transformSpeed(x * COS45 + y * SIN45, -angular);

    // Constrain motor speed
    auto constrainSpeed = [](int16_t speed) {
        // If the speed is below the stall speed, don't bother moving
        if (abs(speed) < DRIVE_STALL_SPEED) return 0;
        return min(abs(speed), DRIVE_MAX_SPEED);
    };

    // Set the motor directions and speeds
    digitalWriteFast(PIN_MOTOR_FL_DIR, FLSpeed > 0 ? HIGH : LOW);
    digitalWriteFast(PIN_MOTOR_FR_DIR, FRSpeed > 0 ? HIGH : LOW);
    digitalWriteFast(PIN_MOTOR_BL_DIR, BLSpeed > 0 ? HIGH : LOW);
    digitalWriteFast(PIN_MOTOR_BR_DIR, BRSpeed > 0 ? LOW : HIGH);
    analogWrite(PIN_MOTOR_FL_PWM, constrainSpeed(FLSpeed));
    analogWrite(PIN_MOTOR_FR_PWM, constrainSpeed(FRSpeed));
    analogWrite(PIN_MOTOR_BL_PWM, constrainSpeed(BLSpeed));
    analogWrite(PIN_MOTOR_BR_PWM, constrainSpeed(BRSpeed));
}

void waitForSubprocessorInit() {
#ifndef DEBUG_MUX
    // Wait for STM32 MUX to initialise
    Serial.println("Waiting for STM32 MUX to initialise...");
    MUXSerial.waitForPacket();
#endif

#ifndef DEBUG_TOF
    // Wait for STM32 TOF to initialise
    Serial.println("Waiting for STM32 TOF to initialise...");
    TOFSerial.waitForPacket();
#endif

#ifndef DEBUG_IMU
    // Wait for STM32 IMU to initialise and get the angle offset
    Serial.println("Waiting for STM32 IMU to initialise...");
    IMUTXPayload imuTXPayload;
    IMUSerial.waitForPacket(&imuTXPayload);
    // We can assume the value has already stabilised as the Teensy can be
    // reset separately from the IMU, after the IMU has been on for a while
    robotAngleOffset = imuTXPayload.robotAngle.angle;
#else
    robotAngleOffset = 0;
#endif

    // NOTE: We do not wait for the coral here.
}

void readSensors() {
#ifndef DEBUG_MUX
    MUXTXPayload muxTXPayload;
    if (MUXSerial.readPacket(&muxTXPayload)) line = muxTXPayload.line;
#endif

#ifndef DEBUG_IMU
    // Read attitude data from STM32 IMU
    IMUTXPayload imuTXPayload;
    if (IMUSerial.readPacket(&imuTXPayload)) {
        robotAngle = imuTXPayload.robotAngle.withOffset(robotAngleOffset);
    }
#endif

#ifndef DEBUG_TOF
    // Read bounds data from STM32 TOF
    TOFTXPayload tofTXPayload;
    if (TOFSerial.readPacket(&tofTXPayload)) {
        bounds = tofTXPayload.bounds;
        bluetoothInboundPayload = tofTXPayload.bluetoothInboundPayload;
    }
#endif

    // Read ball data from coral
    CoralTXPayload coralTXPayload;
    if (CoralSerial.readPacket(&coralTXPayload)) ball = coralTXPayload.ball;

    // Read lightgate
    hasBall = analogRead(PIN_LIGHTGATE) < LIGHTGATE_THRESHOLD;
}

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
        drive();
    }
#endif

#ifdef CALIBRATE_ROBOT_ANGLE_CONTROLLER
    auto circleCounter = Counter();
    auto rotateCounter = Counter();
    while (10) {
        // Read all sensor values
        readSensors();

        // Keep straight
        if (robotAngle.exists() && robotAngle.newData) {
            movement.angularVelocity =
                robotAngleController.advance(robotAngle.angle);

            // Print controller info
            robotAngleController.debugPrint("Robot Angle");
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
            if (robotAngleController.setpoint == 18000)
                robotAngleController.setpoint = -17000;
            else
                robotAngleController.setpoint += 100;
        }

        // Actuate outputs
        drive();
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
        if (line.exists())
            Serial.printf("Line %4d.%02dº %01d.%02d | ", line.angle / 100,
                          abs(line.angle % 100), line.size / 100,
                          line.size % 100);
        else
            Serial.printf("Line             | ");
        if (robotAngle.exists())
            Serial.printf("Robot Angle %4d.%02dº | ", robotAngle.angle / 100,
                          abs(robotAngle.angle % 100));
        else
            Serial.printf("Robot Angle          | ");
        Serial.print("Bounds ");
        if (bounds.front != NO_BOUNDS)
            Serial.printf("F: %3d.%1d cm ", bounds.front / 10,
                          bounds.front % 10);
        else
            Serial.printf("F:          ");
        if (bounds.back != NO_BOUNDS)
            Serial.printf("B: %3d.%1d cm ", bounds.back / 10, bounds.back % 10);
        else
            Serial.printf("B:          ");
        if (bounds.left != NO_BOUNDS)
            Serial.printf("L: %3d.%1d cm ", bounds.left / 10, bounds.left % 10);
        else
            Serial.printf("L:          ");
        if (bounds.right != NO_BOUNDS)
            Serial.printf("R: %3d.%1d cm | ", bounds.right / 10,
                          bounds.right % 10);
        else
            Serial.printf("R:          | ");
        if (ball.exists())
            Serial.printf("Ball %4d.%02dº %4d.%02d cm | ", ball.angle / 100,
                          abs(ball.angle % 100), ball.distance / 100,
                          ball.distance % 100);
        else
            Serial.printf("Ball                     | ");
        Serial.printf("Has Ball: %d | ", hasBall);
        Serial.printf("BT Inbound: 0x%02X | ",
                      bluetoothInboundPayload.testByte);
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

    // Initialise pins
    pinMode(PIN_MOTOR_FL_DIR, OUTPUT);
    pinMode(PIN_MOTOR_FR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_BL_DIR, OUTPUT);
    pinMode(PIN_MOTOR_BR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_FL_PWM, OUTPUT);
    pinMode(PIN_MOTOR_FR_PWM, OUTPUT);
    pinMode(PIN_MOTOR_BL_PWM, OUTPUT);
    pinMode(PIN_MOTOR_BR_PWM, OUTPUT);

    analogReadResolution(12);

    // Values from https://www.pjrc.com/teensy/td_pulse.html
    // (based on F_CPU_ACTUAL = 600 MHz)
    // analogWriteResolution(12); // TODO: Debug with Multimeter
    // analogWriteFrequency(PIN_MOTOR_FL_PWM, 36621);
    // analogWriteFrequency(PIN_MOTOR_FR_PWM, 36621);
    // analogWriteFrequency(PIN_MOTOR_BL_PWM, 36621);
    // analogWriteFrequency(PIN_MOTOR_BR_PWM, 36621);

    // Initialise serial
    MUXSerial.setup(true);
    IMUSerial.setup(true);
    TOFSerial.setup(true);
    CoralSerial.setup(true);

#ifndef DONT_WAIT_FOR_SUBPROCESSOR_INT
    waitForSubprocessorInit(); // this is obviously blocking
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
    readSensors();

    // Keep straight
    if (robotAngle.exists() && robotAngle.newData)
        movement.angularVelocity =
            robotAngleController.advance(robotAngle.angle);

    // Move about the balls
    if (ball.exists()) {
        // We can't just move straight at the ball. We need to move behind it in
        // a curve. Here, we calculate the angle offset we need to achieve this.
        const auto angleOffset =
            // The angle offset is directly related to the angle of the ball,
            // but we constrain it to 90º because the robot should never move
            // in a range greater than 90º away from the ball, as it would be
            // moving away from the ball.
            constrain(ball.angle, -9000, 9000) *
            // The angle offset undergoes exponential decay. As the ball gets
            // closer to the robot, the robot moves more directly at the ball.
            fmin(BALL_MOVEMENT_A * pow(exp(1), BALL_MOVEMENT_B * ball.distance),
                 1.0);
        // Then, we pack it into instructions for our drive function.
        movement.angle = ball.angle + angleOffset;
        // movement.speed = hasBall ? 280 : 240;
        movement.speed = 100;
    } else {
        // If we don't see the ball, we don't move.
        // TODO: Find a better course of action.
        movement.angle = 0;
        movement.speed = 0;
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
    drive();

    // Write to bluetooth
    auto bluetoothOutboundPayload = BluetoothPayload::create(0x01);
    char buf[sizeof(TOFRXPayload)];
    memcpy(buf, &bluetoothOutboundPayload, sizeof(bluetoothOutboundPayload));
    TOFSerial.sendPacket(buf);

    // Mark all sensor data as old
    line.newData = false;
    robotAngle.newData = false;
    bounds.newData = false;
    ball.newData = false;
}
// ------------------------------- MAIN CODE END -------------------------------
