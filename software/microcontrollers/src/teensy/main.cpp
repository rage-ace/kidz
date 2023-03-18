#include <Arduino.h>

#include "config.h"
#include "pid.h"
#include "teensy/include/config.h"
#include "util.h"

// State
struct Line line;       // Read from STM32 MUX
uint16_t bearing = 0;   // Read from STM32 IMU, 000(.)00° to 359(.)99°
uint16_t bearingOffset; // Reset on Teensy init
Bounds bounds;          // Read from STM32 TOF, 000(.)0 cm to 400(.)0 cm
BluetoothPayload bluetoothInboundPayload; // Read from STM32 TOF
struct Movement {
    int16_t angle = 0;           // -180(.)00º to +180(.)00º
    int16_t speed = 0;           // ±50 to ±1024
    int16_t angularVelocity = 0; // ±200 to ±4096
} movement;

// Controllers
PIDController bearingController =
    PIDController(KP_BEARING, KI_BEARING, KD_BEARING, // Gains
                  -4096, 4096,                        // angularVelocity bounds
                  0                                   // Target bearing
    );

// Writes the current movement data to the motors.
void drive() {
    // Convert polar to cartesian
    const auto x = sinf(movement.angle * DEG_TO_RAD);
    const auto y = cosf(movement.angle * DEG_TO_RAD);

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

    // Set the motor directions and speeds
    digitalWriteFast(PIN_MOTOR_FL_DIR, FLSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(PIN_MOTOR_FR_DIR, FRSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(PIN_MOTOR_BL_DIR, BLSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(PIN_MOTOR_BR_DIR, BRSpeed > 0 ? HIGH : LOW);
    analogWrite(PIN_MOTOR_FL_PWM,
                constrain(abs(FLSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
    analogWrite(PIN_MOTOR_FR_PWM,
                constrain(abs(FRSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
    analogWrite(PIN_MOTOR_BL_PWM,
                constrain(abs(BLSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
    analogWrite(PIN_MOTOR_BR_PWM,
                constrain(abs(BRSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
}

void readSensors() {
#ifndef DEBUG_MUX
    // Read line data from STM32 MUX
    MUXTXPayload muxTXPayload;
    if (readPacket(MUX_SERIAL, &muxTXPayload, MUX_TX_PACKET_SIZE,
                   MUX_TX_SYNC_START_BYTE, MUX_TX_SYNC_END_BYTE))
        line = muxTXPayload.line;
#endif

#ifndef DEBUG_IMU
    // Read attitude data from STM32 IMU
    IMUTXPayload imuTXPayload;
    if (readPacket(IMU_SERIAL, &imuTXPayload, IMU_TX_PACKET_SIZE,
                   IMU_TX_SYNC_START_BYTE, IMU_TX_SYNC_END_BYTE)) {
        const int32_t _bearing = imuTXPayload.bearing - bearingOffset;
        bearing = _bearing >= 0 ? _bearing : _bearing + 36000;
    }
#endif

#ifndef DEBUG_TOF
    // Read bounds data from STM32 TOF
    TOFTXPayload tofTXPayload;
    if (readPacket(TOF_SERIAL, &tofTXPayload, TOF_TX_PACKET_SIZE,
                   TOF_TX_SYNC_START_BYTE, TOF_TX_SYNC_END_BYTE)) {
        bounds = tofTXPayload.bounds;
        bluetoothInboundPayload = tofTXPayload.bluetoothInboundPayload;
    }
#endif

    // Read ball data from coral
    // TODO
}

void performCalibration() {
#ifdef CALIBRATE_IMU
    // CALIBRATE IMU
    // Set STM32 IMU to calibration mode
    const bool calibrating = true;
    char buf[sizeof(IMURXPayload)];
    memcpy(buf, &calibrating, sizeof(calibrating));
    sendPacket(IMU_SERIAL, buf, IMU_RX_PACKET_SIZE, IMU_RX_SYNC_START_BYTE,
               IMU_RX_SYNC_END_BYTE);

    int16_t angle = 0;
    uint32_t time = 0;
    while (1) {
        // Redirect IMU Serial to monitor
        if (IMU_SERIAL.available() > 0) Serial.print(char(IMU_SERIAL.read()));

        if (millis() - time >= 10) {
            time = millis();

            // Drive motors to create magnetometer noise
            movement.angle = angle;
            movement.speed = 512;
            drive();

            // Update angle counter
            ++angle;
            if (angle == 180) angle = -179;
        }
    }
#endif
}

void performDebug() {
#ifdef DEBUG_MUX
    // Redirect MUX Serial to monitor
    if (MUX_SERIAL.available() > 0) Serial.print(char(MUX_SERIAL.read()));
#endif

#ifdef DEBUG_IMU
    // Redirect IMU Serial to monitor
    if (IMU_SERIAL.available() > 0) Serial.print(char(IMU_SERIAL.read()));
#endif

#ifdef DEBUG_TOF
    // Redirect TOF Serial to monitor
    if (TOF_SERIAL.available() > 0) Serial.print(char(TOF_SERIAL.read()));
#endif

#ifdef DEBUG_TEENSY
    // // Print debug data
    // if (line.exists())
    //     Serial.printf("Line %03d.%02dº %01d.%02d | ", line.bearing / 100,
    //                   line.bearing % 100, line.size / 100, line.size % 100);
    // else
    //     Serial.printf("Line             | ");
    // if (bearing != NO_BEARING)
    //     Serial.printf("Bearing %03d.%02dº | ", bearing / 100, bearing % 100);
    // else
    //     Serial.printf("Bearing          | ");
    // Serial.print("Bounds ");
    // if (bounds.front != NO_BOUNDS)
    //     Serial.printf("F: %4d ", bounds.front);
    // else
    //     Serial.printf("F:      ");
    // if (bounds.back != NO_BOUNDS)
    //     Serial.printf("B: %4d ", bounds.back);
    // else
    //     Serial.printf("B:      ");
    // if (bounds.left != NO_BOUNDS)
    //     Serial.printf("L: %4d ", bounds.left);
    // else
    //     Serial.printf("L:      ");
    // if (bounds.right != NO_BOUNDS)
    //     Serial.printf("R: %4d | ", bounds.right);
    // else
    //     Serial.printf("R:      | ");
    // Serial.printf("BT Inbound: 0x%02X | ", bluetoothInboundPayload.testByte);
    // Serial.println();
    // delay(100);

    // Print controller info
    Serial.printf("Bearing: %3d.%02dº, Angular Velocity: %4d\n", bearing / 100,
                  abs(bearing) % 100, movement.angularVelocity);

    // // Print loop time
    // printLoopTime();

    // // Test motors
    // movement.angle = 0;
    movement.speed = 50;
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

    // Values from https://www.pjrc.com/teensy/td_pulse.html
    // (based on F_CPU_ACTUAL = 600 MHz)
    // analogWriteResolution(12); // TODO: Debug with Multimeter
    // analogWriteFrequency(PIN_MOTOR_FL_PWM, 36621);
    // analogWriteFrequency(PIN_MOTOR_FR_PWM, 36621);
    // analogWriteFrequency(PIN_MOTOR_BL_PWM, 36621);
    // analogWriteFrequency(PIN_MOTOR_BR_PWM, 36621);

    // Initialise serial
    MUX_SERIAL.begin(TEENSY_MUX_BAUD_RATE);
    IMU_SERIAL.begin(TEENSY_IMU_BAUD_RATE);
    TOF_SERIAL.begin(TEENSY_TOF_BAUD_RATE);
    CORAL_SERIAL.begin(TEENSY_CORAL_BAUD_RATE);
    while (!MUX_SERIAL) delay(10);
    while (!IMU_SERIAL) delay(10);
    while (!TOF_SERIAL) delay(10);
    while (!CORAL_SERIAL) delay(10);

#ifndef CALIBRATE // Don't wait for everything to initialise if calibrating
    #ifndef DEBUG_MUX
    // Wait for STM32 MUX to initialise
    Serial.println("Waiting for STM32 MUX to initialise...");
    MUXTXPayload muxTXPayload;
    while (!readPacket(MUX_SERIAL, &muxTXPayload, MUX_TX_PACKET_SIZE,
                       MUX_TX_SYNC_START_BYTE, MUX_TX_SYNC_END_BYTE))
        delay(10);
    #endif

    #ifndef DEBUG_TOF
    // Wait for STM32 TOF to initialise
    Serial.println("Waiting for STM32 TOF to initialise...");
    TOFTXPayload tofTXPayload;
    while (!readPacket(TOF_SERIAL, &tofTXPayload, TOF_TX_PACKET_SIZE,
                       TOF_TX_SYNC_START_BYTE, TOF_TX_SYNC_END_BYTE))
        delay(10);
    #endif

    #ifndef DEBUG_IMU
    // Wait for STM32 IMU to initialise and get the bearing offset
    Serial.println("Waiting for STM32 IMU to initialise...");
    IMUTXPayload imuTXPayload;
    while (!readPacket(IMU_SERIAL, &imuTXPayload, IMU_TX_PACKET_SIZE,
                       IMU_TX_SYNC_START_BYTE, IMU_TX_SYNC_END_BYTE))
        delay(10);
    Serial.println("Waiting for IMU to stabilise...");
    delay(2000); // Wait for IMU to stabilise
    bearingOffset = imuTXPayload.bearing;
    #else
    bearingOffset = 0;
    #endif
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
    // Read all sensor packets from the different subprocessors
    readSensors();

    // Keep straight
    const int16_t robotAngle =
        bearing > 18000 ? (int32_t)bearing - 36000 : bearing;
    movement.angularVelocity = bearingController.advance(robotAngle);

    // Follow the ball
    // TODO

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
    const BluetoothPayload bluetoothOutboundPayload = {
        .testByte = 0x01,
    };
    char buf[sizeof(TOFRXPayload)];
    memcpy(buf, &bluetoothOutboundPayload, sizeof(bluetoothOutboundPayload));
    sendPacket(TOF_SERIAL, buf, TOF_RX_PACKET_SIZE, TOF_RX_SYNC_START_BYTE,
               TOF_RX_SYNC_END_BYTE);
}
// ------------------------------- MAIN CODE END -------------------------------
