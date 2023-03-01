#include <Arduino.h>

#include "config.h"
#include "teensy/include/config.h"
#include "util.h"

// State
struct LineData line; // Read from stm32 mux
struct MovementData {
    float angle = 0;             // 0º to 360º
    int16_t speed = 30;          // 30 to 4096
    int16_t angularVelocity = 0; // TODO: Figure out the range of this
} movement;

void setup() {
    // Initialise Pins
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
    analogWriteFrequency(PIN_MOTOR_FL_PWM, 36621);
    analogWriteFrequency(PIN_MOTOR_FR_PWM, 36621);
    analogWriteFrequency(PIN_MOTOR_BL_PWM, 36621);
    analogWriteFrequency(PIN_MOTOR_BR_PWM, 36621);

    // Initialise Serial
    Serial.begin(MONITOR_BAUD_RATE);
    Serial.println("Teensy Started Up");

    MUX_SERIAL.begin(TEENSY_MUX_BAUD_RATE);
}

// Writes the current movement data to the motors
void drive() {
    // Convert polar to cartesian
    const auto x = sinf(movement.angle * DEG_TO_RAD);
    const auto y = cosf(movement.angle * DEG_TO_RAD);

    // Compute the speeds of the individual motors
    const auto transformSpeed = [](float speed) {
        // Add angular velocity coefficient
        speed += movement.angularVelocity * ANGULAR_VELOCITY_MULTIPLIER;
        // Scale to speed
        return (int16_t)roundf(speed * movement.speed);
    };
    const int16_t FLSpeed = transformSpeed(x * COS45 + y * SIN45);
    const int16_t FRSpeed = transformSpeed(x * -COS45 + y * SIN45);
    const int16_t BLSpeed = transformSpeed(x * -COS45 + y * SIN45);
    const int16_t BRSpeed = transformSpeed(x * COS45 + y * SIN45);

    // Set the motor directions and speeds
    digitalWrite(PIN_MOTOR_FL_DIR, FLSpeed > 0 ? LOW : HIGH);
    digitalWrite(PIN_MOTOR_FR_DIR, FRSpeed > 0 ? LOW : HIGH);
    digitalWrite(PIN_MOTOR_BL_DIR, BLSpeed > 0 ? LOW : HIGH);
    digitalWrite(PIN_MOTOR_BR_DIR, BRSpeed > 0 ? LOW : HIGH);
    analogWrite(PIN_MOTOR_FL_PWM, max(abs(FLSpeed), DRIVE_STALL_SPEED));
    analogWrite(PIN_MOTOR_FR_PWM, max(abs(FRSpeed), DRIVE_STALL_SPEED));
    analogWrite(PIN_MOTOR_BL_PWM, max(abs(BLSpeed), DRIVE_STALL_SPEED));
    analogWrite(PIN_MOTOR_BR_PWM, max(abs(BRSpeed), DRIVE_STALL_SPEED));
}

void loop() {
    // Read line data from stm32 mux
    while (MUX_SERIAL.available() >= (signed int)(MUX_TX_PACKET_SIZE)) {
        const auto syncByte = MUX_SERIAL.read();
        if (syncByte == MUX_TX_SYNC_START_BYTE) {
            // start of packet is valid, proceed to read rest of packet
            char buf[MUX_TX_PACKET_SIZE - 1];
            MUX_SERIAL.readBytes(buf, MUX_TX_PACKET_SIZE - 1);
            if (buf[MUX_TX_PACKET_SIZE - 2] == MUX_TX_SYNC_END_BYTE) {
                // end of packet is valid, proceed to use payload
                memcpy(&line, &buf, sizeof(line));
            }
        }
    }

    // Ball following
    // TODO

    // Line avoidance
    // TODO

    // START DEBUG
    // // Print debug data
    // if (line.isPresent) {
    //     Serial.printf("Line %03d.%02dº %01d.%02d\n | ", line.bearing / 100,
    //                   line.bearing % 100, line.size / 100, line.size % 100);
    // } else {
    //     Serial.printf("Line            \n |");
    // }

    // // Redirect MUX Serial to monitor
    // if (MUX_SERIAL.available() > 0) Serial.print(char(MUX_SERIAL.read()));

    // Test motors
    movement.angle = 0;
    movement.speed = 10;
    movement.angularVelocity = 10;
    // END DEBUG

    // Actuate outputs
    drive();
}
