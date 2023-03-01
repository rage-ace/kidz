#include <Arduino.h>

#include "config.h"
#include "teensy/include/config.h"

struct LineData line;

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

    // Initialise Serial
    Serial.begin(MONITOR_BAUD_RATE);
    Serial.println("Teensy Started Up");

    MUX_SERIAL.begin(TEENSY_MUX_BAUD_RATE);
}

void loop() {
    // Read line data from mux
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

    // DEBUG
    // Print debug data
    if (line.isPresent) {
        Serial.printf("Line %03d.%02dº %01d.%02d\n | ", line.bearing / 100,
                      line.bearing % 100, line.size / 100, line.size % 100);
    } else {
        Serial.printf("Line            \n |");
    }

    // Redirect MUX Serial to monitor
    // if (MUX_SERIAL.available() > 0) Serial.print(char(MUX_SERIAL.read()));

    // Test motors
    digitalWriteFast(PIN_MOTOR_FL_DIR, HIGH);
    digitalWriteFast(PIN_MOTOR_FR_DIR, HIGH);
    digitalWriteFast(PIN_MOTOR_BL_DIR, HIGH);
    digitalWriteFast(PIN_MOTOR_BR_DIR, HIGH);

    analogWrite(PIN_MOTOR_FL_PWM, 100);
    analogWrite(PIN_MOTOR_FR_PWM, 100);
    analogWrite(PIN_MOTOR_BL_PWM, 100);
    analogWrite(PIN_MOTOR_BR_PWM, 100);
}
