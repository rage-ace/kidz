#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>

#include "config.h"
#include "stm32_tof/include/config.h"
#include "util.h"

// State
Boundary boundary;

// Time-Of-Flight Sensors
VL53L1X tofs[TOF_COUNT];

// ------------------------------ MAIN CODE START ------------------------------
void setup() {
    // Turn on the debug LED
    pinMode(PIN_LED_DEBUG, OUTPUT);
    digitalWrite(PIN_LED_DEBUG, HIGH);

    // Initialise pins
    pinMode(PIN_RING, OUTPUT);
    pinMode(PIN_LEFT_EYE, OUTPUT);
    pinMode(PIN_RIGHT_EYE, OUTPUT);

    Wire.setSDA(PIN_SDA_TOF);
    Wire.setSCL(PIN_SCL_TOF);

    // Initialise serial
    TEENSY_SERIAL.begin(TEENSY_TOF_BAUD_RATE);
#if DEBUG
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
#endif
    while (!TEENSY_SERIAL) delay(10);
#if DEBUG
    while (!DEBUG_SERIAL) delay(10);
#endif

    // Initialise I2C
    Wire.begin();
    Wire.setClock(400000); // Use 400 kHz I2C

    // Disable all TOFs
    for (uint8_t i = 0; i < TOF_COUNT; ++i) {
        // Drive the XSHUT low to disable the sensor
        pinMode(TOF_CONFIGS[i].xshutPin, OUTPUT);
        digitalWrite(TOF_CONFIGS[i].xshutPin, LOW);
    }
    // Enable and initialise each sensor, one by one
    for (uint8_t i = 0; i < TOF_COUNT; ++i) {
        // Allow XSHUT to be pulled HIGH (don't drive it, not level shifted)
        pinMode(TOF_CONFIGS[i].xshutPin, INPUT);
        delay(10);

        tofs[i].setTimeout(500);
        if (!tofs[i].init()) {
            Serial.printf("Failed to detect and initialize sensor %d", i);
            while (1) {}
        }

        tofs[i].setAddress(I2C_ADDRESS_TOF_START + i);
        tofs[i].setDistanceMode(TOF_CONFIGS[i].distanceMode);
        tofs[i].setMeasurementTimingBudget(TOF_CONFIGS[i].measurementPeriod);
        tofs[i].startContinuous(TOF_CONFIGS[i].intermeasurementPeriod);
        tofs[i].setROICenter(199); // 199 is the optical center
        tofs[i].setROISize(4, 4);  // FOV ranges from 4 to 16
    }

    // Turn off the debug LED
    digitalWrite(PIN_LED_DEBUG, LOW);
}

void loop() {
    // Read TOF ranges
    for (uint8_t i = 0; i < TOF_COUNT; i++) {
        tofs[i].read();
        if (tofs[i].ranging_data.range_status == VL53L1X::RangeValid)
            boundary.set(i, tofs[i].ranging_data.range_mm);
    }

    // Send the TOF data over serial to Teensy
    uint8_t buf[sizeof(TOFTXPayload)];
    memcpy(buf, &boundary, sizeof(boundary));
    sendPacket(TEENSY_SERIAL, buf, TOF_TX_PACKET_SIZE, TOF_TX_SYNC_START_BYTE,
               TOF_TX_SYNC_END_BYTE);

    // ------------------------------ START DEBUG ------------------------------

    // // Scan for I2C devices
    // pinMode(PIN_XSHUT_FRONT, INPUT);
    // pinMode(PIN_XSHUT_BACK, INPUT);
    // pinMode(PIN_XSHUT_LEFT, INPUT);
    // pinMode(PIN_XSHUT_RIGHT, INPUT);
    // scanI2C(TEENSY_SERIAL, Wire);
    // delay(5000);

    // // Print payload
    // TEENSY_SERIAL.printf("F: %4d B: %4d L: %4d R: %4d\n", boundary.front,
    //                      boundary.back, boundary.left, boundary.right);

    // ------------------------------- END DEBUG -------------------------------
}
// ------------------------------- MAIN CODE END -------------------------------
