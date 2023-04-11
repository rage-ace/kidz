#include <Arduino.h>
#include <PacketSerial.h>
#include <array>

#include "angle.h"
#include "shared_config.h"
#include "stm32_mux/include/config.h"

// State
struct LineData line;
std::array<uint16_t, LDR_COUNT> activatedCount;

// Serial
PacketSerial teensySerial;

// Sets an LDR MUX to select a specific channel.
void selectLDRMUXChannel(LDRMUX mux, uint8_t channel) {
    switch (mux) {
    case MUX1:
        digitalWrite(PIN_LDRMUX1_S0, channel & 1);
        digitalWrite(PIN_LDRMUX1_S1, (channel >> 1) & 1);
        digitalWrite(PIN_LDRMUX1_S2, (channel >> 2) & 1);
        digitalWrite(PIN_LDRMUX1_S3, (channel >> 3) & 1);
    case MUX2:
        digitalWrite(PIN_LDRMUX2_S0, channel & 1);
        digitalWrite(PIN_LDRMUX2_S1, (channel >> 1) & 1);
        digitalWrite(PIN_LDRMUX2_S2, (channel >> 2) & 1);
        digitalWrite(PIN_LDRMUX2_S3, (channel >> 3) & 1);
    }
}

// Reads the value of an LDR 0-indexed counting clockwise from 000º.
uint16_t readLDR(uint8_t index) {
    const auto real_index = LDR_MAP_REVERSE[index];
    selectLDRMUXChannel(real_index < 16 ? MUX1 : MUX2, real_index % 16);
    return analogRead(real_index < 16 ? PIN_LDRMUX1_SIG : PIN_LDRMUX2_SIG);
}

// Finds the position of the line.
void findLine() {
    // Read LDRs, the LDRs need to be activated for a while to reduce noise
    for (uint8_t i = 0; i < LDR_COUNT; ++i) {
        if (readLDR(i) > LDR_THRESHOLDS[i]) {
            if (activatedCount[i] != UINT16_MAX) ++activatedCount[i];
        } else
            activatedCount[i] = 0;
    }

    // Get the matches (to the line)
    uint8_t matchCount = 0;
    uint8_t matches[LDR_COUNT];
    for (uint8_t i = 0; i < LDR_COUNT; ++i) {
        if (activatedCount[i] >= LDR_ACTIVATION_THRESHOLD) {
            matches[matchCount] = i;
            ++matchCount;
        }
    }

    // No match (to the line) was found
    if (matchCount == 0) {
        line.angleBisector = NO_LINE_INT16;
        line.size = NO_LINE_UINT8;
        return;
    }

    // Find the cluster
    float maxAngleDifference = 0;
    uint8_t clusterStart = LDR_COUNT, clusterEnd = LDR_COUNT;
    // Iterate through all combinations of matching indices and find
    // the pair with the greatest minimum difference
    for (uint8_t i = 0; i < matchCount - 1; ++i) {
        for (uint8_t j = i + 1; j < matchCount; ++j) {
            auto angleDifference =
                fabsf(LDR_BEARINGS[matches[i]] - LDR_BEARINGS[matches[j]]);
            angleDifference =
                angleDifference > 180 ? 360 - angleDifference : angleDifference;
            if (angleDifference > maxAngleDifference) {
                maxAngleDifference = angleDifference;
                clusterStart = matches[i];
                clusterEnd = matches[j];
            }
        }
    }

    // No cluster was found
    if (clusterStart == LDR_COUNT) {
        line.angleBisector = NO_LINE_INT16;
        line.size = NO_LINE_UINT8;
        return;
    }

    // Calculate the line angle and size
    const auto clusterStartAngle = LDR_BEARINGS[clusterStart];
    const auto clusterEndAngle = LDR_BEARINGS[clusterEnd];
    const auto clusterMidpoint =
        bearingMidpoint(clusterStartAngle, clusterEndAngle);

    // Sets lineAngle as perpendicular to angle of the midpoint of cluster ends
    line.angleBisector = roundf(bearingToAngle(clusterMidpoint) * 100);
    // Sets lineSize as the ratio of the cluster size to 180°
    line.size = roundf(
        (smallerBearingDiff(clusterStartAngle, clusterEndAngle) / 180.0F) *
        100);
}

// CALIBRATE: Determines threshold values for the photodiodes.
void printLDRThresholds() {
    // min = green (field), max = white (line)
    uint16_t min[LDR_COUNT], max[LDR_COUNT];
    for (int i = 0; i < LDR_COUNT; ++i) {
        min[i] = 0xFFFF;
        max[i] = 0x0000;
    }

    const auto endTime = millis() + LDR_CALIBRATION_DURATION;
    while (millis() < endTime) {
        for (uint8_t i = 0; i < LDR_COUNT; ++i) {
            const auto value = readLDR(i);
            if (value < min[i]) min[i] = value;
            if (value > max[i]) max[i] = value;
        }
    }

    // Print the thresholds (averages of min and max)
    TEENSY_SERIAL.printf("Thresholds: {");
    for (uint8_t i = 0; i < LDR_COUNT; ++i) {
        const auto threshold =
            (max[i] - min[i]) * LDR_CALIBRATION_MULTIPLIER + min[i];
        TEENSY_SERIAL.printf("%d, ", (uint16_t)threshold);
    }
    TEENSY_SERIAL.printf("}\n");
}

// DEBUG: Prints detected line data.
void printLDR() {
    findLine();

    uint16_t values[LDR_COUNT];
    for (uint8_t i = 0; i < LDR_COUNT; ++i) values[i] = readLDR(i);

    if (line.exists()) {
        TEENSY_SERIAL.printf("%4d.%02dº %01d.%02d |", line.angleBisector / 100,
                             abs(line.angleBisector % 100), line.size / 100,
                             line.size % 100);
    } else {
        TEENSY_SERIAL.printf("              |");
    }
    for (uint8_t i = 0; i < LDR_COUNT / 2; ++i)
        TEENSY_SERIAL.printf("%s", values[i] > LDR_THRESHOLDS[i] ? "1" : " ");
    TEENSY_SERIAL.printf("|");
    for (uint8_t i = LDR_COUNT / 2; i < LDR_COUNT; ++i)
        TEENSY_SERIAL.printf("%s", values[i] > LDR_THRESHOLDS[i] ? "1" : " ");
    TEENSY_SERIAL.printf("| ");
    for (uint8_t i = 0; i < LDR_COUNT / 2; ++i)
        TEENSY_SERIAL.printf("%4d ", values[i]);
    TEENSY_SERIAL.printf("| ");
    for (uint8_t i = LDR_COUNT / 2; i < LDR_COUNT; ++i)
        TEENSY_SERIAL.printf("%4d ", values[i]);
    TEENSY_SERIAL.printf("\n");
}

// ------------------------------ MAIN CODE START ------------------------------
void onTeensyPacket(const byte *buf, size_t size) {
    MUXRXPayload payload;
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));

    // If the STM32 is in calibration mode, print the thresholds
    if (payload.calibrating) { // defaults to false
        TEENSY_SERIAL.println("Calibrating...");
        while (1) printLDRThresholds();
    }
}

void setup() {
    // Turn on the debug LED
    pinMode(PIN_LED_DEBUG, OUTPUT);
    digitalWrite(PIN_LED_DEBUG, HIGH);

    // Initialise pins
    pinMode(PIN_LDRMUX1_S0, OUTPUT);
    pinMode(PIN_LDRMUX1_S1, OUTPUT);
    pinMode(PIN_LDRMUX1_S2, OUTPUT);
    pinMode(PIN_LDRMUX1_S3, OUTPUT);
    pinMode(PIN_LDRMUX2_S0, OUTPUT);
    pinMode(PIN_LDRMUX2_S1, OUTPUT);
    pinMode(PIN_LDRMUX2_S2, OUTPUT);
    pinMode(PIN_LDRMUX2_S3, OUTPUT);
    pinMode(PIN_LDRMUX1_SIG, INPUT);
    pinMode(PIN_LDRMUX2_SIG, INPUT);

    analogReadResolution(12);

    // Initialise serial
    TEENSY_SERIAL.begin(TEENSY_MUX_BAUD_RATE);
#ifdef DEBUG
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL) delay(10);
#endif
    teensySerial.setStream(&TEENSY_SERIAL);
    teensySerial.setPacketHandler(&onTeensyPacket);

    // Line data is always new
    line.newData = true;

    // Turn off the debug LED
    digitalWrite(PIN_LED_DEBUG, LOW);
}

void loop() {
    // Read packets from serial
    teensySerial.update();

    // Find the line
    findLine();

    // Send the line data over serial to Teensy
    byte buf[sizeof(MUXTXPayload)];
    memcpy(buf, &line, sizeof(line));
    teensySerial.send(buf, sizeof(buf));

    // ------------------------------ START DEBUG ------------------------------
    // // Print LDR data
    // printLDR();

    // // Print loop time
    // printLoopTime();
    // ------------------------------- END DEBUG -------------------------------
}
// ------------------------------- MAIN CODE END -------------------------------
