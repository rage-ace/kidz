#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

#include "config.h"
#include "stm32_imu/include/config.h"
#include "util.h"

// State
uint16_t bearing;

// IMU (Sensor ID, I2C Address, I2C Wire)
Adafruit_BNO055 bno = Adafruit_BNO055(55, I2C_ADDRESS_BNO055, &Wire);

// Reads the current bearing from the IMU.
uint16_t readBearing() {
    sensors_event_t eulerAngles;
    bno.getEvent(&eulerAngles, Adafruit_BNO055::VECTOR_EULER);
    return roundf(eulerAngles.orientation.x * 100);
}

// Prints all data read from IMU sensors.
void printAllIMUData() {
    // Get sensor data
    sensors_event_t eul, gyr, lac, mag, acc, gra;
    bno.getEvent(&eul, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&gyr, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&acc, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&lac, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&gra, Adafruit_BNO055::VECTOR_GRAVITY);
    bno.getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Get calibration states
    uint8_t systemCalib, gyroCalib, accCalib, magCalib;
    bno.getCalibration(&systemCalib, &gyroCalib, &accCalib, &magCalib);

    // Print everything to serial
    const auto printVector = [](const char *name, const sensors_vec_t &vector) {
        Serial.printf("%s: x = %4d.%02d y = %4d.%02d z = %4d.%02d\n", name,
                      (int16_t)vector.x, abs((int32_t)(vector.x * 100) % 100),
                      (int16_t)vector.y, abs((int32_t)(vector.y * 100) % 100),
                      (int16_t)vector.z, abs((int32_t)(vector.z * 100) % 100));
    };
    printVector("Euler Angle (º)            ", eul.orientation);
    printVector("Angular Velocity (rad s⁻¹) ", gyr.gyro);
    printVector("Acceleration (m s⁻²)       ", acc.acceleration);
    printVector("Linear Acceleration (m s⁻²)", lac.acceleration);
    printVector("Gravity (m s⁻²)            ", gra.acceleration);
    printVector("Magnetic Field (μT)        ", mag.magnetic);
    Serial.printf("Calibration: System = %d Gyroscope = %d Accelerometer = %d "
                  "Magnetometer = %d\n\n",
                  systemCalib, gyroCalib, accCalib, magCalib);
}

// CALIBRATE: Calibrates the IMU and stores the offsets in EEPROM.
void calibrate() {
    Serial.println("Calibrating...");
    delay(1000);

    // Calibration phase
    while (!bno.isFullyCalibrated()) {
        printAllIMUData();
        delay(1000);
    }
    printAllIMUData();

    // Calibration completed
    adafruit_bno055_offsets_t offsets;
    bno.getSensorOffsets(offsets);

    // Print results
    const auto printVector = [](const char *name, const float x, const float y,
                                const float z) {
        Serial.printf("%s: x = %11d y = %11d z = %11d\n", name, x, y, z);
    };
    Serial.println("\nCalibration Complete");
    Serial.println("\nOffsets");
    printVector("Accelerometer: ", offsets.accel_offset_x,
                offsets.accel_offset_y, offsets.accel_offset_z);
    printVector("Gyroscope    : ", offsets.gyro_offset_x, offsets.gyro_offset_y,
                offsets.gyro_offset_z);
    printVector("Magnetometer : ", offsets.mag_offset_x, offsets.mag_offset_y,
                offsets.mag_offset_z);
    Serial.printf("Accelerometer Radius: %d\n", offsets.accel_radius);
    Serial.printf("Magnetometer Radius : %d\n", offsets.mag_radius);

    Serial.println("\n\nStoring calibration data to EEPROM...");
    EEPROM.put(EEPROM_ADDRESS_HAS_OFFSETS, true);
    EEPROM.put(EEPROM_ADDRESS_OFFSETS, offsets);
    Serial.println("Offsets Saved");
}

// ------------------------------ MAIN CODE START ------------------------------
void setup() {
    // Turn on the debug LED
    pinMode(PIN_LED_DEBUG, OUTPUT);
    digitalWrite(PIN_LED_DEBUG, HIGH);

    // Initialise Pins
    Wire.setSDA(PIN_SDA_IMU);
    Wire.setSCL(PIN_SCL_IMU);

    // Initialise Serial
    Serial.begin(TEENSY_IMU_BAUD_RATE);
#if DEBUG
    Serial1.begin(DEBUG_BAUD_RATE);
#endif
    while (!Serial) delay(10);
#if DEBUG
    while (!Serial1) delay(10);
#endif

    // Initialise I2C
    Wire.begin();

    // Initialise IMU
    if (!bno.begin()) {
#if DEBUG
        Serial1.println("IMU not found");
#endif
        // Blink the debug LED if IMU not found
        while (1) {
            digitalWrite(PIN_LED_DEBUG, HIGH);
            delay(100);
            digitalWrite(PIN_LED_DEBUG, LOW);
            delay(100);
        }
    }

    // Check if the STM32 is in calibration mode
    delay(2000);
    IMURXPayload payload;
    readPacket(Serial, &payload, IMU_RX_PACKET_SIZE, IMU_RX_SYNC_START_BYTE,
               IMU_RX_SYNC_END_BYTE);
    if (payload.calibrating) { // defaults to false
        calibrate();
        while (1) {};
    }

    // Attempt to load IMU offsets
    bool hasOffsets;
    EEPROM.get(EEPROM_ADDRESS_HAS_OFFSETS, hasOffsets);
    if (hasOffsets) {
        adafruit_bno055_offsets_t offsets;
        EEPROM.get(EEPROM_ADDRESS_OFFSETS, offsets);
        bno.setSensorOffsets(offsets);

        // Turn off the debug LED
        digitalWrite(PIN_LED_DEBUG, LOW);
    }
}

void loop() {
    // Read IMU data
    bearing = readBearing();

    // Send the IMU data over serial to Teensy
    uint8_t buf[sizeof(bearing)];
    memcpy(buf, &bearing, sizeof(bearing));
    sendPacket(Serial, buf, IMU_TX_PACKET_SIZE, IMU_TX_SYNC_START_BYTE,
               IMU_TX_SYNC_END_BYTE);

    // ---------------------------- START DEBUG ----------------------------

    // // Scan for I2C devices
    // scanI2C(Serial, Wire);
    // delay(5000);

    // // Print all IMU data
    // printAllIMUData();
    // delay(1000);

    // // Print payload
    // Serial.printf("bearing: %3d.%02dº\n", attitude.bearing / 100,
    //               attitude.bearing % 100);

    // ----------------------------- END DEBUG -----------------------------
}
// ------------------------------- MAIN CODE END -------------------------------
