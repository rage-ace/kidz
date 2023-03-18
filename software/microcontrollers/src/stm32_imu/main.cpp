#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

#include "angle.h"
#include "config.h"
#include "stm32_imu/include/config.h"

// State
int16_t robotAngle;

// IMU (Sensor ID, I2C Address, I2C Wire)
Adafruit_BNO055 bno = Adafruit_BNO055(55, I2C_ADDRESS_BNO055, &Wire);

// Reads the current robot angle from the IMU.
int16_t readRobotAngle() {
    sensors_event_t eulerAngles;
    bno.getEvent(&eulerAngles, Adafruit_BNO055::VECTOR_EULER);
    return bearingToAngle(eulerAngles.orientation.x);
}

// DEBUG: Prints all data read from IMU sensors.
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
        TeensySerial.printf(
            "%s: x = %4d.%02d y = %4d.%02d z = %4d.%02d\n", name,
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
    TeensySerial.printf(
        "Calibration: System = %d Gyroscope = %d Accelerometer = %d "
        "Magnetometer = %d\n\n",
        systemCalib, gyroCalib, accCalib, magCalib);
}

// CALIBRATE: Calibrates the IMU and stores the offsets in EEPROM.
void calibrate() {
    TeensySerial.printf("Calibrating...\n");
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
        TeensySerial.printf("%s: x = %11d y = %11d z = %11d\n", name, x, y, z);
    };
    TeensySerial.printf("\nCalibration Complete\n");
    TeensySerial.printf("\nOffsets\n");
    printVector("Accelerometer: ", offsets.accel_offset_x,
                offsets.accel_offset_y, offsets.accel_offset_z);
    printVector("Gyroscope    : ", offsets.gyro_offset_x, offsets.gyro_offset_y,
                offsets.gyro_offset_z);
    printVector("Magnetometer : ", offsets.mag_offset_x, offsets.mag_offset_y,
                offsets.mag_offset_z);
    TeensySerial.printf("Accelerometer Radius: %d\n", offsets.accel_radius);
    TeensySerial.printf("Magnetometer Radius : %d\n", offsets.mag_radius);

    TeensySerial.printf("\n\nStoring calibration data to EEPROM...\n");
    EEPROM.put(EEPROM_ADDRESS_HAS_OFFSETS, true);
    EEPROM.put(EEPROM_ADDRESS_OFFSETS, offsets);
    TeensySerial.printf("Offsets Saved\n");
}

// ------------------------------ MAIN CODE START ------------------------------
void setup() {
    // Turn on the debug LED
    pinMode(PIN_LED_DEBUG, OUTPUT);
    digitalWrite(PIN_LED_DEBUG, HIGH);

    // Initialise pins
    Wire.setSDA(PIN_SDA_IMU);
    Wire.setSCL(PIN_SCL_IMU);

    // Initialise serial
    TeensySerial.setup(true);
#ifdef DEBUG
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL) delay(10);
#endif

    // Initialise I2C
    Wire.begin();
    Wire.setClock(400000); // Use 400 kHz I2C

    // Initialise IMU
    if (!bno.begin()) {
#ifdef DEBUG
        DEBUG_SERIAL.println("IMU not found");
#endif
        // Blink the debug LED if IMU not found
        while (1) {
            digitalWrite(PIN_LED_DEBUG, HIGH);
            delay(100);
            digitalWrite(PIN_LED_DEBUG, LOW);
            delay(100);
        }
    }
    bno.setExtCrystalUse(false); // we do not have an external crystal

    // Check if the STM32 is in calibration mode
    delay(2000);
    IMURXPayload payload;
    // TeensySerial.readPacket(&payload) // TODO: WHY DOES THIS BUILD ERROR
    TeensySerial.readPacket(nullptr);
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
    robotAngle = readRobotAngle();

    // Send the IMU data over serial to Teensy
    uint8_t buf[sizeof(IMUTXPayload)];
    memcpy(buf, &robotAngle, sizeof(robotAngle));
    TeensySerial.sendPacket(buf);

    // ------------------------------ START DEBUG ------------------------------

    // // Scan for I2C devices
    // scanI2C(TEENSY_SERIAL, Wire);
    // delay(5000);

    // // Print all IMU data
    // printAllIMUData();
    // delay(1000);

    // // Print payload
    // TEENSY_SERIAL.printf("bearing: %3d.%02dº\n", bearing / 100, bearing %
    // 100);

    // // Print loop time
    // printLoopTime(2000);

    // ------------------------------- END DEBUG -------------------------------
}
// ------------------------------- MAIN CODE END -------------------------------
