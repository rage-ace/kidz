#include <Arduino.h>
#include <PacketSerial.h>

#include "teensy/include/config.h"
#include "teensy/include/main.h"
#include "teensy/include/movement.h"
#include "teensy/include/sensors.h"

PacketSerial muxSerial;
PacketSerial tofSerial;
PacketSerial imuSerial;
PacketSerial coralSerial;

// IO
Sensors sensors = Sensors(muxSerial, tofSerial, imuSerial, coralSerial);
Movement movement = Movement();

void setup() {
    // Turn on the debug LED
    pinMode(PIN_LED_DEBUG, OUTPUT);
    digitalWriteFast(PIN_LED_DEBUG, HIGH);

#ifdef DEBUG
    // Runs any debug code if the corresponding flag is defined
    performSetupDebug();
#endif

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

    // Maintain heading
    if (sensors.robot.angle.newData)
        movement.updateHeadingController(sensors.robot.angle.value);

#ifdef MASTER
    // Performs tasks as the master robot
    runStriker();
#else
    // Performs tasks as the slave robot
    runGoalie();
#endif

#ifdef DEBUG
    // Runs any debug code if the corresponding flag is defined
    performLoopDebug();
#endif

    // Actuate outputs
    movement.update();

    // Mark all sensor data as old
    sensors.markAsRead();
}
