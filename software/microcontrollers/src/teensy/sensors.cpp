#include "sensors.h"

#include "config.h"
#include "teensy/include/config.h"

void Sensors::init() {
    analogReadResolution(12);

    // Initialise serial
    _muxSerial.setup(true);
    _imuSerial.setup(true);
    _tofSerial.setup(true);
    _coralSerial.setup(true);

    // Initialise subprocessors
#ifndef DONT_WAIT_FOR_SUBPROCESSOR_INT
    #ifndef DEBUG_MUX
    // Wait for STM32 MUX to initialise
    Serial.println("Waiting for STM32 MUX to initialise...");
    _muxSerial.waitForPacket();
    #endif

    #ifndef DEBUG_TOF
    // Wait for STM32 TOF to initialise
    Serial.println("Waiting for STM32 TOF to initialise...");
    _tofSerial.waitForPacket();
    #endif

    #ifndef DEBUG_IMU
    // Wait for STM32 IMU to initialise and get the angle offset
    Serial.println("Waiting for STM32 IMU to initialise...");
    IMUTXPayload imuTXPayload;
    _imuSerial.waitForPacket(&imuTXPayload);
    // We can assume the value has already stabilised as the Teensy can be
    // reset separately from the IMU, after the IMU has been on for a while
    _robotAngleOffset = imuTXPayload.imu.robotAngle;
    #else
    _robotAngleOffset = 0;
    #endif

    // NOTE: We do not wait for the coral here.
#endif
}

void Sensors::read() {
#ifndef DEBUG_MUX
    // Read line data from STM32 MUX
    MUXTXPayload muxTXPayload;
    if (_muxSerial.readPacket(&muxTXPayload)) {
        _line.newData = muxTXPayload.line.newData;
        _line.angle = (float)muxTXPayload.line.angle / 100;
        // TODO: convert to line size to line depth
        _line.depth = (float)muxTXPayload.line.size / 100;
    }
#endif

#ifndef DEBUG_IMU
    // Read attitude data from STM32 IMU
    IMUTXPayload imuTXPayload;
    if (_imuSerial.readPacket(&imuTXPayload)) {
        _robot.newData = imuTXPayload.imu.newData;
        const auto adjustedAngle =
            clipAngle((int32_t)imuTXPayload.imu.robotAngle - _robotAngleOffset);
        _robot.angle = (float)adjustedAngle / 100;
    }
#endif

#ifndef DEBUG_TOF
    // Read bounds and bluetooth data from STM32 TOF
    TOFTXPayload tofTXPayload;
    if (_tofSerial.readPacket(&tofTXPayload)) {
        _bounds.newData = tofTXPayload.bounds.newData;
        _bounds.front = (float)tofTXPayload.bounds.front / 10;
        _bounds.back = (float)tofTXPayload.bounds.back / 10;
        _bounds.left = (float)tofTXPayload.bounds.left / 10;
        _bounds.right = (float)tofTXPayload.bounds.right / 10;

        _otherRobot.newData = tofTXPayload.bluetoothInboundPayload.newData;
        _otherRobot.testByte = tofTXPayload.bluetoothInboundPayload.testByte;
    }
#endif

    // Read ball data from coral
    CoralTXPayload coralTXPayload;
    if (_coralSerial.readPacket(&coralTXPayload)) {
        _ball.newData = coralTXPayload.camera.newData;
        _ball.angle = (float)coralTXPayload.camera.ballAngle / 100;
        _ball.distance = (float)coralTXPayload.camera.ballDistance / 10;
    }

    // Read lightgate
    _hasBall = analogRead(PIN_LIGHTGATE) < LIGHTGATE_THRESHOLD;
}

void Sensors::markAsRead() {
    _line.newData = false;
    _robot.newData = false;
    _bounds.newData = false;
    _otherRobot.newData = false;
    _ball.newData = false;
}
