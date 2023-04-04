#include "sensors.h"

#include "shared_config.h"
#include "teensy/include/config.h"

void Sensors::init() {
    analogReadResolution(12);

    // Initialise serial
    MUX_SERIAL.begin(TEENSY_MUX_BAUD_RATE);
    TOF_SERIAL.begin(TEENSY_TOF_BAUD_RATE);
    IMU_SERIAL.begin(TEENSY_IMU_BAUD_RATE);
    CORAL_SERIAL.begin(TEENSY_CORAL_BAUD_RATE);

#ifndef DEBUG_MUX
    _muxSerial.setStream(&MUX_SERIAL);
#endif
#ifndef DEBUG_TOF
    _tofSerial.setStream(&TOF_SERIAL);
#endif
#ifndef DEBUG_IMU
    _imuSerial.setStream(&IMU_SERIAL);
#endif
#ifndef DEBUG_CORAL
    _coralSerial.setStream(&CORAL_SERIAL);
#endif
}

void Sensors::waitForSubprocessorInit() {
    // Initialise subprocessors
#ifndef DEBUG_MUX
    // Wait for STM32 MUX to initialise
    Serial.println("Waiting for STM32 MUX to initialise...");
    while (!_muxInit) _muxSerial.update();
#endif

#ifndef DEBUG_TOF
    // Wait for STM32 TOF to initialise
    Serial.println("Waiting for STM32 TOF to initialise...");
    while (!_tofInit) _tofSerial.update();
#endif

#ifndef DEBUG_IMU
    // Wait for STM32 IMU to initialise and get the angle offset
    Serial.println("Waiting for STM32 IMU to initialise...");
    while (!_imuInit) _imuSerial.update();
#else
    _robotAngleOffset = 0;
#endif

    // NOTE: We do not wait for the coral here.
}

void Sensors::onMuxPacket(const byte *buf, size_t size) {
    // Load payload
    MUXTXPayload payload;
    memcpy(&payload, buf, sizeof(payload));

    // Update new flag
    _line.newData = payload.line.newData;

    // Update line angle
    if (payload.line.angleBisector != NO_LINE_INT16)
        _line.angleBisector = (float)payload.line.angleBisector / 100;
    else
        _line.angleBisector = NAN;

    // Compute line depth
    if (payload.line.size != NO_LINE_UINT8) {
        const auto lineSize = (float)payload.line.size / 100;

        if (smallerBearingDiff(clipBearing(_line.angleBisector),
                               clipBearing(_lastLineAngleBisector)) > 90) {
            // There's a huge jump in the line angle bisector
            if (_line.depth < 0.5) {
                // Robot moved from the inner half to the outer half of the line
                _line.depth = 1 - lineSize / 2;
                _line.angleBisector =
                    bearingToAngle(clipBearing(_line.angleBisector + 180));
            } else {
                // Robot moved from the outer half to the inner half of the line
                _line.depth = lineSize / 2;
            }
        } else if (_line.depth < 0.5) {
            // The robot is on the inner half of the line
            _line.depth = lineSize / 2;
        } else {
            // The robot is on the outer half of the line
            _line.depth = 1 - lineSize / 2;
            _line.angleBisector =
                bearingToAngle(clipBearing(_line.angleBisector + 180));
        }
    } else if (_wasOnLine) {
        // The robot just exited the line, snap the depth to 0 or 1
        if (_line.depth < 0.5)
            _line.depth = 0; // It exited on the inside of the line
        else
            _line.depth = 1; // It exited on the outside of the line
    }

    // Update history for next iteration
    _wasOnLine = payload.line.size != NO_LINE_UINT8;
    _lastLineAngleBisector = _line.angleBisector;

    // Consider the STM32 MUX to be initialised
    _muxInit = true;
}

void Sensors::onTofPacket(const byte *buf, size_t size) {
    // Load payload
    TOFTXPayload payload;
    memcpy(&payload, buf, sizeof(payload));

    // Update bounds data
    _bounds.front.newData = payload.bounds.front.newData;
    _bounds.back.newData = payload.bounds.back.newData;
    _bounds.left.newData = payload.bounds.left.newData;
    _bounds.right.newData = payload.bounds.right.newData;
    if (payload.bounds.front.value == NO_BOUNDS)
        _bounds.front.value = NAN;
    else
        _bounds.front.value = (float)payload.bounds.front.value / 10;
    if (payload.bounds.back.value == NO_BOUNDS)
        _bounds.back.value = NAN;
    else
        _bounds.back.value = (float)payload.bounds.back.value / 10;
    if (payload.bounds.left.value == NO_BOUNDS)
        _bounds.left.value = NAN;
    else
        _bounds.left.value = (float)payload.bounds.left.value / 10;
    if (payload.bounds.right.value == NO_BOUNDS)
        _bounds.right.value = NAN;
    else
        _bounds.right.value = (float)payload.bounds.right.value / 10;

    // Update bluetooth data
    _otherRobot.newData = payload.bluetoothInboundPayload.newData;
    _otherRobot.testByte = payload.bluetoothInboundPayload.testByte;

    // Consider the STM32 TOF to be initialised
    _tofInit = true;
}

void Sensors::onImuPacket(const byte *buf, size_t size) {
    // Load payload
    IMUTXPayload payload;
    memcpy(&payload, buf, sizeof(payload));

    // If this is the first reading, record down the initial angle offset
    if (!_imuInit)
        // We can assume the value has already stabilised as the Teensy can
        // be reset separately from the IMU, after the IMU has been on for a
        // while
        _robotAngleOffset = payload.imu.robotAngle;

    // Update new flag
    _robot.newData = payload.imu.newData;

    // Update robot angle
    const auto adjustedAngle =
        clipAngle((int32_t)payload.imu.robotAngle - _robotAngleOffset);
    _robot.angle = (float)adjustedAngle / 100;

    // Consider the STM32 IMU to be initialised
    _imuInit = true;
}

void Sensors::onCoralPacket(const byte *buf, size_t size) {
    // Load payload
    CoralTXPayload payload;
    memcpy(&payload, buf, sizeof(payload));

    // Update ball data
    _ball.newData = payload.camera.newData;
    _ball.angle = (float)payload.camera.ballAngle / 100;
    _ball.distance = (float)payload.camera.ballDistance / 10;

    // Consider the Coral to be initialised
    _coralInit = true;
}

void Sensors::read() {
    // Read packets from serial
    _muxSerial.update();
    _tofSerial.update();
    _imuSerial.update();
    _coralSerial.update();

    // Read lightgate
    _hasBall = analogRead(PIN_LIGHTGATE) < LIGHTGATE_THRESHOLD;
}

void Sensors::markAsRead() {
    _line.newData = false;
    _robot.newData = false;
    _bounds.front.newData = false;
    _bounds.back.newData = false;
    _bounds.left.newData = false;
    _bounds.right.newData = false;
    _otherRobot.newData = false;
    _ball.newData = false;
}
