#include "sensors.h"

#include <algorithm>

#include "shared_config.h"
#include "teensy/include/config.h"

void Sensors::init() {
    analogReadResolution(12);
    analogReadAveraging(16);

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
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
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
        // The robot is on the line, proceed to compute line parameters

        const auto lineSize = (float)payload.line.size / 100;
        const auto lineAngleBisector = _line.angleBisector;

        // This part records jumps in line angle bisectors by referencing the
        // history by updating switchCount
        const auto hasJump = [this](float lastAngleBisector) {
            return smallerBearingDiff(clipBearing(_line.angleBisector),
                                      clipBearing(lastAngleBisector)) >
                   LINE_ANGLE_SWITCH_ANGLE;
        };
        if (!_lineAngleBisectorHistory.empty() &&
            std::all_of(_lineAngleBisectorHistory.begin(),
                        _lineAngleBisectorHistory.end(), hasJump)) {
            // There's a huge jump in the line angle bisector while in the line
            // so we have probbaly switched sides, increment the counter.
            ++switchCount;
        } else {
            // Only add to the line angle history if we haven't switched sides
            // as we want it to reflect the last line angle at the last side.
            // This also filters out noise in the line angle history.
            _lineAngleBisectorHistory.push_back(lineAngleBisector);
            while (_lineAngleBisectorHistory.size() > LINE_ANGLE_HISTORY)
                _lineAngleBisectorHistory.pop_front();
        }

        // Now, we can use switchCount to determine if the robot switched sides
        if (switchCount >= LINE_ANGLE_SWITCH_COUNT) {
            // The robot has probably switched sides
            if (_isInside) {
                // Robot moved from the inner half to the outer half of the line
                _line.depth = 1 - lineSize / 2;
            } else {
                // Robot moved from the outer half to the inner half of the line
                _line.depth = lineSize / 2;
                _line.angleBisector = clipAngle(_line.angleBisector + 180);
            }
            _isInside = !_isInside;

            // Register the switch
            switchCount = 0;
            _lineAngleBisectorHistory.clear();
            _lineAngleBisectorHistory.push_back(
                (float)payload.line.angleBisector / 100);
        } else if (_isInside) {
            // The robot didn't switch sides, on the inner half of the line
            _line.depth = lineSize / 2;
            _line.angleBisector = clipAngle(_line.angleBisector + 180);
        } else {
            // The robot didn't switch sides, on the outer half of the line
            _line.depth = 1 - lineSize / 2;
        }
    } else {
        // The robot is not on the line

        if (!_lineAngleBisectorHistory.empty()) {
            // The robot just exited the line, snap the depth to 0 or 1
            if (_isInside) {
                _line.depth = 0; // It exited on the inside of the line
            } else {
                _line.depth = 1; // It exited on the outside of the line
            }
        }

        // Reset the line angle bisector history
        _lineAngleBisectorHistory.clear();
    }

    // Consider the STM32 MUX to be initialised
    _muxInit = true;
}

void Sensors::onTofPacket(const byte *buf, size_t size) {
    // Load payload
    TOFTXPayload payload;
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));

    // Update bounds data if we have the robot angle
    if (_robot.angle.established()) {
        _bounds.front.newData = payload.bounds.front.newData;
        _bounds.back.newData = payload.bounds.back.newData;
        _bounds.left.newData = payload.bounds.left.newData;
        _bounds.right.newData = payload.bounds.right.newData;

        // Calculate bounds, taking into account robot angle
        // TODO: Come up with a more robust way to do this that fits a rectangle
        // to the measured distances
        if (payload.bounds.front.value == NO_BOUNDS ||
            payload.bounds.front.value > TOF_MAX_DISTANCE * 10) {
            _bounds.front.value = NAN;
        } else {
            const auto measurement = (float)payload.bounds.front.value / 10;
            _bounds.front.value =
                measurement * fabsf(cosfd(_robot.angle.value));
        }
        if (payload.bounds.back.value == NO_BOUNDS ||
            payload.bounds.back.value > TOF_MAX_DISTANCE * 10) {
            _bounds.back.value = NAN;
        } else {
            const auto measurement = (float)payload.bounds.back.value / 10;
            _bounds.back.value = measurement * fabsf(cosfd(_robot.angle.value));
        }
        if (payload.bounds.left.value == NO_BOUNDS ||
            payload.bounds.left.value > TOF_MAX_DISTANCE * 10) {
            _bounds.left.value = NAN;
        } else {
            const auto measurement = (float)payload.bounds.left.value / 10;
            _bounds.left.value = measurement * fabsf(cosfd(_robot.angle.value));
        }
        if (payload.bounds.right.value == NO_BOUNDS ||
            payload.bounds.right.value > TOF_MAX_DISTANCE * 10) {
            _bounds.right.value = NAN;
        } else {
            const auto measurement = (float)payload.bounds.right.value / 10;
            _bounds.right.value =
                measurement * fabsf(cosfd(_robot.angle.value));
        }

        _updateRobotPosition();
    }

    // Update bluetooth data
    _otherRobot = payload.bluetoothInboundPayload;

    // Consider the STM32 TOF to be initialised
    _tofInit = true;
}

void Sensors::onImuPacket(const byte *buf, size_t size) {
    // Load payload
    IMUTXPayload payload;
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));

    const auto robotAngle = (float)payload.imu.robotAngle / 100;
    // If this is the first reading, record down the initial angle offset
    if (!_imuInit)
        // We can assume the value has already stabilised as the Teensy can
        // be reset separately from the IMU, after the IMU has been on for a
        // while
        _robotAngleOffset = robotAngle;

    // Update new flag
    _robot.angle.newData = payload.imu.newData;

    // Update robot angle
    _robot.angle.value = clipAngle(robotAngle - _robotAngleOffset);

    // Consider the STM32 IMU to be initialised
    _imuInit = true;
}

void Sensors::onCoralPacket(const byte *buf, size_t size) {
    // Load payload
    CoralTXPayload payload;
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
    memcpy(&payload, buf, sizeof(payload));

    // Update ball data
    _ball.newData = payload.camera.newData;
    _ball.value = {payload.camera.ballAngle != INT16_MAX
                       ? (float)payload.camera.ballAngle / 100
                       : NAN,
                   payload.camera.ballDistance != UINT16_MAX
                       ? (float)payload.camera.ballDistance / 100
                       : NAN};

    // Update goal data
    _goals.newData = payload.camera.newData;
#if TARGET_BLUE_GOAL
    _goals.offensive = {payload.camera.blueGoalAngle != INT16_MAX
                            ? (float)payload.camera.blueGoalAngle / 100
                            : NAN,
                        payload.camera.blueGoalDistance != UINT16_MAX
                            ? (float)payload.camera.blueGoalDistance / 100
                            : NAN};
    _goals.defensive = {payload.camera.yellowGoalAngle != INT16_MAX
                            ? (float)payload.camera.yellowGoalAngle / 100
                            : NAN,
                        payload.camera.yellowGoalDistance != UINT16_MAX
                            ? (float)payload.camera.yellowGoalDistance / 100
                            : NAN};
#else
    _goals.offensive = {payload.camera.yellowGoalAngle != INT16_MAX
                            ? (float)payload.camera.yellowGoalAngle / 100
                            : NAN,
                        payload.camera.yellowGoalDistance != UINT16_MAX
                            ? (float)payload.camera.yellowGoalDistance / 100
                            : NAN};
    _goals.defensive = {payload.camera.blueGoalAngle != INT16_MAX
                            ? (float)payload.camera.blueGoalAngle / 100
                            : NAN,
                        payload.camera.blueGoalDistance != UINT16_MAX
                            ? (float)payload.camera.blueGoalDistance / 100
                            : NAN};
#endif

    _updateRobotPosition();

    // Consider the Coral to be initialised
    _coralInit = true;
}

void Sensors::_updateRobotPosition() {
    if (_goals.offensive.exists() && _goals.defensive.exists()) {
        // We can see both goals, so we can perform localisation with that :D

        // We use an algorithm that allows us to minimise error propagated by
        // goal distance and rely more on goal angle

        // Computer a "real" center vector from the two goal vectors
        const auto fakeCenter = (goals.offensive + goals.defensive) / 2;
        const auto scalingFactor =
            (goals.offensive - fakeCenter).distance / HALF_GOAL_SEPARATION;
        const auto realCenter = fakeCenter * scalingFactor;

        // Update robot position
        _robot.position.value = -realCenter;
    } else if (_goals.offensive.exists()) {
        // Compute a "fake" center vector from the offensive goal vector
        const Vector realGoalToCenter = {180 - _robot.angle.value,
                                         HALF_GOAL_SEPARATION};
        const auto fakeCenter = _goals.offensive + realGoalToCenter;

        // Update robot position
        _robot.position.value = -fakeCenter;
    } else if (_goals.defensive.exists()) {
        // Compute a "fake" center vector from the defensive goal vector
        const Vector realGoalToCenter = {-_robot.angle.value,
                                         HALF_GOAL_SEPARATION};
        const auto fakeCenter = _goals.defensive + realGoalToCenter;

        // Update robot position
        _robot.position.value = -fakeCenter;
    } else {
        // We can't see any goals, but we might be able to use the TOFs :0

        if ((_bounds.front.valid() || _bounds.back.valid()) &&
            (_bounds.left.valid() || _bounds.right.valid())) {
            // Compute x
            float x;
            if (_bounds.left.valid() && _bounds.right.valid()) {
                x = (_bounds.left.value - _bounds.right.value) *
                    (_bounds.left.value + _bounds.right.value) / FIELD_WIDTH;
            } else if (_bounds.left.valid()) {
                x = -(FIELD_WIDTH / 2 - _bounds.left.value);
            } else {
                x = FIELD_WIDTH / 2 - _bounds.right.value;
            }

            // Compute y
            float y;
            if (_bounds.front.valid() && _bounds.back.valid()) {
                y = (_bounds.back.value - _bounds.front.value) *
                    (_bounds.front.value + _bounds.back.value) / FIELD_LENGTH;
            } else if (_bounds.front.valid()) {
                y = FIELD_LENGTH / 2 - _bounds.front.value;
            } else {
                y = -(FIELD_LENGTH / 2 - _bounds.back.value);
            }

            // Update robot position
            _robot.position.value = Vector::fromPoint({x, y});
        } else {
            // We can't use the TOFs, so we can't localise :(
            _robot.position.value = {};
        }
    }
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
    _robot.angle.newData = false;
    _robot.position.newData = false;
    _bounds.front.newData = false;
    _bounds.back.newData = false;
    _bounds.left.newData = false;
    _bounds.right.newData = false;
    _otherRobot.newData = false;
    _ball.newData = false;
}
