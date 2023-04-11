#ifndef CONFIG_H
#define CONFIG_H

#include <array>
#include <cstdint>

#include "angle.h"
#include "vector.h"

// NULL values
#define NO_LINE_INT16  INT16_MAX
#define NO_LINE_UINT8  UINT8_MAX
#define NO_ANGLE       INT16_MAX
#define NO_BOUNDS      UINT16_MAX
#define NO_BALL_INT16  INT16_MAX
#define NO_BALL_UINT16 UINT16_MAX

// Raw sensor data transmitted over serial
struct _RenewableData {
    bool newData = true;
};
struct LineData : _RenewableData {
    int16_t angleBisector = NO_LINE_INT16; // -179(.)99° to 180(.)00°
    uint8_t size = NO_LINE_UINT8;          // 0(.)00 to 1(.)00

    bool exists() {
        return angleBisector != NO_LINE_INT16 && size != NO_LINE_UINT8;
    }
};
struct IMUData : _RenewableData {
    int16_t robotAngle = NO_ANGLE; // -179(.)99º to +180(.)00º
};
struct BoundsData {
    struct Bound : _RenewableData {
        uint16_t value = NO_BOUNDS; // 0(.)0 cm to 400(.)0 cm
    } front, back, left, right;

    void set(uint8_t index, uint16_t value = NO_BOUNDS) {
        switch (index) {
        case 0:
            front.value = value;
            front.newData = value != NO_BOUNDS;
            break;
        case 1:
            back.value = value;
            back.newData = value != NO_BOUNDS;
            break;
        case 2:
            left.value = value;
            left.newData = value != NO_BOUNDS;
            break;
        case 3:
            right.value = value;
            right.newData = value != NO_BOUNDS;
            break;
        }
    }

    void markAsOld() {
        front.newData = false;
        back.newData = false;
        left.newData = false;
        right.newData = false;
    }
};
struct CameraData : _RenewableData {
    int16_t ballAngle = NO_BALL_INT16;            // -179(.)99° to 180(.)00°
    uint16_t ballDistance = NO_BALL_UINT16;       // 0(.)0 cm to ~400(.)0 cm
    int16_t blueGoalAngle = NO_BALL_INT16;        // -179(.)99° to 180(.)00°
    uint16_t blueGoalDistance = NO_BALL_UINT16;   // 0(.)0 cm to ~400(.)0 cm
    int16_t yellowGoalAngle = NO_BALL_INT16;      // -179(.)99° to 180(.)00°
    uint16_t yellowGoalDistance = NO_BALL_UINT16; // 0(.)0 cm to ~400(.)0 cm
};
struct BluetoothPayload : _RenewableData { // This should be symmetric
    bool masterIsStriker = true;
    Vector ball;  // relative to field center
    Vector robot; // relative to field center

    // Creates a new BluetoothPayload with newData set as true
    static BluetoothPayload create(bool masterIsStriker, Vector ball,
                                   Vector robot) {
        BluetoothPayload newPayload;
        newPayload.newData = true;
        newPayload.masterIsStriker = masterIsStriker;
        newPayload.ball = ball;
        newPayload.robot = robot;
        return newPayload;
    }
};

// Serial packet payloads
struct MUXTXPayload {
    LineData line;
};
struct MUXRXPayload {
    bool calibrating = false;
    // I couldn't find a way to send the thresholds over serial reliably ;-;
    // TODO: Try adding a CRC check?
};

struct IMUTXPayload {
    IMUData imu;
};
struct IMURXPayload {
    bool calibrating = false;
};

struct TOFTXPayload {
    BoundsData bounds;
    BluetoothPayload bluetoothInboundPayload;
};
struct TOFRXPayload {
    BluetoothPayload bluetoothOutboundPayload;
};

struct CoralTXPayload {
    CameraData camera;
};
struct CoralRXPayload {};

// Shared serial information
#define MONITOR_BAUD_RATE 115200
// 1M baud works, 2M doesn't
#define TEENSY_MUX_BAUD_RATE   1000000
#define TEENSY_IMU_BAUD_RATE   1000000
#define TEENSY_TOF_BAUD_RATE   1000000
#define TEENSY_CORAL_BAUD_RATE 1000000

// Loop times (measured on 2023-03-18)
// #define _TEENSY_LOOP_TIME 0U // in µs, default:   260 (min=  172, max=  348)
// #define _MUX_LOOP_TIME    0U // in µs, default:  2787 (min= 2783, max= 2860)
// #define _IMU_LOOP_TIME    0U // in µs, default:  1262 (min=  917, max= 2238)
// #define _TOF_LOOP_TIME    0U // in µs, default: 30972 (min=25295, max=31470)
// #define _CORAL_LOOP_TIME  0U // in µs

#endif
