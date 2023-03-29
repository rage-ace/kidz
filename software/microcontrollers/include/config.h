#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#include "angle.h"
#include "serial.h"

// NULL values
#define NO_LINE_INT16  INT16_MAX
#define NO_LINE_UINT8  UINT8_MAX
#define NO_ANGLE       INT16_MAX
#define NO_BOUNDS      UINT16_MAX
#define NO_BALL_INT16  INT16_MAX
#define NO_BALL_UINT16 UINT16_MAX

struct _RenewableData {
    bool newData = true;
};

struct Line : _RenewableData {
    int16_t angle = NO_LINE_INT16; // -179(.)99° to 180(.)00°
    uint8_t size = NO_LINE_UINT8;  // 0(.)00 to 1(.)00

    bool exists() { return angle != NO_LINE_INT16 && size != NO_LINE_UINT8; }
};

struct RobotAngle : _RenewableData {
    int16_t angle = NO_ANGLE; // -179(.)99º to +180(.)00º

    bool exists() { return angle != NO_ANGLE; }

    RobotAngle withOffset(int32_t offset) {
        RobotAngle newAngle;
        newAngle.newData = newData;
        newAngle.angle = clipAngle(angle - offset);
        return newAngle;
    }
};

struct Bounds : _RenewableData {
    uint16_t front = NO_BOUNDS; // 0(.)0 cm to 400(.)0 cm
    uint16_t back = NO_BOUNDS;  // 0(.)0 cm to 400(.)0 cm
    uint16_t left = NO_BOUNDS;  // 0(.)0 cm to 400(.)0 cm
    uint16_t right = NO_BOUNDS; // 0(.)0 cm to 400(.)0 cm

    // Bounds in all four directions are established
    bool established() {
        return front != NO_BOUNDS && back != NO_BOUNDS && left != NO_BOUNDS &&
               right != NO_BOUNDS;
    }

    void set(uint8_t index, uint16_t value) {
        switch (index) {
        case 0:
            front = value;
            break;
        case 1:
            back = value;
            break;
        case 2:
            left = value;
            break;
        case 3:
            right = value;
            break;
        }
    }
};

struct Ball : _RenewableData {
    int16_t angle = NO_BALL_INT16; // -179(.)99° to 180(.)00°
    uint16_t distance = NO_BALL_UINT16;

    bool exists() {
        return angle != NO_BALL_INT16 && distance != NO_BALL_UINT16;
    }
};

struct BluetoothPayload : _RenewableData { // This should be symmetric
    // TODO
    byte testByte = 0x00;

    // Creates a new BluetoothPayload with newData set as true
    static BluetoothPayload create(byte testByte) {
        BluetoothPayload newPayload;
        newPayload.newData = true;
        newPayload.testByte = testByte;
        return newPayload;
    }
};

struct MUXTXPayload {
    Line line;
};
struct MUXRXPayload {};

struct IMUTXPayload {
    RobotAngle robotAngle;
};
struct IMURXPayload {
    bool calibrating = false;
};

struct TOFTXPayload {
    Bounds bounds;
    BluetoothPayload bluetoothInboundPayload;
};
struct TOFRXPayload {
    BluetoothPayload bluetoothOutboundPayload;
};

struct CoralTXPayload {
    Ball ball;
};
struct CoralRXPayload {};

// Shared serial information
#define MONITOR_BAUD_RATE      115200
#define TEENSY_MUX_BAUD_RATE   115200
#define TEENSY_IMU_BAUD_RATE   115200
#define TEENSY_TOF_BAUD_RATE   115200
#define TEENSY_CORAL_BAUD_RATE 115200

#define MUX_TX_PACKET_SIZE     sizeof(MUXTXPayload) + 2U
#define MUX_TX_SYNC_START_BYTE 0b11010110
#define MUX_TX_SYNC_END_BYTE   0b00110010

#define MUX_RX_PACKET_SIZE     sizeof(MUXRXPayload) + 2U
#define MUX_RX_SYNC_START_BYTE 0b11010110
#define MUX_RX_SYNC_END_BYTE   0b00110010

#define IMU_TX_PACKET_SIZE     sizeof(IMUTXPayload) + 2U
#define IMU_TX_SYNC_START_BYTE 0b11010110
#define IMU_TX_SYNC_END_BYTE   0b00110010

#define IMU_RX_PACKET_SIZE     sizeof(IMURXPayload) + 2U
#define IMU_RX_SYNC_START_BYTE 0b11010110
#define IMU_RX_SYNC_END_BYTE   0b00110010

#define TOF_TX_PACKET_SIZE     sizeof(TOFTXPayload) + 2U
#define TOF_TX_SYNC_START_BYTE 0b11010110
#define TOF_TX_SYNC_END_BYTE   0b00110010

#define TOF_RX_PACKET_SIZE     sizeof(TOFRXPayload) + 2U
#define TOF_RX_SYNC_START_BYTE 0b11010110
#define TOF_RX_SYNC_END_BYTE   0b00110010

#define CORAL_TX_PACKET_SIZE     sizeof(CoralTXPayload) + 2U
#define CORAL_TX_SYNC_START_BYTE 0b11010110
#define CORAL_TX_SYNC_END_BYTE   0b00110010

#define CORAL_RX_PACKET_SIZE     sizeof(CoralRXPayload) + 2U
#define CORAL_RX_SYNC_START_BYTE 0b11010110
#define CORAL_RX_SYNC_END_BYTE   0b00110010

// Loop times (measured on 2023-03-18)
// #define _TEENSY_LOOP_TIME 0U // in µs, default:   260 (min=  172, max=  348)
// #define _MUX_LOOP_TIME    0U // in µs, default:  2787 (min= 2783, max= 2860)
// #define _IMU_LOOP_TIME    0U // in µs, default:  1262 (min=  917, max= 2238)
// #define _TOF_LOOP_TIME    0U // in µs, default: 30972 (min=25295, max=31470)
// #define _CORAL_LOOP_TIME  0U // in µs

#endif
