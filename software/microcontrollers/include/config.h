#ifndef CONFIG_H
#define CONFIG_H

#define UINT16_NO_LINE UINT16_MAX
#define UINT8_NO_LINE  UINT8_MAX
#define NO_BEARING     UINT16_MAX

struct Line {
    uint16_t bearing = UINT16_NO_LINE; // 0(.)00° to 360(.)00°
    uint8_t size = UINT8_NO_LINE;      // 0(.)00 to 1(.)00

    bool exists() { return bearing != UINT16_NO_LINE && size != UINT8_NO_LINE; }
};

struct MUXTXPayload {
    Line line;
};

struct IMUTXPayload {
    uint16_t bearing = NO_BEARING;
};

struct IMURXPayload {
    bool calibrating = false;
};

#define MONITOR_BAUD_RATE      115200
#define TEENSY_MUX_BAUD_RATE   115200
#define TEENSY_CORAL_BAUD_RATE 115200
#define TEENSY_TOF_BAUD_RATE   115200
#define TEENSY_IMU_BAUD_RATE   115200
#define TEENSY_CORAL_BAUD_RATE 115200

#define MUX_TX_PACKET_SIZE     sizeof(MUXTXPayload) + 2U
#define MUX_TX_SYNC_START_BYTE 0b11010110
#define MUX_TX_SYNC_END_BYTE   0b00110010

#define IMU_TX_PACKET_SIZE     sizeof(IMUTXPayload) + 2U
#define IMU_TX_SYNC_START_BYTE 0b11010110
#define IMU_TX_SYNC_END_BYTE   0b00110010

#define IMU_RX_PACKET_SIZE     sizeof(IMURXPayload) + 2U
#define IMU_RX_SYNC_START_BYTE 0b11010110
#define IMU_RX_SYNC_END_BYTE   0b00110010

#endif
