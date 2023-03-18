#ifndef CONFIG_H
#define CONFIG_H

#define UINT16_NO_LINE UINT16_MAX
#define UINT8_NO_LINE  UINT8_MAX
#define NO_BEARING     UINT16_MAX
#define NO_BOUNDARY    UINT16_MAX

struct Line {
    uint16_t bearing = UINT16_NO_LINE; // 0(.)00° to 359(.)99°
    uint8_t size = UINT8_NO_LINE;      // 0(.)00 to 1(.)00

    bool exists() { return bearing != UINT16_NO_LINE && size != UINT8_NO_LINE; }
};

struct Boundary {
    uint16_t front = NO_BOUNDARY; // 0(.)0 cm to 400(.)0 cm
    uint16_t back = NO_BOUNDARY;  // 0(.)0 cm to 400(.)0 cm
    uint16_t left = NO_BOUNDARY;  // 0(.)0 cm to 400(.)0 cm
    uint16_t right = NO_BOUNDARY; // 0(.)0 cm to 400(.)0 cm

    // Boundary in all four directions are established
    bool established() {
        return front != NO_BOUNDARY && back != NO_BOUNDARY &&
               left != NO_BOUNDARY && right != NO_BOUNDARY;
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

struct MUXTXPayload {
    Line line;
};

struct IMUTXPayload {
    uint16_t bearing = NO_BEARING;
};

struct IMURXPayload {
    bool calibrating = false;
};

struct TOFTXPayload {
    Boundary boundary;
};

#define MONITOR_BAUD_RATE      115200
#define TEENSY_MUX_BAUD_RATE   115200
#define TEENSY_IMU_BAUD_RATE   115200
#define TEENSY_TOF_BAUD_RATE   115200
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

#define TOF_TX_PACKET_SIZE     sizeof(TOFTXPayload) + 2U
#define TOF_TX_SYNC_START_BYTE 0b11010110
#define TOF_TX_SYNC_END_BYTE   0b00110010

#endif
