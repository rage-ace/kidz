#ifndef CONFIG_H
#define CONFIG_H

#define UINT16_NO_LINE UINT16_MAX
#define UINT8_NO_LINE  UINT8_MAX

struct LineData {
    bool isPresent = false;
    uint16_t bearing = UINT16_NO_LINE; // 0(.)00° to 360(.)00°
    uint8_t size = UINT8_NO_LINE;      // 0(.)00 to 1(.)00
};

#define MONITOR_BAUD_RATE      115200
#define TEENSY_MUX_BAUD_RATE   115200
#define TEENSY_CORAL_BAUD_RATE 115200
#define TEENSY_TOF_BAUD_RATE   115200
#define TEENSY_IMU_BAUD_RATE   115200

#define MUX_TX_PACKET_SIZE     sizeof(LineData) + 2U
#define MUX_TX_SYNC_START_BYTE 0b11010110
#define MUX_TX_SYNC_END_BYTE   0b00110010

// #define CAMERA_PACKET_SIZE       sizeof(CameraData) + 1U
// #define CAMERA_SYNC_START        0b00101010

#endif
