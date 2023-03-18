#ifndef STM32_MUX_CONFIG_H
#define STM32_MUX_CONFIG_H

#include <cstdint>
#include <stm32f103c_variant_generic.h>

#define DEBUG false

// Baud Rates
#define DEBUG_BAUD_RATE 115200

// Pins
#define PIN_LED_DEBUG PA8
#define PIN_RX_DEBUG  PB7
#define PIN_TX_DEBUG  PB6

#define PIN_RX PA3
#define PIN_TX PA2

#define PIN_LIGHTGATE PA1

#define PIN_LDRMUX1_SIG PB0
#define PIN_LDRMUX1_S0  PA10
#define PIN_LDRMUX1_S1  PA9
#define PIN_LDRMUX1_S2  PA11
#define PIN_LDRMUX1_S3  PA12

#define PIN_LDRMUX2_SIG PB1
#define PIN_LDRMUX2_S0  PA7
#define PIN_LDRMUX2_S1  PA6
#define PIN_LDRMUX2_S2  PA5
#define PIN_LDRMUX2_S3  PA4

#define TEENSY_SERIAL Serial
#define DEBUG_SERIAL  Serial1

// Light Sensor Config
enum LDRMUX { MUX1 = 0x00, MUX2 = 0x01 };

#define LDR_CALIBRATION_DURATION 5000 // In milliseconds

#define LDR_PIN_COUNT 16 * 2 // The number of possible input pins on the MUXs
#define LDR_COUNT     30     // The number of actual photodiodes
const uint8_t LDR_MAP[LDR_PIN_COUNT] = {
    255, 14, 13, 12, 11, 10, 9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
    255, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15,
};
const uint8_t LDR_MAP_REVERSE[LDR_COUNT] = {
    15, 14, 13, 12, 11, 10, 9,  8,  7,  6,  5,  4,  3,  2,  1,
    31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17,
};
const float LDR_BEARINGS[LDR_COUNT] = {
    8.1461,   26.9079,  39.7366,  51.1272,  61.88670, 73.1367,
    84.3867,  95.6367,  106.8867, 118.1367, 129.1957, 141.0268,
    152.4409, 162.7907, 174.2699, 185.7539, 197.2331, 208.3438,
    219.6814, 231.0705, 241.8867, 253.1367, 264.3867, 275.6367,
    286.8867, 298.1367, 309.2879, 320.4649, 333.8737, 351.8859,
};
const uint16_t LDR_THRESHOLDS[LDR_COUNT] = {
    709, 728, 717, 745, 756, 776, 797, 787, 740, 802, 834, 798, 796, 776, 772,
    800, 773, 786, 783, 780, 760, 766, 759, 735, 720, 748, 770, 757, 734, 775,
};

#endif
