#ifndef STM32_MUX_CONFIG_H
#define STM32_MUX_CONFIG_H

#include <array>
#include <cstdint>
#include <stm32f103c_variant_generic.h>

#include "shared_config.h"

// #define DEBUG

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

// Tell the compiler the specific pins each HardwareSerial uses
#ifdef DEBUG
    #define PIN_DEBUG_SERIAL_RX PB7
    #define PIN_DEBUG_SERIAL_TX PB6
    #define DEBUG_SERIAL        Serial1
#endif

#define PIN_TEENSY_SERIAL_RX PA3
#define PIN_TEENSY_SERIAL_TX PA2
#define TEENSY_SERIAL        Serial2

// Light Sensor Config
enum LDRMUX { MUX1 = 0x00, MUX2 = 0x01 };

#define LDR_CALIBRATION_DURATION   10000 // In milliseconds
#define LDR_CALIBRATION_MULTIPLIER 0.7

#define LDR_PIN_COUNT 16 * 2 // The number of possible input pins on the MUXs
#define LDR_COUNT     30     // The number of actual photodiodes below the robot
const std::array<uint8_t, LDR_PIN_COUNT> LDR_MAP = {
    255, 14, 13, 12, 11, 10, 9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
    255, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15,
};
const std::array<uint8_t, LDR_COUNT> LDR_MAP_REVERSE = {
    15, 14, 13, 12, 11, 10, 9,  8,  7,  6,  5,  4,  3,  2,  1,
    31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17,
};
const std::array<float, LDR_COUNT> LDR_BEARINGS = {
    8.1461,   26.9079,  39.7366,  51.1272,  61.88670, 73.1367,
    84.3867,  95.6367,  106.8867, 118.1367, 129.1957, 141.0268,
    152.4409, 162.7907, 174.2699, 185.7539, 197.2331, 208.3438,
    219.6814, 231.0705, 241.8867, 253.1367, 264.3867, 275.6367,
    286.8867, 298.1367, 309.2879, 320.4649, 333.8737, 351.8859,
};
// // Computer Lab (multiplier=0.7), L1 A
// const std::array<uint16_t, LDR_COUNT> LDR_THRESHOLDS = {
//     3536, 3481, 3511, 3517, 3519, 3531, 3536, 3526, 3609, 3643,
//     3591, 3545, 3592, 1872, 3534, 3602, 3455, 3550, 3593, 3555,
//     3394, 3313, 3195, 3347, 3285, 3217, 3200, 3200, 3183, 3238,
// };
// // Home, L1 A
// const std::array<uint16_t, LDR_COUNT> LDR_THRESHOLDS = {
//     3160, 3125, 3147, 3144, 3150, 3116, 3172, 3152, 3364, 3361,
//     3146, 3173, 3295, 1507, 3206, 3232, 3020, 3162, 3280, 3143,
//     2882, 2901, 2747, 2609, 2874, 2763, 2797, 2687, 2735, 2660,
// };
// Home, L1 B
const std::array<uint16_t, LDR_COUNT> LDR_THRESHOLDS = {
    3182, 3145, 3242, 3081, 3210, 2971, 2927, 3082, 3116, 3294,
    3305, 3209, 3322, 3411, 3062, 3957, 3368, 3946, 3329, 3338,
    3330, 3335, 3318, 3294, 3453, 3226, 3378, 1757, 2971, 2958,
};
#define LDR_ACTIVATION_THRESHOLD 3

#endif
