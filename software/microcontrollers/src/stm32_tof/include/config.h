#ifndef STM32_TOF_CONFIG_H
#define STM32_TOF_CONFIG_H

#include <VL53L1X.h>
#include <cstdint>
#include <stm32f103c_variant_generic.h>

#include "config.h"
#include "serial.h"

// #define DEBUG

// Baud Rates
#define BLUETOOTH_BAUD_RATE 9600
#define DEBUG_BAUD_RATE     115200

// Pins
#define PIN_LED_DEBUG PC13
#define PIN_RX_DEBUG  PB7
#define PIN_TX_DEBUG  PB6

#define PIN_RX PA3
#define PIN_TX PA2

#define PIN_RX_BLUETOOTH PB11
#define PIN_TX_BLUETOOTH PB10

#define PIN_SDA_TOF PB9
#define PIN_SCL_TOF PB8

#define PIN_XSHUT_FRONT PB14
#define PIN_XSHUT_BACK  PB2
#define PIN_XSHUT_LEFT  PA11
#define PIN_XSHUT_RIGHT PB12

#define PIN_RING      PA9
#define PIN_LEFT_EYE  PB0
#define PIN_RIGHT_EYE PB1

// Tell the compiler the specific pins each HardwareSerial uses
#ifdef DEBUG
    #define PIN_DEBUG_SERIAL_RX PB7
    #define PIN_DEBUG_SERIAL_TX PB6
    #define DEBUG_SERIAL        Serial1
#endif

#define PIN_TEENSY_SERIAL_RX PA3
#define PIN_TEENSY_SERIAL_TX PA2
#define TEENSY_SERIAL        Serial2

#define PIN_BLUETOOTH_SERIAL_RX PB11
#define PIN_BLUETOOTH_SERIAL_TX PB10
#define BLUETOOTH_SERIAL        Serial3

// EEPROM Addresses

// I2C Addresses
#define I2C_ADDRESS_TOF_START 0x2A

// Time-Of-Flight Sensors
// Read: https://www.pololu.com/product/3415
struct TOFConfig {
    uint8_t xshutPin;
    VL53L1X::DistanceMode distanceMode; // Short, Medium, Long
    uint16_t measurementPeriod;         // in us, 20 ms to 50 ms for Short,
                                        // 33 ms to 50 ms for Medium and Long
    uint8_t intermeasurementPeriod;     // in ms, at least measurementPeriod
};

#define TOF_COUNT 4
const TOFConfig TOF_CONFIGS[TOF_COUNT] = {
    {PIN_XSHUT_FRONT, VL53L1X::Short, 33000, 33}, // Front
    {PIN_XSHUT_BACK, VL53L1X::Short, 33000, 33},  // Back
    {PIN_XSHUT_LEFT, VL53L1X::Short, 20000, 20},  // Left
    {PIN_XSHUT_RIGHT, VL53L1X::Short, 20000, 20}, // Right
};

#endif
