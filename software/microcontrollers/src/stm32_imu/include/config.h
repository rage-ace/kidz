#ifndef STM32_IMU_CONFIG_H
#define STM32_IMU_CONFIG_H

#include <stm32f103c_variant_generic.h>

#include "config.h"
#include "serial.h"

// #define DEBUG

// Baud Rates
#define DEBUG_BAUD_RATE 115200

// Pins
#define PIN_LED_DEBUG PC13
#define PIN_RX_DEBUG  PB7
#define PIN_TX_DEBUG  PB6

#define PIN_RX PA3
#define PIN_TX PA2

#define PIN_SDA_IMU PB9
#define PIN_SCL_IMU PB8

// Tell the compiler the specific pins each HardwareSerial uses
#ifdef DEBUG
    #define PIN_SERIAL1_RX PB7
    #define PIN_SERIAL1_TX PB6
#endif

#define PIN_SERIAL2_RX PA3
#define PIN_SERIAL2_TX PA2

// Serial Ports
SerialManager TeensySerial = SerialManager(
    Serial2, TEENSY_IMU_BAUD_RATE, IMU_RX_PACKET_SIZE, IMU_RX_SYNC_START_BYTE,
    IMU_RX_SYNC_END_BYTE, IMU_TX_PACKET_SIZE, IMU_TX_SYNC_START_BYTE,
    IMU_TX_SYNC_END_BYTE);
#define DEBUG_SERIAL Serial1

// EEPROM Addresses
#define EEPROM_ADDRESS_HAS_OFFSETS 0x000
#define EEPROM_ADDRESS_OFFSETS     0x000 + sizeof(bool)

// I2C Addresses
#define I2C_ADDRESS_BNO055 0x29

#endif
