#ifndef TEENSY_CONFIG_H
#define TEENSY_CONFIG_H

#include <HardwareSerial.h>
#include <array>

#include "shared_config.h"

// Flags
#define DEBUG_TEENSY
// #define DEBUG_MUX
// #define DEBUG_IMU
// #define DEBUG_TOF
// #define DEBUG_CORAL
// #define CALIBRATE_IMU
// #define CALIBRATE_MUX
// #define CALIBRATE_ROBOT_ANGLE_CONTROLLER

// Macro Flags
#ifdef DEBUG_TEENSY
    #define DEBUG
#endif
#ifdef DEBUG_MUX
    #define DEBUG
#endif
#ifdef DEBUG_IMU
    #define DEBUG
#endif
#ifdef DEBUG_TOF
    #define DEBUG
#endif
#ifdef DEBUG_CORAL
    #define DEBUG
#endif
#ifdef CALIBRATE_IMU
    #define CALIBRATE
    #define DONT_WAIT_FOR_SUBPROCESSOR_INIT
#endif
#ifdef CALIBRATE_MUX
    #define CALIBRATE
    #define DONT_WAIT_FOR_SUBPROCESSOR_INIT
#endif
#ifdef CALIBRATE_ROBOT_ANGLE_CONTROLLER
    #define CALIBRATE
#endif

// Pins
#define PIN_LED_DEBUG 13

#define PIN_RX_CORAL 1 // Serial1
#define PIN_TX_CORAL 0
#define CORAL_SERIAL Serial1

#define PIN_RX_STM32_MUX 8 // Serial2
#define PIN_TX_STM32_MUX 7
#define MUX_SERIAL       Serial2

#define PIN_RX_STM32_TOF 14 // Serial3
#define PIN_TX_STM32_TOF 15
#define TOF_SERIAL       Serial3

#define PIN_RX_STM32_IMU 17 // Serial4
#define PIN_TX_STM32_IMU 16
#define IMU_SERIAL       Serial4

#define PIN_MOTOR_FL_PWM  2
#define PIN_MOTOR_FR_PWM  3
#define PIN_MOTOR_BL_PWM  4
#define PIN_MOTOR_BR_PWM  5
#define PIN_MOTOR_FL_DIR  9
#define PIN_MOTOR_FR_DIR  10
#define PIN_MOTOR_BL_DIR  11
#define PIN_MOTOR_BR_DIR  12
#define MOTOR_FL_REVERSED true
#define MOTOR_FR_REVERSED true
#define MOTOR_BL_REVERSED true
#define MOTOR_BR_REVERSED false

#define PIN_DRIBBLER_PWM 18
#define PIN_LIGHTGATE    22
#define PIN_KICKER       23

// Sensor Config
#define LIGHTGATE_THRESHOLD 1200

// Movement Config
#define DRIVE_STALL_SPEED          (int16_t)35
#define DRIVE_MAX_SPEED            (int16_t)150
#define DRIBBLER_ARM_SPEED         (int16_t)130
#define DRIBBLER_SPEED             (int16_t)150
#define KICKER_ACTIVATION_DURATION 100 // in ms

#define BALL_MOVEMENT_A 1e-3F
#define BALL_MOVEMENT_B 16.0F

// #define STOP_WITHIN_LINE_BALL_RANGE  40
// #define STOP_WITHIN_LINE_STRENGTH    0.2
// #define BEGIN_ENTERING_LINE_STRENGTH 0.5
// #define ENTER_LINE_SPEED_MULTIPLIER  150.0
// #define ENTER_LINE_MAX_SPEED         200

// PID Constants
// kU = 18e-2F
#define KP_ROBOT_ANGLE     14e0F
#define KI_ROBOT_ANGLE     0e-6F
#define KD_ROBOT_ANGLE     90e4F
#define MIN_DT_ROBOT_ANGLE 5000 // in µs, minimum value for kD to have effect

#endif