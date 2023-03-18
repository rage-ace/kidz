#ifndef TEENSY_CONFIG_H
#define TEENSY_CONFIG_H

// Flags
#define DEBUG_TEENSY
// #define DEBUG_MUX
// #define DEBUG_IMU
// #define DEBUG_TOF
// #define CALIBRATE_IMU

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
#ifdef CALIBRATE_IMU
    #define CALIBRATE
#endif

// Pins
#define PIN_LED_DEBUG 13

#define PIN_RX_CORAL 1 // Serial1
#define PIN_TX_CORAL 0

#define PIN_RX_STM32_MUX 8 // Serial2
#define PIN_TX_STM32_MUX 7

#define PIN_RX_STM32_TOF 14 // Serial3
#define PIN_TX_STM32_TOF 15

#define PIN_RX_STM32_IMU 17 // Serial4
#define PIN_TX_STM32_IMU 16

#define PIN_MOTOR_FL_DIR 9  // Forward is LOW, backward is HIGH
#define PIN_MOTOR_FR_DIR 10 // Forward is LOW, backward is HIGH
#define PIN_MOTOR_BL_DIR 11 // Forward is LOW, backward is HIGH
#define PIN_MOTOR_BR_DIR 12 // Forward is HIGH, backward is LOW

#define PIN_MOTOR_FL_PWM 2
#define PIN_MOTOR_FR_PWM 3
#define PIN_MOTOR_BL_PWM 4
#define PIN_MOTOR_BR_PWM 5

#define PIN_DRIBBLER_PWM 18
#define PIN_LIGHTGATE    22
#define PIN_KICKER       23

#define CORAL_SERIAL Serial1
#define MUX_SERIAL   Serial2
#define TOF_SERIAL   Serial3
#define IMU_SERIAL   Serial4

// Movement Config

// Should ideally be <=1.0F to preserve resolution
#define ANGULAR_VELOCITY_MULTIPLIER 0.25F
#define DRIVE_STALL_SPEED           50
#define DRIVE_MAX_SPEED             1024

// #define MOVE_ABOUT_BALL_MULTIPLIER 13.0

// #define STOP_WITHIN_LINE_BALL_RANGE  40
// #define STOP_WITHIN_LINE_STRENGTH    0.2
// #define BEGIN_ENTERING_LINE_STRENGTH 0.5
// #define ENTER_LINE_SPEED_MULTIPLIER  150.0
// #define ENTER_LINE_MAX_SPEED         200

// PID Gains
#define KP_BEARING 0.1F
#define KI_BEARING 0.0F
#define KD_BEARING 0.0F

#endif