#ifndef TEENSY_CONFIG_H
#define TEENSY_CONFIG_H

#include <HardwareSerial.h>
#include <array>

#include "shared_config.h"
#include "vector.h"

// Flags
#define MASTER
// #define DEBUG
// #define DEBUG_TEENSY
// #define DEBUG_MUX
// #define DEBUG_IMU
// #define DEBUG_TOF
// #define DEBUG_CORAL
// #define CALIBRATE_IMU
// #define CALIBRATE_MUX
#define CALIBRATE_ROBOT_ANGLE
// #define CALIBRATE_BALL_CURVE
// #define CALIBRATE_AVOIDANCE
// #define CALIBRATE_GOAL_MOVEMENT
#define DISABLE_DRIBBLER

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
#ifdef CALIBRATE_ROBOT_ANGLE
    #define CALIBRATE
#endif
#ifdef CALIBRATE_BALL_CURVE
    #define CALIBRATE
#endif
#ifdef CALIBRATE_AVOIDANCE
    #define CALIBRATE
#endif
#ifdef CALIBRATE_GOAL_MOVEMENT
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
#define MOTOR_FL_REVERSED false
#define MOTOR_FR_REVERSED false
#define MOTOR_BL_REVERSED false
#define MOTOR_BR_REVERSED false

#define PIN_DRIBBLER_PWM 18
#define PIN_LIGHTGATE    22
#define PIN_KICKER       23

// Sensor Config
// To have switched sides of the line, the line angle must have jumped this much
#define LINE_ANGLE_SWITCH_ANGLE 90.0F
// for this number of times
#define LINE_ANGLE_SWITCH_COUNT 2
// when compared to this number of previous line angles
#define LINE_ANGLE_HISTORY 2

#define LIGHTGATE_THRESHOLD 800

#define TOF_MAX_DISTANCE 130.0F // in cm

// true if blue is the offensive goal, false if yellow is the offensive goal
#define TARGET_BLUE_GOAL false

// Movement Config
#define DRIVE_STALL_SPEED          (int16_t)20
#define DRIVE_MAX_SPEED            (int16_t)1023
#define DRIBBLER_ARM_SPEED         (int16_t)128
#define DRIBBLER_SPEED             (int16_t)190 // stalls at 177
#define DRIBBLER_ARM_DURATION      3000         // in ms
#define KICKER_ACTIVATION_DURATION 100          // in ms
#define KICKER_COOLDOWN_DURATION   1500         // in ms

// ------------------------------- Robot Heading -------------------------------

#define KU_ROBOT_ANGLE 8.0e1F // tuned to ±0.5e1F
// ZN (no overshoot)  : kP=0.2  kI=0.4  kD=0.066
// ZN (some overshoot): kP=0.33 kI=0.66 kD=0.11
// ZN (classic)       : kP=0.6  kI=1.2  kD=0.075
#define KP_ROBOT_ANGLE                  0.5 * KU_ROBOT_ANGLE
#define KI_ROBOT_ANGLE                  1.2
#define KD_ROBOT_ANGLE                  10.5
#define MAXI_ROBOT_ANGLE                0.1e4F
#define MAX_SETPOINT_CHANGE_ROBOT_ANGLE 0.1F
#define MIN_DT_ROBOT_ANGLE              5000 // in µs, minimum value for kD to have effect
// Old PID version
// #define KP_ROBOT_ANGLE 3.6e1F  // tuned to ±0.2e1F
// #define KI_ROBOT_ANGLE 2.5e-5F // tuned to ±0.5e-5F
// #define KD_ROBOT_ANGLE 5.0e6F  // tuned to ±0.5e6F
// Old PD version but the kP was probably too high
// #define KP_ROBOT_ANGLE 5.8e1F // tuned to ±0.1e1F
// #define KI_ROBOT_ANGLE 0
// #define KD_ROBOT_ANGLE 2.7e6F // tuned to ±0.05e6F

// -------------------------------- Robot Speed --------------------------------

// This is the main speed the robot will be travelling at throughout gameplay,
// and one of the fastest speeds. Other aspects of gameplay will start from this
// speed and decelerate where necessary.
#define BASE_SPEED 500.0F

// ------------------------------- Ball Movement -------------------------------

// Multiplier = e^(DECAY * (START - Ball Distance))
// https://www.desmos.com/calculator/ixwhywbd5i
// This is the distance where we start curving maximally as the ball gets closer
#define BALL_MOVEMENT_MAX_CURVE 15.0F // in cm, tuned to ±1.0
// The larger the value, the faster the decay of the angle offset
#define BALL_MOVEMENT_DECAY 0.03F

#define BALL_MOVEMENT_START_DECELERATING 70.0F // in cm, from ball
#define BALL_MOVEMENT_START_SPEED        BASE_SPEED
#define BALL_MOVEMENT_END_SPEED          250.0F

#define BALL_MOVEMENT_FACE_BALL_DISTANCE 20.0F // in cm
#define BALL_MOVEMENT_MAX_HEADING        40.0F // in degrees

// ------------------------------- Goal Movement -------------------------------

#define GOAL_MOVEMENT_MULTIPLIER         0.2F
#define GOAL_MOVEMENT_START_DECELERATING 120.0F // in cm, from goal
#define GOAL_MOVEMENT_START_SPEED        800.0F
#define GOAL_MOVEMENT_END_SPEED          0.0F

// ------------------------------- Line Avoidance ------------------------------

#define LINE_AVOIDANCE_THRESHOLD        0.3F
#define LINE_AVOIDANCE_SPEED_MULTIPLIER 2000.0F // TODO: tune
#define LINE_AVOIDANCE_MAX_SPEED        1023.0F // TODO: tune

// ------------------------------ Wall Avoidance -------------------------------

// Note that our TOFs probably can't sense beyond 130 cm
#define WALL_AVOIDANCE_THRESHOLD   20.0F // in cm
#define WALL_AVOIDANCE_START_SPEED BASE_SPEED
#define WALL_AVOIDANCE_END_SPEED   50.0F

// ------------------------------- Localisation --------------------------------

#define MOVE_TO_PRECISION 3.5F // in cm

#define KU_MOVE_TO 8.0e1F // tuned to ±0.5e1F
// ZN (no overshoot)  : kP=0.2  kI=0.4  kD=0.066
// ZN (some overshoot): kP=0.33 kI=0.66 kD=0.11
// ZN (classic)       : kP=0.6  kI=1.2  kD=0.075
#define KP_MOVE_TO     0.6 * KU_MOVE_TO
#define KI_MOVE_TO     1.2
#define KD_MOVE_TO     0.075
#define MIN_DT_MOVE_TO 40000 // in µs, minimum value for kD to have effect

// Field Parameters
#define HALF_GOAL_SEPARATION 107.5F // in cm
#define FIELD_LENGTH         243.0F // in cm
#define FIELD_WIDTH          182.0F // in cm
// clang-format off
#define NEUTRAL_SPOT_CENTER (Point){0, 0}
#define NEUTRAL_SPOT_FL     (Point){-11.5, 45}
#define NEUTRAL_SPOT_FR     (Point){11.5, 45}
#define NEUTRAL_SPOT_BL     (Point){-11.5, -45}
#define NEUTRAL_SPOT_BR     (Point){11.5, -45}
// clang-format on

// -----------------------------------------------------------------------------

#endif