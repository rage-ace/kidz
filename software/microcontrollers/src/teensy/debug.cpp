#include <Arduino.h>

#include "counter.h"
#include "teensy/include/main.h"

// Counters
auto debugPrintCounter = Counter();

void performSetupDebug() {}

void performLoopDebug() {
#ifdef DEBUG_MUX
    // Redirect MUX Serial to monitor
    while (MUX_SERIAL.available() > 0) Serial.write(MUX_SERIAL.read());
#endif
#ifdef DEBUG_TOF
    // Redirect TOF Serial to monitor
    while (TOF_SERIAL.available() > 0) Serial.write(TOF_SERIAL.read());
#endif
#ifdef DEBUG_IMU
    // Redirect IMU Serial to monitor
    while (IMU_SERIAL.available() > 0) Serial.write(IMU_SERIAL.read());
#endif
#ifdef DEBUG_CORAL
    // Redirect Coral Serial to monitor
    while (CORAL_SERIAL.available() > 0) Serial.write(CORAL_SERIAL.read());
#endif

#ifdef DEBUG_TEENSY
    // Test motors
    // movement.dribble = true;
    // movement.angle = 0;
    // movement.velocity = 300;
    // movement.heading = 0;
    // movement.updateHeadingController(0);
    // movement.update();
    // analogWrite(PIN_MOTOR_FL_PWM, 250);
    // analogWrite(PIN_MOTOR_FR_PWM, 250);
    // analogWrite(PIN_MOTOR_BL_PWM, 250);
    // analogWrite(PIN_MOTOR_BR_PWM, 250);
    // while (1) {}
    // for (auto i = 0; i < 4; ++i) {
    //     analogWrite(PIN_MOTOR_FL_PWM, i == 0 ? 250 : 0);
    //     analogWrite(PIN_MOTOR_FR_PWM, i == 1 ? 250 : 0);
    //     analogWrite(PIN_MOTOR_BL_PWM, i == 2 ? 250 : 0);
    //     analogWrite(PIN_MOTOR_BR_PWM, i == 3 ? 250 : 0);
    //     delay(4000);
    // }

    // // Line Track
    // TODO

    if (debugPrintCounter.millisElapsed(100)) {
        // Print debug data
        Serial.printf("Line ");
        if (sensors.line.exists())
            Serial.printf("%4d.%02dº ", (int)sensors.line.angleBisector,
                          abs(sensors.line.angleBisector * 100) % 100);
        else
            Serial.printf("         ");
        Serial.printf("%01d.%02d | ", (int)sensors.line.depth,
                      abs((int)(sensors.line.depth * 100) % 100));
        if (sensors.robot.angle.established())
            Serial.printf("Robot Angle %4d.%02dº | ",
                          (int)sensors.robot.angle.value,
                          abs(sensors.robot.angle.value * 100) % 100);
        else
            Serial.printf("Robot Angle          | ");
        if (sensors.robot.position.exists()) {
            Serial.printf(
                "Position %4d.%02dº %3d.%02d cm | ",
                (int)sensors.robot.position.value.angle,
                abs((int)(sensors.robot.position.value.angle * 100) % 100),
                (int)sensors.robot.position.value.distance,
                abs((int)(sensors.robot.position.value.distance * 100) % 100));
        } else {
            Serial.printf("Position                    | ");
        }
        // Serial.print("Bounds ");
        // if (sensors.bounds.front.valid())
        //     Serial.printf("F: %3d.%1d cm ", (int)sensors.bounds.front.value,
        //                   abs((int)(sensors.bounds.front.value * 10) % 10));
        // else
        //     Serial.printf("F:          ");
        // if (sensors.bounds.back.valid())
        //     Serial.printf("B: %3d.%1d cm ", (int)sensors.bounds.back.value,
        //                   abs((int)(sensors.bounds.back.value * 10) % 10));
        // else
        //     Serial.printf("B:          ");
        // if (sensors.bounds.left.valid())
        //     Serial.printf("L: %3d.%1d cm ", (int)sensors.bounds.left.value,
        //                   abs((int)(sensors.bounds.left.value * 10) % 10));
        // else
        //     Serial.printf("L:          ");
        // if (sensors.bounds.right.valid())
        //     Serial.printf("R: %3d.%1d cm | ",
        //     (int)sensors.bounds.right.value,
        //                   abs((int)(sensors.bounds.right.value * 10) % 10));
        // else
        //     Serial.printf("R:          | ");
        if (sensors.ball.value.exists())
            Serial.printf("Ball %4d.%02dº %4d.%02d cm | ",
                          (int)sensors.ball.value.angle,
                          abs((int)(sensors.ball.value.angle * 100) % 100),
                          (int)sensors.ball.value.distance,
                          abs((int)(sensors.ball.value.distance * 100) % 100));
        else
            Serial.printf("Ball                     | ");
        Serial.printf("Has Ball: %d | ", sensors.hasBall);
        // Serial.printf("Goal O ");
        // if (sensors.goals.offensive.exists())
        //     Serial.printf(
        //         "%4d.%02dº %4d.%02d cm D ",
        //         (int)sensors.goals.offensive.angle,
        //         abs((sensors.goals.offensive.angle * 100) % 100),
        //         (int)sensors.goals.offensive.distance,
        //         abs((sensors.goals.offensive.distance * 100) % 100));
        // else
        //     Serial.printf("                    D ");
        // if (sensors.goals.defensive.exists())
        //     Serial.printf(
        //         "%4d.%02dº %4d.%02d cm | ",
        //         (int)sensors.goals.defensive.angle,
        //         abs((sensors.goals.defensive.angle * 100) % 100),
        //         (int)sensors.goals.defensive.distance,
        //         abs((sensors.goals.defensive.distance * 100) % 100));
        // else
        //     Serial.printf("                    | ");
        Serial.printf(
            "Drive %4d.%02dº at %4d facing %4d.%02dº | ", (int)movement.angle,
            abs((int)(movement.angle * 100) % 100), movement.velocity,
            (int)movement.heading, abs((int)(movement.heading * 100) % 100));
        Serial.println();
    }

    // // Print loop time
    // printLoopTime();

    // // Figure out lightgate threshold
    // Serial.println(analogRead(PIN_LIGHTGATE));
#endif
}
