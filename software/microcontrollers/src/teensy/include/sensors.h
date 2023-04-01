#ifndef TEENSY_SENSORS_H
#define TEENSY_SENSORS_H

#include <cmath>
#include <cstdint>

#include "serial.h"

class Sensors {
  public:
    Sensors(SerialManager &muxSerial, SerialManager &imuSerial,
            SerialManager &tofSerial, SerialManager &coralSerial)
        : _muxSerial(muxSerial), _imuSerial(imuSerial), _tofSerial(tofSerial),
          _coralSerial(coralSerial){};

    void init();
    void read();
    void markAsRead();

  private:
    // Write-possible private variables for sensor output
    struct {
        bool newData = false;
        float angle = NAN; // -179.99º to 180.00º

        bool exists() const { return !std::isnan(angle); }
    } _robot;
    struct {
        bool newData = false;
        byte testByte;
    } _otherRobot;
    struct {
        bool newData = false;
        float angle = NAN; // -179.99º to 180.00º
        float depth = NAN; // 0.00 (inside edge) to 1.00 (outside edge)

        bool exists() const { return !std::isnan(angle) && !std::isnan(depth); }
    } _line;
    struct {
        bool newData = false;
        float front = NAN; // 0.0 to 400.0 cm
        float back = NAN;  // 0.0 to 400.0 cm
        float left = NAN;  // 0.0 to 400.0 cm
        float right = NAN; // 0.0 to 400.0 cm

        // Bounds in all four directions are established
        bool established() const {
            return !std::isnan(front) && !std::isnan(back) &&
                   !std::isnan(left) && !std::isnan(right);
        }
    } _bounds;
    struct {
        bool newData;
        float angle = NAN;    // -179.99º to 180.00º
        float distance = NAN; // 0.0 to ~400.0 cm

        bool exists() const {
            return !std::isnan(angle) && !std::isnan(distance);
        }
    } _ball;
    bool _hasBall = false; // Assume the robot does not have the ball initially

  public:
    // Read-only public interface to sensor output
    const decltype(_robot) &robot = _robot;
    const decltype(_otherRobot) &otherRobot = _otherRobot;
    const decltype(_line) &line = _line;
    const decltype(_bounds) &bounds = _bounds;
    const decltype(_ball) &ball = _ball;
    const decltype(_hasBall) &hasBall = _hasBall;

  private:
    // Serial managers to receive packets
    SerialManager &_muxSerial;
    SerialManager &_imuSerial;
    SerialManager &_tofSerial;
    SerialManager &_coralSerial;

    // Internal offsets
    int16_t _robotAngleOffset;
};

#endif
