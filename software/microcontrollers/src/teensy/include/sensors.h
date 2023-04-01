#ifndef TEENSY_SENSORS_H
#define TEENSY_SENSORS_H

#include <PacketSerial.h>
#include <cmath>
#include <cstdint>

class Sensors {
  public:
    Sensors(PacketSerial &muxSerial, PacketSerial &tofSerial,
            PacketSerial &imuSerial, PacketSerial &coralSerial)
        : _muxSerial(muxSerial), _tofSerial(tofSerial), _imuSerial(imuSerial),
          _coralSerial(coralSerial){};

    void init();
    void waitForSubprocessorInit();

    void onMuxPacket(const byte *buffer, size_t size);
    void onTofPacket(const byte *buffer, size_t size);
    void onImuPacket(const byte *buffer, size_t size);
    void onCoralPacket(const byte *buffer, size_t size);

    void read();
    void markAsRead();

  private:
    // Write-possible private variables for sensor output
    struct {
        bool newData = false;
        float angle = NAN; // -179.99º to 180.00º

        bool established() const { return !std::isnan(angle); }
    } _robot;
    struct {
        bool newData = false;
        byte testByte;
    } _otherRobot;
    struct {
        bool newData = false;
        float angleBisector = NAN; // -179.99º to 180.00º
        float depth = 0;           // 0.00 (inside edge) to 1.00 (outside edge)

        bool exists() const { return !std::isnan(angleBisector); }
    } _line;
    struct {
        struct {
            bool newData = false;
            float value = NAN; // 0.0 to 400.0 cm

            bool established() const { return !std::isnan(value); }
        } front, back, left, right;

        // Bounds in all four directions are established
        bool established() const {
            return !std::isnan(front.value) && !std::isnan(back.value) &&
                   !std::isnan(left.value) && !std::isnan(right.value);
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
    PacketSerial &_muxSerial;
    PacketSerial &_tofSerial;
    PacketSerial &_imuSerial;
    PacketSerial &_coralSerial;

    // Init flags
    bool _muxInit = false;
    bool _tofInit = false;
    bool _imuInit = false;
    bool _coralInit = false;

    // Internal state
    int16_t _robotAngleOffset;
    bool _wasOnLine = false;
    float _lastLineAngleBisector = NAN;
};

#endif
