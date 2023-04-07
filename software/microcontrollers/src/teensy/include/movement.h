#ifndef TEENSY_MOVEMENT_H
#define TEENSY_MOVEMENT_H

#include <cstdint>

#include "pid.h"
#include "sensors.h"
#include "teensy/include/config.h"
#include "vector.h"

class Movement {
  public:
    Movement();

    // Movement parameters
    bool stop = false;
    float angle = 0;      // -179.99º to +180.00º
    int16_t velocity = 0; // ±35 to ±256
    float heading = 0;    // -179.99º to +180.00º
    bool dribble = false;

    void init();
    void updateHeadingController(const float angle);
    void setMoveTo(const Point &destination, const float targetHeading,
                   const Goals &goals);
    void update();
    void kick();

  private:
    // Movement parameters
    int16_t _angularVelocity = 0; // ~±140 to ~±1024

    // Internal values
    bool _moveToActive = false;
    Point *_lastDestination = nullptr;
    float _actualHeading = 0;
    float _initialDistance = NAN;

    bool _kickerActivated = false;
    uint32_t _kickTime = 0;

    // Controllers
    PIDController _headingController =
        PIDController(0,         // Target angle
                      -256, 256, // Output limits
                      KP_ROBOT_ANGLE, KI_ROBOT_ANGLE, KD_ROBOT_ANGLE, // Gains
                      MIN_DT_ROBOT_ANGLE                              // min dt
        );
    PIDController _moveToController =
        PIDController(0,         // Target distance offset
                      -256, 256, // Output limits
                      KP_MOVE_TO, KI_MOVE_TO, KD_MOVE_TO, // Gains
                      MIN_DT_MOVE_TO                      // min dt
        );
};

#endif
