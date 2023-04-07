#ifndef TEENSY_MOVEMENT_H
#define TEENSY_MOVEMENT_H

#include <cstdint>

#include "pid.h"
#include "teensy/include/config.h"

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
    void updateHeadingController(float angle);
    void update();
    void kick();

  private:
    // Movement parameters
    int16_t _angularVelocity = 0; // ~±140 to ~±1024

    // Internal values
    bool _kickerActivated = false;
    uint32_t _kickTime = UINT32_MAX;

    // Controllers
    PIDController _headingController =
        PIDController(0,         // Target angle
                      -256, 256, // Output limits
                      KP_ROBOT_ANGLE, KI_ROBOT_ANGLE, KD_ROBOT_ANGLE, // Gains
                      MIN_DT_ROBOT_ANGLE                              // min dt
        );
};

#endif
