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
    float angle = 0;      // -179.99º to +180.00º
    int16_t velocity = 0; // ±35 to ±1023
    float heading = 0;    // -179.99º to +180.00º
    bool dribble = false;

    void init();
    // Read input
    void updateHeadingController(const float angle);
    // Set parameters in the body of the loop
    void setStop(bool maintainHeading = true);
    void setMoveTo(const Vector &robot, const Point &destination,
                   const float targetHeading);
    void setLineTrack(const float lineDepth, const float targetLineAngle,
                      const float targetLineDepth, const bool trackRightSide);
    void setMoveOnLineToBall(const float lineDepth, const Vector &ball,
                             const float targetLineDepth,
                             const bool trackRightSide);
    void setLinearDecelerate(const int16_t startSpeed, int16_t endSpeed,
                             const float multiplier,
                             const bool replaceVelocity = false);
    // Call for updates at the end of the loop
    void update();
    // Call for immediate updates anywhere
    void kick();

    // Controllers
    PIDController headingController = PIDController(
        0,                                              // Target angle
        -1023, 1023,                                    // Output limits
        KP_ROBOT_ANGLE, KI_ROBOT_ANGLE, KD_ROBOT_ANGLE, // Gains
        MIN_DT_ROBOT_ANGLE, MAXI_ROBOT_ANGLE, MAX_SETPOINT_CHANGE_ROBOT_ANGLE);
    PIDController moveToController =
        PIDController(0,           // Target distance offset
                      -1023, 1023, // Output limits
                      KP_MOVE_TO, KI_MOVE_TO, KD_MOVE_TO, // Gains
                      MIN_DT_MOVE_TO);
    PIDController lineTrackController =
        PIDController(0,       // Target angle offset
                      -90, 90, // Output limits
                      KP_LINE_TRACK, KI_LINE_TRACK, KD_LINE_TRACK, // Gains
                      MIN_DT_LINE_TRACK, MAXI_LINE_TRACK);

    PIDController moveOnLineToBallController =
        PIDController(MOVE_ON_LINE_TO_BALL_DISTANCE_OFFSET, // Target
                      -MOVE_ON_LINE_TO_BALL_MAX_SPEED,
                      MOVE_ON_LINE_TO_BALL_MAX_SPEED, // Output limits
                      KP_MOVE_ON_LINE_TO_BALL, KI_MOVE_ON_LINE_TO_BALL,
                      KD_MOVE_ON_LINE_TO_BALL, // Gains
                      MIN_DT_MOVE_ON_LINE_TO_BALL, MAXI_MOVE_ON_LINE_TO_BALL,
                      MAX_SETPOINT_CHANGE_MOVE_ON_LINE_TO_BALL);

  private:
    // Movement parameters
    bool _brake = false;

    // Internal values
    float _actualHeading = 0;
    // for setMoveTo()
    bool _moveToActive = false;
    Point *_lastDestination = nullptr; // checks if different destination
    float _initialDistance = NAN;
    // for setLineTrack()
    bool _lineTrackActive = false;
    float _lastTargetLineAngle = NAN; // checks if different line
    // for setMoveOnLineToBall()
    bool _moveOnLineToBallActive = false;
    bool *_lastTrackBallRightSide = nullptr; // checks if different line

    bool _kickerActivated = false;
    uint32_t _kickTime = 0;
};

#endif
