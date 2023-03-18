#ifndef PID_H
#define PID_H

class PIDController {
  public:
    PIDController(const float kp, const float ki, const float kd,
                  const float min, const float max,
                  const float setpoint = 0.0F);

    // Update controller
    float advance(const float input);

    // Update parameters
    void updateSetpoint(const float setpoint);
    void updateLimits(const float min, const float max);
    void updateGains(const float kp, const float ki, const float kd);

  private:
    // Parameters
    float _kp;
    float _ki;
    float _kd;
    float _min;
    float _max;
    float _setpoint;
    // Internal values
    float _integral = 0.0;
    float _lastError = 0.0;
    float _lastTime = 0.0;
};

#endif
