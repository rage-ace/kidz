#ifndef PID_H
#define PID_H

class PIDController {
  public:
    PIDController(float kp, float ki, float kd, float min, float max,
                  float setpoint = 0.0F);

    // Update controller
    float advance(float input);

    // Update parameters
    void updateSetpoint(float setpoint);
    void updateLimits(float min, float max);
    void updateGains(float kp, float ki, float kd);

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