#pragma once

class PID {
  float kp;
  float ki;
  float kd;
  float acc_err;
  float err[3];
  float output;

public:
  PID(const float &kp, const float &ki, const float &kd) : kp(kp), ki(ki), kd(kd){}
  float run(const float now, const float target);
};

float PID::run(const float input, const float target) {
  err[0] = target - input;
  acc_err += err[0];
  output = kp * err[0] + ki * acc_err + kd * (err[0] - err[1]);
  err[2] = err[1];
  err[1] = err[0];
  return output;
}
