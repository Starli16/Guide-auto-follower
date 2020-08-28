#include "PIDcontroller.h"

PID::PID(const float p, const float i, const float d) {
  kp = p;
  ki = i;
  kd = d;
  target = 0;
  actual = 0;
  error_pre = 0;
}

PID::~PID() {}

float PID::pid_control(float tar, float act) {
  float u;
  target = tar;
  actual = act;
  error = target - act;
  integral += error;
  u = kp * error + ki * integral + kd * (error - error_pre);
  error_pre = error;
  return u;
}