#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
class PID{
private:
    float kp;
    float ki;
    float kd;
    float target;
    float actual;
    float error;
    float error_pre;
    float integral;
public:
     PID(const float p,const float i,const float d);
    ~ PID();
    float pid_control(float tar,float act);
};
#endif //PID_CONTROLLER_H
