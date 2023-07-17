#ifndef BASIC_PID_H
#define BASIC_PID_H

#include <Arduino.h>

struct PIDvariabel {
   float _KP;
   float _KI;
   float _KD;
   float _Proportional;
   float _Integrator;
   float _Derivative;
};
class BasicPID {
public:
    BasicPID(float _kp, float _ki, float _kd);
    void updatePID(float _value, float _setpoint, float _dt = 1000.0);
    float outputPID(float _min = -400.0, float _max = 400.0);
    void resetPID();

private:
    PIDvariabel PID_system;
    float Limit(float val, float min, float max);
    float _timeNow, _timePrev, _deltatime;
    float _Errors_P, _Errors_I, _Errors_D, _Previous_Error;
    int _PID;
};

#endif