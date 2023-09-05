#ifndef BASIC_PID_H
#define BASIC_PID_H

#include <Arduino.h>

struct PIDvariabel {
   float _KP;
   float _KI;
   float _KD;
   float _DT;
   float _Proportional;
   float _Integrator;
   float _Derivative;
   int _Output;
};
class BasicPID {
public:
    BasicPID();
    void setConfig(float _kp, float _ki, float _kd, uint16_t _dt = 1000.0f);
    void update(float _value, float _setpoint);
    float output(float _min = -400.0f, float _max = 400.0f);
    void reset();

private:
    PIDvariabel _PID;
    float _Errors_P, _Errors_I, _Errors_D, _Previous_Error, _deltatime;;
    uint32_t _timeNow, _timePrev, _timeChange;
};

#endif