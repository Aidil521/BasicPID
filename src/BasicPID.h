#ifndef BASIC_PID_H
#define BASIC_PID_H

#include <Arduino.h>

class BasicPID {
public:
    BasicPID(float _kp, float _ki, float _kd);
    void updatePID(int _value, int _setpoint, float _dt = 1000.0);
    int outputPID(int minValue = (-400), int maxValue = 400);
    void resetPID();

private:
    float _KP, _KI, _KD, _DT;
    float _time, _timePrev, _deltatime;
    float _Errors_P, _Errors_I, _Errors_D, _Previous_Error;
    int _output;
};

#endif