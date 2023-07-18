#include <BasicPID.h>

BasicPID::BasicPID(float _kp, float _ki, float _kd) {
    _PID._KP = _kp;
    _PID._KI = _ki;
    _PID._KD = _kd;
}

void BasicPID::updatePID(float _value, float _setpoint, float _dt) {
    _timePrev = _timeNow;  // simpan nilai waktu sebelumnya
    _timeNow = millis();  // waktu sekarang
    _deltatime = (_timeNow - _timePrev) / _dt;

    // Hitung Nilai Error Proportional (P)
    _Errors_P = _value - _setpoint;
    _PID._Proportional = _Errors_P * _PID._KP;

    // Hitung Nilai Error Integral (I)
    _Errors_I += _Errors_P * _deltatime;
    _PID._Integrator = _Errors_I * _PID._KI;

    // Hitung Nilai Error Derivative (D)
    _Errors_D = (_Errors_P - _Previous_Error) / _deltatime;
    _PID._Derivative = _Errors_D * _PID._KD;

    // Simpan Nilai Error Proportional (P) sebelumnya
    _Previous_Error = _Errors_P;
}

float BasicPID::outputPID(float _min, float _max) {
    // Jumlahkan Nilai P, I dan D
    _PID._Output = _PID._Proportional + _PID._Integrator + _PID._Derivative;
    // Limitasi output PID rentang +-400
    return Limit(_PID._Output, _min, _max);
}

void BasicPID::resetPID() {
    // Reset Nilai output PID
    _PID._Output = 0;
    
    // Reset Nilai Error P
    _Errors_P  = 0;

    // Reset Nilai Error I
    _Errors_I  = 0;

    // Reset Nilai Error D
    _Errors_D  = 0;

    // Reset Nilai Previous Error
    _Previous_Error = 0;
}

float BasicPID::Limit(float val, float min, float max) {
    if (val > max)val = max;
    if (val < min)val = min;
    return val;
}