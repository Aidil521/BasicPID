#include <BasicPID.h>

BasicPID::BasicPID(){}

float Limit(float val, float limit) {
    if (val < -limit)val = -limit;
    if (val > limit)val = limit;
    return val;
}

void BasicPID::setConfig(float _kp, float _ki, float _kd, uint16_t _dt) {
    _PID._KP = _kp;
    _PID._KI = _ki;
    _PID._KD = _kd;
    _PID._DT = _dt;
}

void BasicPID::update(float _inertial, float _setpoint) {
    _timeNow = millis();  // waktu sekarang
    _timeChange = (_timeNow - _timePrev);

    if (_timeChange > _PID._DT) {
        // Reset Value Error I
        _Errors_I  = 0;
        _PID._Integrator = 0;
    }
    _timePrev = _timeNow;  // simpan nilai waktu sebelumnya
    _deltatime = _timeChange / _PID._DT;

    // Hitung Nilai Error Proportional (P)
    _Errors_P = _setpoint - _inertial;
    _PID._Proportional = _Errors_P * _PID._KP;

    // Hitung Nilai Error Integral (I)
    _Errors_I += _Errors_P * _deltatime;
    _PID._Integrator = Limit((_Errors_I * _PID._KI), 400.0f);

    // Hitung Nilai Error Derivative (D)
    _Errors_D = (_Errors_P - _Previous_Error) / _deltatime;
    _PID._Derivative = _Errors_D * _PID._KD;

    // Simpan Nilai Error Proportional (P) sebelumnya
    _Previous_Error = _Errors_P;
}

float BasicPID::output(float _limit) {
    // Jumlahkan Nilai P, I dan D
    _PID._Output = _PID._Proportional + _PID._Integrator + _PID._Derivative;
    // Limitasi output PID rentang +-400
    return Limit(_PID._Output, _limit);
}

void BasicPID::reset() {
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