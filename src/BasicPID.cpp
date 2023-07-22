#include <BasicPID.h>

BasicPID::BasicPID(){}

void BasicPID::setPID(float _kp, float _ki, float _kd, float _dt) {
    _PID._KP = _kp;
    _PID._KI = _ki;
    _PID._KD = _kd;
    _PID._DT = _dt;

    _timePrev = _timeNow;  // simpan nilai waktu sebelumnya
    _timeNow = millis();  // waktu sekarang
    _deltatime = (_timeNow - _timePrev) / _PID._DT;
}

void BasicPID::updatePID(float _value, float _setpoint) {
    _timePrev = _timeNow;  // simpan nilai waktu sebelumnya
    _timeNow = millis();  // waktu sekarang
    _timeChange = (_timeNow - _timePrev);
    _deltatime = _timeChange / _PID._DT;
    if (_timeChange >= _deltatime) {
        // Hitung Nilai Error Proportional (P)
        _Errors_P = _setpoint - _value;
        _PID._Proportional = _Errors_P * _PID._KP;

        // Hitung Nilai Error Integral (I)
        _Errors_I += _Errors_P * _deltatime;
        _PID._Integrator = _Errors_I * (_PID._KI * _deltatime);

        // Hitung Nilai Error Derivative (D)
        _Errors_D = (_Errors_P - _Previous_Error);
        _PID._Derivative = _Errors_D * (_PID._KD / _deltatime);

        // Simpan Nilai Error Proportional (P) sebelumnya
        _Previous_Error = _Errors_P;
    }
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