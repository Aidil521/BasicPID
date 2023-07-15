#include <BasicPID.h>

BasicPID::BasicPID(float _kp, float _ki, float _kd) {
    _KP = _kp;
    _KI = _ki;
    _KD = _kd;
}

void BasicPID::updatePID(int _value, int _setpoint, float _dt) {
    _timePrev = _time;  // simpan nilai waktu sebelumnya
    _time = millis();  // waktu sekarang
    _deltatime = (_time - _timePrev) / _dt;

    // Hitung Nilai Error Proportional (P)
    _Errors_P  = _value - _setpoint;

    // Hitung Nilai Error Integral (I)
    _Errors_I  += _Errors_P  * _deltatime;

    // Hitung Nilai Error Derivative (D)
    _Errors_D  = (_Errors_P  - _Previous_Error) / _deltatime;

    // Simpan Nilai Error Proportional (P) sebelumnya
    _Previous_Error = _Errors_P;
}

int BasicPID::outputPID(int minValue, int maxValue) {
    // Jumlahkan Nilai P, I dan D
    _output = (_Errors_P * _KP) + (_Errors_I * _KI) + (_Errors_D * _KD);
    // Limitasi output PID rentang +-400
    if (_output > maxValue) {
        _output = maxValue;
    }
    else if (_output < minValue) {
        _output = minValue;
    }
    return _output;
}

void BasicPID::resetPID() {
    // Reset Nilai output PID
    // _output = 0;
    
    // Reset Nilai Error P
    _Errors_P  = 0;

    // Reset Nilai Error I
    _Errors_I  = 0;

    // Reset Nilai Error D
    _Errors_D  = 0;
    // Reset Nilai Previous Error
    _Previous_Error = 0;
}