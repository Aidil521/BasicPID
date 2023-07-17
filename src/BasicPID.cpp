#include <BasicPID.h>

BasicPID::BasicPID(float _kp, float _ki, float _kd) {
    PID_system._KP = _kp;
    PID_system._KI = _ki;
    PID_system._KD = _kd;
}

void BasicPID::updatePID(float _value, float _setpoint, float _dt) {
    _timePrev = _timeNow;  // simpan nilai waktu sebelumnya
    _timeNow = millis();  // waktu sekarang
    _deltatime = (_timeNow - _timePrev) / _dt;

    // Hitung Nilai Error Proportional (P)
    _Errors_P = _value - _setpoint;
    PID_system._Proportional = _Errors_P * PID_system._KP;

    // Hitung Nilai Error Integral (I)
    _Errors_I += _Errors_P * _deltatime;
    PID_system._Integrator = _Errors_I * PID_system._KI;

    // Hitung Nilai Error Derivative (D)
    _Errors_D = (_Errors_P - _Previous_Error) / _deltatime;
    PID_system._Derivative = _Errors_D * PID_system._KD;

    // Simpan Nilai Error Proportional (P) sebelumnya
    _Previous_Error = _Errors_P;
}

float BasicPID::outputPID(float _min, float _max) {
    // Jumlahkan Nilai P, I dan D
    _PID = PID_system._Proportional + PID_system._Integrator + PID_system._Derivative;
    // Limitasi output PID rentang +-400
    return Limit(_PID, _min, _max);
}

void BasicPID::resetPID() {
    // Reset Nilai output PID
    _PID = 0;
    
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