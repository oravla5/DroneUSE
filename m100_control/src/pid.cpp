#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "m100_control/pid.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double max, double min, double Kp, double Kd, double Ki, double iSaturator );
        ~PIDImpl();
        double calculate( double dt, double setpoint, double pv );

    private:
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
	double _iSaturator;
        double _pre_error;
	double _pre_Dout;
        double _integral;
};


PID::PID( double max, double min, double Kp, double Kd, double Ki, double iSaturator )
{
    pimpl = new PIDImpl(max,min,Kp,Kd,Ki,iSaturator);
}
double PID::calculate(double dt, double setpoint, double pv )
{
    return pimpl->calculate(dt, setpoint, pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( double max, double min, double Kp, double Kd, double Ki, double iSaturator ) :
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _iSaturator(iSaturator),
    _pre_error(0),
    _pre_Dout(0.0),
    _integral(0)
{
}

double PIDImpl::calculate( double dt, double setpoint, double pv )
{
    
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral = _integral + error * dt * _Ki;
    if(_integral > _iSaturator)
	_integral = _iSaturator;
    else if(_integral < -_iSaturator)
	_integral = -_iSaturator;

    double Iout = _integral;

    // Derivative term
    double derivative = (error - _pre_error) / dt; 
	// Low Pass Filter to avoid peaks
    double Dout = 0.3*derivative*_Kd + 0.7*_pre_Dout;
    _pre_Dout = Dout;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif
