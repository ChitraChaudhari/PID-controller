#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  prev_cte = 0.0;
}

void PID::UpdateError(double cte) {
  //Proportional error
  p_error = cte;
  
  //Integral error
  i_error += cte;
  
  //diffrential error
  d_error = cte - prev_cte;
  prev_cte = cte;  
}

double PID::TotalError() {
  return -p_error*Kp - i_error*Ki - d_error*Kd;  //total error calculation
}