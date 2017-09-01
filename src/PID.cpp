#include "PID.h"
#include <algorithm>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, double lower_bound_ , double upper_bound_) {
  // re-initalization of error terms
  this->p_error = 0;
  this->i_error = 0;
  this->d_error = 0;
  this->previous_cte = 0;

  // assignation of controller bounds
  this->lower_bound = lower_bound_;
  this->upper_bound = upper_bound_;

  // assignation of PID controller gains
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
}

void PID::UpdateError(double cte) {
  this->p_error = cte;
  this->i_error += cte;
  this->d_error = cte - this->previous_cte;
  this->previous_cte = cte;
}

double PID::TotalError() {
  double control = -(this->Kp * this->p_error) - (this->Ki * this->i_error) - (this->Kd * this->d_error);
  control = Clip(control, this->lower_bound, this->upper_bound);
  return control;
}



double PID::Clip(double n, double lower, double upper) {
  return max(lower, min(n, upper));
}