#ifndef QUADRATIC_THRUST_MODEL_H
#define QUADRATIC_THRUST_MODEL_H

#include <cmath>

namespace quadratic_thrust_model
{

struct MotorParams
{
  double A;
  double B;
  double C;
  int    n_motors;
};

// kp = (baro/101325.0) * (273.15/(273.15+temp));

double inline thrustToForce(const MotorParams &motor_params, double thrust, double kp)
{
  return motor_params.n_motors * kp * (std::pow((thrust - motor_params.B), 2)/(motor_params.A * std::abs(motor_params.A)) + motor_params.C);
}

double inline forceToThrust(const MotorParams &motor_params, double force, double kp) 
{
  return std::sqrt(motor_params.C - force / (motor_params.n_motors*kp)) * motor_params.A + motor_params.B;
}

}  // namespace quadratic_thrust_model

#endif  // QUADRATIC_THRUST_MODEL_H
