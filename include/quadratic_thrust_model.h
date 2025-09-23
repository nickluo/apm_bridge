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
  double voltage_map_a;
  double voltage_map_b;
};

void convert_from_abc(MotorParams &params, double a, double b, double c)
{
  if (std::abs(a) < 1e-6)
  {
    params.A = 0.0;
    params.B = b;
    params.C = c;
    return;
  }
  // A=sign(a1)*sqrt(1/a1), B=-a2/(2*a1), C=a3-a1*B^2
  params.A = std::copysign(1.0, a) * std::sqrt(1.0 / a);
  params.B = -b / (2 * a);
  params.C = c - a * params.B * params.B;
}

// kp = (baro/101325.0) * (273.15/(273.15+temp));

double thrustToForce(const MotorParams &motor_params, double thrust, double kp, double battery_voltage = -1.0)
{
  double force = 0.0;
  if (std::abs(motor_params.A) < 1e-6)
    force = motor_params.n_motors * kp * (motor_params.B * thrust + motor_params.C);
  else
    force = motor_params.n_motors * kp * (std::pow((thrust - motor_params.B), 2)/(motor_params.A * std::abs(motor_params.A)) + motor_params.C);
  if (battery_voltage > 0) {
    const double ratio =
            motor_params.voltage_map_a * battery_voltage +
            motor_params.voltage_map_b;
    force /= ratio;
  }
  return force< 0.0 ? 0.0 : force;
}

double forceToThrust(const MotorParams &motor_params, double force, double kp, double battery_voltage = -1.0) 
{
  if (battery_voltage > 0) {
    const double ratio =
            motor_params.voltage_map_a * battery_voltage +
            motor_params.voltage_map_b;
    force *= ratio;
  }
  if (std::abs(motor_params.A) < 1e-6)
  {
    auto thrust = (force / (motor_params.n_motors*kp) - motor_params.C) / motor_params.B;
    return thrust>0.0 ? thrust : 0.0;
  }
  else
    return std::sqrt(std::abs(motor_params.C - force / (motor_params.n_motors*kp))) * motor_params.A + motor_params.B;
}

}  // namespace quadratic_thrust_model

#endif  // QUADRATIC_THRUST_MODEL_H
