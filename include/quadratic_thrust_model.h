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

// kp = (baro/101325.0) * (273.15/(273.15+temp));

double thrustToForce(const MotorParams &motor_params, double thrust, double kp, double battery_voltage = -1.0)
{
  double force = motor_params.n_motors * kp * (std::pow((thrust - motor_params.B), 2)/(motor_params.A * std::abs(motor_params.A)) + motor_params.C);
  if (battery_voltage > 0) {
    const double ratio =
            motor_params.voltage_map_a * battery_voltage +
            motor_params.voltage_map_b;
    force /= ratio;
  }
  return force;
}

double forceToThrust(const MotorParams &motor_params, double force, double kp, double battery_voltage = -1.0) 
{
  if (battery_voltage > 0) {
    const double ratio =
            motor_params.voltage_map_a * battery_voltage +
            motor_params.voltage_map_b;
    force *= ratio;
  }
  return std::sqrt(motor_params.C - force / (motor_params.n_motors*kp)) * motor_params.A + motor_params.B;
}

}  // namespace quadratic_thrust_model

#endif  // QUADRATIC_THRUST_MODEL_H
