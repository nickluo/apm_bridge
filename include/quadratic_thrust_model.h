#ifndef QUADRATIC_THRUST_MODEL_H
#define QUADRATIC_THRUST_MODEL_H

#include <cmath>
#include <cstdio>

namespace quadratic_thrust_model
{

    struct MotorParams
    {
        double A;
        double B;
        double C;
        double D;
        int n_motors;
        double thrust_limit;
        double spin_k;
        double volt_max;   // 满电电压 (cells*4.2): 线性补偿口径 (vbat 未启用时)
        double vbat_a;     // vbat 电压模型 y = a*V + b (thrust_test_node --vbat 台架拟合)。
        double vbat_b;     // a=0 (默认) 时未启用, 退回线性 volt_max/V 补偿。
        double volt_ref;   // 标准电压 (cells*3.7 标称): A..D 曲线与 vbat 补偿的锚点
    };

    const double EPSILON = 1e-6; // 精度阈值
    const int MAX_ITER = 100;   // 最大迭代次数，防止死循环

    // 计算函数值 f(x) = aX^3 + bX^2 + cX + d - Y
    static double f(const MotorParams &params, double x, double targetY) {
        auto x2 = x * x;
        auto x3 = x2 * x;
        return (params.A * x3) + (params.B * x2) + (params.C * x) + params.D - targetY;
    }

    // 计算导数值 f'(x) = 3aX^2 + 2bX + c
    // 导数用于确定切线斜率，指引下一次迭代的方向
    static double df(const MotorParams &params, double x) {
        return (3.0 * params.A * x * x) + (2.0 * params.B * x) + params.C;
    }

    // kp = (baro/101325.0) * (273.15/(273.15+temp));

    // vbat 电压因子 phi(V) = (a*V+b)/(a*V_ref+b):
    // 当前电压下的力 <-> 标准电压 (V_ref) 口径力 的换算。
    // 标定端 (thrust_test_node --map-a/--map-b) 与本运行时对称使用。
    static double vbatScale(const MotorParams &params, double battery_voltage)
    {
        return (params.vbat_a * battery_voltage + params.vbat_b) /
               (params.vbat_a * params.volt_ref + params.vbat_b);
    }

    double thrustToForce(const MotorParams &motor_params, double thrust, double kp, double battery_voltage = -1.0)
    {
        auto throttle = thrust / kp;
        if (std::abs(motor_params.spin_k) > EPSILON) {
            auto k_1 = motor_params.spin_k - 1.0;
            throttle = (k_1 + std::sqrt(k_1 * k_1 + 4.0 * motor_params.spin_k * throttle)) /
                            (2.0 * motor_params.spin_k);
        }

        // vbat 补偿: 系数为标准电压口径, 多项式反解后换算回当前电压的实际力;
        // 未启用时退回线性补偿 (油门侧)。
        const bool vbat = battery_voltage > 0 && motor_params.vbat_a != 0.0;
        if (battery_voltage > 0 && !vbat)
            throttle *= battery_voltage / motor_params.volt_max;    // legacy linear voltage compensation


        double force = 0.0;
        // 使用牛顿法求解方程 motor_params.A * x^3 + motor_params.B * x^2 + motor_params.C * x - thrust = 0
        // 目标是找到 x，使得上述方程成立，然后计算对应的力

        // 1. 初始猜测值
        double x = 0.0;
        // 避免初始导数为 0 的情况（导数为 0 会导致除以 0 错误）
        if (std::abs(df(motor_params, x)) < 1e-6) {
            x = 1.0; 
        }

        // 2. 开始迭代
        for (int i = 0; i < MAX_ITER; ++i) 
        {
            double y_curr = f(motor_params, x, throttle);
            // 检查是否已经足够接近目标（收敛）
            if (std::abs(y_curr) < EPSILON) {
                break;
            }

            double slope = df(motor_params, x);
            // 检查导数是否过小（防止除以零或发散）
            if (std::abs(slope) < 1e-6) {
                // 如果导数几乎为0，稍微扰动一下 x 继续尝试
                x += 0.01; 
                continue;
            }

            // 牛顿法公式: x_new = x_old - f(x) / f'(x)
            double next_x = x - (y_curr / slope);

            // 检查两次迭代之间变化是否极小（另一种收敛判断）
            if (std::abs(next_x - x) < EPSILON) {
                x = next_x;
                break;
            }
            x = next_x;
        }

        if (vbat)
            x /= vbatScale(motor_params, battery_voltage);    // 标准口径力 -> 当前电压实际力
        force = motor_params.n_motors * x;
        return force < 0.0 ? 0.0 : force;
    }

    double forceToThrust(const MotorParams &motor_params, double force, double kp, double battery_voltage = -1.0)
    {
        force /= motor_params.n_motors;

        // vbat 补偿: 请求力换算到标准电压口径后进多项式 (入口缩放, 与标定端对称);
        // 未启用时退回线性补偿 (油门出口侧)。
        const bool vbat = battery_voltage > 0 && motor_params.vbat_a != 0.0;
        if (vbat)
            force *= vbatScale(motor_params, battery_voltage);
        auto force_2 = force * force;
        auto force_3 = force_2 * force;

        auto throttle = motor_params.A * force_3 + motor_params.B * force_2 + motor_params.C * force + motor_params.D;
        if (battery_voltage > 0 && !vbat)
            throttle *= motor_params.volt_max / battery_voltage;    // legacy linear voltage compensation
        auto thrust = (1.0 - motor_params.spin_k) * throttle + motor_params.spin_k * throttle * throttle; // linear compensation
        thrust *= kp; // air pressure compensation
        if (motor_params.thrust_limit > 0.0 && thrust > motor_params.thrust_limit)
            thrust = motor_params.thrust_limit;
        return thrust; 
    }

} // namespace quadratic_thrust_model

#endif // QUADRATIC_THRUST_MODEL_H
