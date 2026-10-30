#pragma once

#include <functional>
#include <memory>
#include <sensor_msgs/msg/imu.hpp>
#include "gimbal_tracking.h"

namespace rclcpp { class Node; }

namespace apm_bridge
{
    class Gimbal
    {
    public:
        // Camera FLU -> world ENU orientation; rates in camera FLU.
        // Preserve legacy heading-relative Euler slots in orientation_covariance[0:3].
        // May run on a backend worker; lifecycle methods are called by the owner.
        using Callback = std::function<void(sensor_msgs::msg::Imu::UniquePtr)>;

        virtual ~Gimbal() = default;
        virtual void start() = 0;
        virtual void stop() = 0;
        // Vehicle FLU -> ENU orientation, body specific force (including gravity), m/s^2.
        virtual void updateVehicleState(const sensor_msgs::msg::Imu &imu, double gravity) = 0;
        virtual void setMode(GimbalMode mode) = 0;
        virtual void updateTargetLos(const TargetLos &target) = 0;
        // 启动回中 (机械零点) 是否已完成。默认 true: 无回中语义的后端 (捷联相机/
        // AirSim) 始终视为就绪; 串口云台在 home_on_connect 首次连接回中完成前返回
        // false, 期间驱动以 FPV+角度回零接管, 不接受模式控制律。
        virtual bool homed() const { return true; }
    };

    // nullptr means intentionally disabled. Invalid configuration throws.
    std::unique_ptr<Gimbal> createGimbal(rclcpp::Node &node, Gimbal::Callback callback);
}
