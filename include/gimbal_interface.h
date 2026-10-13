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
    };

    // nullptr means intentionally disabled. Invalid configuration throws.
    std::unique_ptr<Gimbal> createGimbal(rclcpp::Node &node, Gimbal::Callback callback);
}
