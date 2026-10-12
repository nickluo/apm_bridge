#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "gimbal_interface.h"

namespace custom_link_bridge
{
    class AirsimGimbal : public apm_bridge::Gimbal
    {
    public:
        AirsimGimbal(rclcpp::Node &node, Callback callback);
        ~AirsimGimbal() override;
        void start() override;
        void stop() override;
        void updateVehicleState(const sensor_msgs::msg::Imu &imu, double gravity) override;

    private:
        class Impl;
        std::unique_ptr<Impl> impl_;
    };
}
