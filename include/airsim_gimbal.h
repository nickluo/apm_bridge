#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace custom_link_bridge
{
    class AirsimGimbal
    {
    public:
        using Callback = std::function<void(sensor_msgs::msg::Imu::UniquePtr)>;
        AirsimGimbal(rclcpp::Node &node, Callback callback);
        ~AirsimGimbal();
        void start();
        void stop();

    private:
        class Impl;
        std::unique_ptr<Impl> impl_;
    };
}
