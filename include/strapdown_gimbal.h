#pragma once

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include "gimbal_interface.h"

namespace apm_bridge
{
    class StrapdownGimbal final : public Gimbal
    {
    public:
        StrapdownGimbal(rclcpp::Node &node, Callback callback);
        ~StrapdownGimbal() override { stop(); }
        void start() override;
        void stop() override;
        void updateVehicleState(const sensor_msgs::msg::Imu &imu, double gravity) override;

    private:
        rclcpp::Node &node_;
        Callback callback_;
        tf2::Quaternion camera_to_body_;
        std::mutex mutex_;
        bool running_ = false;
    };
}
