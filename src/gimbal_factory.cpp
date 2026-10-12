#include "gimbal_interface.h"

#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include "airsim_gimbal.h"
#include "gimbal.h"
#include "strapdown_gimbal.h"

namespace apm_bridge
{
    std::unique_ptr<Gimbal> createGimbal(rclcpp::Node &node, Gimbal::Callback callback)
    {
        if (node.get_node_parameters_interface()->get_parameter_overrides().count("gimbal.fixed_pitch"))
            throw std::invalid_argument("gimbal.fixed_pitch was merged into gimbal.pitch_hold; update the camera parameters");
        const auto port = node.declare_parameter<std::string>("gimbal_port", "");
        if (node.get_parameter("airsim.enabled").as_bool())
        {
            if (!port.empty())
                throw std::invalid_argument("AirSim SITL requires an empty gimbal_port");
            if (!node.declare_parameter<bool>("airsim.gimbal_enabled", true))
                return nullptr;
            return std::make_unique<custom_link_bridge::AirsimGimbal>(node, std::move(callback));
        }
        if (port.empty())
            return std::make_unique<StrapdownGimbal>(node, std::move(callback));
        const auto config = xfrobot::GimbalConfig::fromParameters(node);
        return std::make_unique<xfrobot::GimbalControl>(node, port, config, std::move(callback));
    }
}
