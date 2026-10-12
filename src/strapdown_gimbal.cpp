#include "strapdown_gimbal.h"

#include <cmath>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace apm_bridge
{
    StrapdownGimbal::StrapdownGimbal(rclcpp::Node &node, Callback callback)
        : node_(node), callback_(std::move(callback))
    {
        if (!callback_)
            throw std::invalid_argument("Gimbal orientation callback is required");
        const double pitch = node_.declare_parameter<double>("gimbal.pitch_hold", 0.0);
        if (!std::isfinite(pitch) || pitch < -180.0 || pitch > 180.0)
            throw std::invalid_argument("gimbal.pitch_hold must be finite and within [-180,180] degrees for a strapdown camera");
        camera_to_body_.setRotation(tf2::Vector3(0, 1, 0), pitch * M_PI / 180.0);
        RCLCPP_INFO(node_.get_logger(),
                    "Strapdown camera: fixed pitch %.2f deg FLU (positive nose-down, negative nose-up)", pitch);
    }

    void StrapdownGimbal::start()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = true;
    }

    void StrapdownGimbal::stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
    }

    void StrapdownGimbal::updateVehicleState(const sensor_msgs::msg::Imu &imu, double)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_)
            return;
        tf2::Quaternion body_to_world;
        tf2::fromMsg(imu.orientation, body_to_world);
        if (!std::isfinite(body_to_world.length2()) || body_to_world.length2() < 1e-12)
        {
            RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                 "Ignoring invalid vehicle orientation for strapdown camera");
            return;
        }
        body_to_world.normalize();
        const auto camera_to_world = body_to_world * camera_to_body_;
        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        msg->header = imu.header;
        msg->header.frame_id = "base_link";
        msg->orientation = tf2::toMsg(camera_to_world);
        msg->linear_acceleration_covariance[0] = -1.0; // No camera accelerometer/lever-arm model.
        double roll, pitch, yaw;
        tf2::Matrix3x3(body_to_world).getRPY(roll, pitch, yaw);
        const tf2::Quaternion heading(tf2::Vector3(0, 0, 1), yaw);
        tf2::Matrix3x3(heading.inverse() * camera_to_world).getRPY(
            msg->orientation_covariance[0], msg->orientation_covariance[1], msg->orientation_covariance[2]);

        const auto &omega = imu.angular_velocity;
        if (imu.angular_velocity_covariance[0] < 0.0)
        {
            msg->angular_velocity_covariance[0] = -1.0;
        }
        else if (!std::isfinite(omega.x) || !std::isfinite(omega.y) || !std::isfinite(omega.z))
        {
            RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                 "Strapdown camera angular velocity unavailable: invalid vehicle gyro");
            msg->angular_velocity_covariance[0] = -1.0;
        }
        else
        {
            const tf2::Matrix3x3 body_to_camera(camera_to_body_.inverse());
            const auto camera_omega = body_to_camera * tf2::Vector3(omega.x, omega.y, omega.z);
            msg->angular_velocity.x = camera_omega.x();
            msg->angular_velocity.y = camera_omega.y();
            msg->angular_velocity.z = camera_omega.z();
            bool valid_covariance = true;
            for (double value : imu.angular_velocity_covariance)
                valid_covariance &= std::isfinite(value);
            valid_covariance &= imu.angular_velocity_covariance[0] >= 0.0 &&
                imu.angular_velocity_covariance[4] >= 0.0 && imu.angular_velocity_covariance[8] >= 0.0;
            if (!valid_covariance)
            {
                RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                     "Strapdown camera angular covariance unavailable: invalid vehicle covariance");
                msg->angular_velocity_covariance[0] = -1.0;
            }
            else
            {
                for (int row = 0; row < 3; ++row)
                    for (int col = 0; col < 3; ++col)
                        for (int i = 0; i < 3; ++i)
                            for (int j = 0; j < 3; ++j)
                                msg->angular_velocity_covariance[row * 3 + col] += body_to_camera[row][i] *
                                    imu.angular_velocity_covariance[i * 3 + j] * body_to_camera[col][j];
            }
        }
        callback_(std::move(msg));
    }
}
