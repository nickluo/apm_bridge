#include "airsim_gimbal.h"

#include <stdexcept>

#ifdef APM_BRIDGE_AIRSIM
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <rpc/client.h>
#include <rpc/rpc_error.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace custom_link_bridge
{
    class AirsimGimbal::Impl
    {
    public:
        Impl(rclcpp::Node &node, Callback callback) : node_(node), callback_(std::move(callback))
        {
            if (!callback_)
                throw std::invalid_argument("Gimbal orientation callback is required");
            host_ = node_.declare_parameter<std::string>("airsim.host", "127.0.0.1");
            port_ = node_.declare_parameter<int>("airsim.port", 41451);
            vehicle_ = node_.declare_parameter<std::string>("airsim.vehicle_name", "Copter");
            camera_ = node_.declare_parameter<std::string>("airsim.camera_name", "0");
            timeout_ = node_.declare_parameter<double>("airsim.timeout", 1.0);
            if (port_ <= 0 || port_ > 65535 || !std::isfinite(timeout_) || timeout_ <= 0)
                throw std::invalid_argument("Invalid AirSim gimbal port or timeout");
        }

        ~Impl() { stop(); }
        void start()
        {
            if (running_.exchange(true))
                return;
            thread_ = std::thread([this]() { run(); });
        }
        void stop()
        {
            running_ = false;
            wake_.notify_all();
            if (thread_.joinable())
                thread_.join();
        }
        void updateVehicleState(const sensor_msgs::msg::Imu &imu)
        {
            tf2::Quaternion q;
            tf2::fromMsg(imu.orientation, q);
            if (!std::isfinite(q.length2()) || q.length2() < 1e-12)
            {
                RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                     "Ignoring invalid vehicle orientation for AirSim gimbal");
                return;
            }
            q.normalize();
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            vehicle_heading_ = yaw;
        }

    private:
        void run()
        {
            rpc::client client(host_, static_cast<uint16_t>(port_));
            client.set_timeout(static_cast<int64_t>(timeout_ * 1000));
            std::optional<tf2::Quaternion> previous;
            auto last_time = std::chrono::steady_clock::now();
            while (running_)
            {
                try
                {
                    const auto response = client.call("simGetCameraInfo", camera_, vehicle_);
                    using ObjectMap = std::map<std::string, RPCLIB_MSGPACK::object>;
                    const auto info = response.get().as<ObjectMap>();
                    const auto pose = info.at("pose").as<ObjectMap>();
                    const auto q = pose.at("orientation").as<ObjectMap>();
                    tf2::Quaternion ned(q.at("x_val").as<double>(), q.at("y_val").as<double>(),
                                        q.at("z_val").as<double>(), q.at("w_val").as<double>());
                    if (!std::isfinite(ned.length2()) || ned.length2() < 1e-12)
                        throw std::runtime_error("Invalid AirSim camera quaternion");
                    const double h = std::sqrt(0.5);
                    const tf2::Quaternion enu = tf2::Quaternion(h, h, 0, 0) *
                        ned.normalized() * tf2::Quaternion(1, 0, 0, 0);
                    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
                    msg->header.stamp = node_.now();
                    msg->header.frame_id = "base_link";
                    msg->orientation = tf2::toMsg(enu);
                    const tf2::Quaternion heading(tf2::Vector3(0, 0, 1), vehicle_heading_.load());
                    tf2::Matrix3x3(heading.inverse() * enu).getRPY(msg->orientation_covariance[0],
                        msg->orientation_covariance[1], msg->orientation_covariance[2]);
                    msg->angular_velocity_covariance[0] = -1;
                    msg->linear_acceleration_covariance[0] = -1;
                    const auto now = std::chrono::steady_clock::now();
                    if (previous)
                    {
                        auto delta = previous->inverse() * enu;
                        if (delta.w() < 0)
                            delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
                        const double dt = std::chrono::duration<double>(now - last_time).count();
                        if (dt > 1e-6)
                        {
                            const auto omega = delta.getAxis() * (delta.getAngle() / dt);
                            msg->angular_velocity.x = omega.x();
                            msg->angular_velocity.y = omega.y();
                            msg->angular_velocity.z = omega.z();
                            msg->angular_velocity_covariance[0] = 0;
                        }
                    }
                    previous = enu;
                    last_time = now;
                    callback_(std::move(msg));
                }
                catch (const std::runtime_error &e)
                {
                    previous.reset();
                    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                         "AirSim gimbal API: %s", e.what());
                }
                catch (const std::out_of_range &e)
                {
                    previous.reset();
                    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                         "Invalid AirSim camera response: %s", e.what());
                }
                catch (const RPCLIB_MSGPACK::type_error &e)
                {
                    previous.reset();
                    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                         "Invalid AirSim camera response type: %s", e.what());
                }
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait_for(lock, std::chrono::milliseconds(20), [this]() { return !running_; });
            }
        }
        rclcpp::Node &node_;
        Callback callback_;
        std::string host_, vehicle_, camera_;
        int port_;
        double timeout_;
        std::atomic_bool running_{false};
        std::atomic<double> vehicle_heading_{0.0};
        std::thread thread_;
        std::mutex mutex_;
        std::condition_variable wake_;
    };

    AirsimGimbal::AirsimGimbal(rclcpp::Node &node, Callback callback)
        : impl_(std::make_unique<Impl>(node, std::move(callback))) {}
    AirsimGimbal::~AirsimGimbal() = default;
    void AirsimGimbal::start() { impl_->start(); }
    void AirsimGimbal::stop() { impl_->stop(); }
    void AirsimGimbal::updateVehicleState(const sensor_msgs::msg::Imu &imu, double) { impl_->updateVehicleState(imu); }
}
#else
namespace custom_link_bridge
{
    class AirsimGimbal::Impl {};
    AirsimGimbal::AirsimGimbal(rclcpp::Node &, Callback)
    {
        throw std::runtime_error("AirSim gimbal unavailable: rebuild with -DAPM_BRIDGE_AIRSIM=ON -DAIRSIM_ROOT=<Cosys-AirSim checkout>, or disable airsim.gimbal_enabled");
    }
    AirsimGimbal::~AirsimGimbal() = default;
    void AirsimGimbal::start() {}
    void AirsimGimbal::stop() {}
    void AirsimGimbal::updateVehicleState(const sensor_msgs::msg::Imu &, double) {}
}
#endif
