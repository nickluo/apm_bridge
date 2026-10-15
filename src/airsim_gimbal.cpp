#include "airsim_gimbal.h"
#include "gimbal.h"

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
#include <rpc/msgpack.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace custom_link_bridge
{
    namespace
    {
        struct RpcVector
        {
            double x_val = 0, y_val = 0, z_val = 0;
            MSGPACK_DEFINE_MAP(x_val, y_val, z_val);
        };
        struct RpcQuaternion
        {
            double x_val = 0, y_val = 0, z_val = 0, w_val = 1;
            MSGPACK_DEFINE_MAP(x_val, y_val, z_val, w_val);
        };
        struct RpcPose
        {
            RpcVector position;
            RpcQuaternion orientation;
            MSGPACK_DEFINE_MAP(position, orientation);
        };
        using ObjectMap = std::map<std::string, RPCLIB_MSGPACK::object>;

        tf2::Quaternion readQuaternion(const ObjectMap &pose)
        {
            const auto q = pose.at("orientation").as<ObjectMap>();
            tf2::Quaternion result(q.at("x_val").as<double>(), q.at("y_val").as<double>(),
                                   q.at("z_val").as<double>(), q.at("w_val").as<double>());
            if (!std::isfinite(result.length2()) || result.length2() < 1e-12)
                throw std::runtime_error("Invalid AirSim quaternion");
            return result.normalized();
        }

        tf2::Vector3 readPosition(const ObjectMap &pose)
        {
            const auto p = pose.at("position").as<ObjectMap>();
            tf2::Vector3 result(p.at("x_val").as<double>(), p.at("y_val").as<double>(),
                                p.at("z_val").as<double>());
            if (!std::isfinite(result.length2()))
                throw std::runtime_error("Invalid AirSim position");
            return result;
        }

        tf2::Quaternion nedToEnu(const tf2::Quaternion &ned)
        {
            const double h = std::sqrt(0.5);
            return tf2::Quaternion(h, h, 0, 0) * ned * tf2::Quaternion(1, 0, 0, 0);
        }

        tf2::Quaternion enuRelativeToFrd(const tf2::Quaternion &relative)
        {
            return tf2::Quaternion(relative.x(), -relative.y(), -relative.z(), relative.w());
        }
    }

    class AirsimGimbal::Impl
    {
    public:
        Impl(rclcpp::Node &node, Callback callback)
            : node_(node), callback_(std::move(callback)), cfg_(xfrobot::GimbalConfig::fromParameters(node)),
              aim_target_degrees_(cfg_.aimTargetDegrees()), tracking_(cfg_.tracking)
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
            RCLCPP_INFO(node_.get_logger(),
                        "AirSim gimbal owns camera pose; set camera Gimbal.Stabilization=0 to avoid competing controllers");
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
        void setMode(apm_bridge::GimbalMode mode)
        {
            const auto previous = mode_.exchange(mode);
            if (mode == apm_bridge::GimbalMode::AIM && previous != mode &&
                std::abs(aim_target_degrees_[1] - (90.0 + cfg_.pitch_aim_deg)) > 1e-6)
                RCLCPP_WARN(node_.get_logger(),
                    "AIM requested %.2f deg relative pitch, limited to %.2f deg; body FLU +Z is not reachable with this offset/limit",
                    90.0 + cfg_.pitch_aim_deg, aim_target_degrees_[1]);
        }
        void updateTargetLos(const apm_bridge::TargetLos &target) { tracking_.update(target); }

    private:
        double missionBank(const tf2::Quaternion &body, const tf2::Quaternion &camera)
        {
            double r, p, heading;
            tf2::Matrix3x3(body).getRPY(r, p, heading);
            const tf2::Matrix3x3 joint(body.inverse() * camera);
            if (cfg_.yaw.present)
                heading += std::atan2(-joint[0][1], joint[1][1]);
            const tf2::Vector3 forward(std::cos(heading), std::sin(heading), 0);
            const tf2::Vector3 level_left(-forward.y(), forward.x(), 0);
            auto left = tf2::quatRotate(camera, tf2::Vector3(0, 1, 0));
            left -= forward * left.dot(forward);
            if (left.length2() < 1e-12)
            {
                RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                    "MISSION camera bank is unobservable; holding roll rate at zero");
                return 0;
            }
            return std::atan2(level_left.cross(left).dot(forward), level_left.dot(left));
        }

        tf2::Quaternion horizonOrientation(const tf2::Quaternion &body)
        {
            const auto forward = tf2::quatRotate(body, tf2::Vector3(1, 0, 0));
            const double horizontal = std::hypot(forward.x(), forward.y());
            if (horizontal > 1e-6)
                horizon_heading_ = std::atan2(forward.y(), forward.x());
            const tf2::Vector3 f(std::cos(horizon_heading_), std::sin(horizon_heading_), 0);
            const tf2::Vector3 reference(-f.y(), f.x(), 0);
            auto left = tf2::quatRotate(body, tf2::Vector3(0, 1, 0));
            left -= f * left.dot(f);
            const double bank = cfg_.roll.present ? 0.0 :
                std::atan2(reference.cross(left).dot(f), reference.dot(left));
            return tf2::Quaternion(tf2::Vector3(0, 0, 1), horizon_heading_) *
                tf2::Quaternion(tf2::Vector3(0, 1, 0), -cfg_.pitch_hold_deg * M_PI / 180.0) *
                tf2::Quaternion(tf2::Vector3(1, 0, 0), bank);
        }

        tf2::Quaternion limitRelative(const tf2::Quaternion &relative)
        {
            double roll, pitch, yaw;
            tf2::Matrix3x3(relative).getRPY(roll, pitch, yaw);
            const auto limit = [](double angle, const xfrobot::GimbalAxisConfig &axis, double sign) {
                if (!axis.present)
                    return 0.0;
                if (axis.min_deg == axis.max_deg)
                    return angle;
                return sign * std::clamp(sign * angle * 180.0 / M_PI,
                    double(axis.min_deg), double(axis.max_deg)) * M_PI / 180.0;
            };
            tf2::Quaternion result;
            result.setRPY(limit(roll, cfg_.roll, 1), limit(pitch, cfg_.pitch, -1),
                          limit(yaw, cfg_.yaw, -1));
            return result;
        }

        void run()
        {
            rpc::client client(host_, static_cast<uint16_t>(port_));
            client.set_timeout(static_cast<int64_t>(timeout_ * 1000));
            std::optional<tf2::Quaternion> previous;
            std::optional<tf2::Quaternion> mission_world;
            double last_sample_time = 0;
            double last_control_time = node_.now().seconds();
            std::optional<tf2::Vector3> mount_position;
            auto previous_mode = mode_.load();
            while (running_)
            {
                try
                {
                    const auto response = client.call("simGetCameraInfo", camera_, vehicle_);
                    const auto camera_stamp = node_.now();
                    const auto info = response.get().as<ObjectMap>();
                    const auto pose = info.at("pose").as<ObjectMap>();
                    const tf2::Quaternion enu = nedToEnu(readQuaternion(pose));
                    const auto vehicle_response = client.call("simGetVehiclePose", vehicle_);
                    const auto vehicle_pose = vehicle_response.get().as<ObjectMap>();
                    const auto body_ned = readQuaternion(vehicle_pose);
                    const auto body = nedToEnu(body_ned);
                    if (!mount_position)
                        mount_position = tf2::quatRotate(body_ned.inverse(),
                            readPosition(pose) - readPosition(vehicle_pose));
                    const double control_time = node_.now().seconds();
                    const double dt = std::clamp(control_time - last_control_time, 0.0, 0.1);
                    last_control_time = control_time;
                    const auto mode = mode_.load();
                    if (mode != previous_mode)
                    {
                        tracking_.reset();
                        mission_world.reset();
                        previous_mode = mode;
                        roll_hold_active_ = false;
                    }
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(enu).getRPY(roll, pitch, yaw);
                    const double bank = mode == apm_bridge::GimbalMode::MISSION ? missionBank(body, enu) : roll;
                    if (!roll_hold_active_ && std::abs(bank) > cfg_.roll_hold_enter_deg * M_PI / 180.0)
                        roll_hold_active_ = true;
                    else if (roll_hold_active_ && std::abs(bank) < cfg_.roll_hold_exit_deg * M_PI / 180.0)
                        roll_hold_active_ = false;
                    const bool protection = cfg_.model == xfrobot::GimbalModel::C20S && roll_hold_active_;
                    tf2::Quaternion relative(0, 0, 0, 1);
                    if (mode == apm_bridge::GimbalMode::AIM)
                        relative = tf2::Quaternion(tf2::Vector3(0, 0, 1), -aim_target_degrees_[2] * M_PI / 180.0) *
                            tf2::Quaternion(tf2::Vector3(1, 0, 0), aim_target_degrees_[0] * M_PI / 180.0) *
                            tf2::Quaternion(tf2::Vector3(0, 1, 0), -aim_target_degrees_[1] * M_PI / 180.0);
                    else if (mode == apm_bridge::GimbalMode::HORIZON)
                        relative = body.inverse() * (protection ? enu : horizonOrientation(body));
                    else if (mode == apm_bridge::GimbalMode::MISSION)
                    {
                        if (!mission_world)
                            mission_world = enu;
                        std::array<double, 3> rates{};
                        if (dt > 0 && !protection)
                            rates = tracking_.step(cfg_.roll.present ? -bank : 0.0, dt,
                                apm_bridge::GimbalTrackingController::Clock::now(),
                                {cfg_.roll.present, cfg_.pitch.present, cfg_.yaw.present});
                        else
                            tracking_.reset();
                        if (!tracking_.fresh())
                            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                "MISSION gimbal: target LOS missing/stale; decelerating to locked hold");
                        const tf2::Vector3 omega(cfg_.roll.present ? rates[0] : 0.0,
                            rates[1], cfg_.yaw.present ? rates[2] : 0.0);
                        const double speed = omega.length();
                        const tf2::Quaternion delta = speed > 1e-9 ?
                            tf2::Quaternion(omega / speed, speed * dt) : tf2::Quaternion(0, 0, 0, 1);
                        relative = body.inverse() * *mission_world * delta;
                    }
                    if (mode == apm_bridge::GimbalMode::MISSION)
                    {
                        relative = cfg_.constrainMissionOrientation(relative);
                        mission_world = body * relative;
                    }
                    else if (mode == apm_bridge::GimbalMode::HORIZON)
                        relative = limitRelative(relative);
                    const auto frd = enuRelativeToFrd(relative);
                    RpcPose command;
                    command.position = {mount_position->x(), mount_position->y(), mount_position->z()};
                    command.orientation = {frd.x(), frd.y(), frd.z(), frd.w()};
                    client.call("simSetCameraPose", camera_, command, vehicle_);
                    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
                    msg->header.stamp = camera_stamp;
                    msg->header.frame_id = "base_link";
                    msg->orientation = tf2::toMsg(enu);
                    const tf2::Quaternion heading(tf2::Vector3(0, 0, 1), vehicle_heading_.load());
                    tf2::Matrix3x3(heading.inverse() * enu).getRPY(msg->orientation_covariance[0],
                        msg->orientation_covariance[1], msg->orientation_covariance[2]);
                    msg->angular_velocity_covariance[0] = -1;
                    msg->linear_acceleration_covariance[0] = -1;
                    const double sample_time = rclcpp::Time(msg->header.stamp).seconds();
                    if (previous)
                    {
                        auto delta = previous->inverse() * enu;
                        if (delta.w() < 0)
                            delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
                        const double sample_dt = sample_time - last_sample_time;
                        if (sample_dt > 1e-6 && sample_dt < 0.3)
                        {
                            const auto omega = delta.getAngle() > 1e-9 ?
                                delta.getAxis() * (delta.getAngle() / sample_dt) : tf2::Vector3(0, 0, 0);
                            msg->angular_velocity.x = omega.x();
                            msg->angular_velocity.y = omega.y();
                            msg->angular_velocity.z = omega.z();
                            msg->angular_velocity_covariance[0] = 0;
                        }
                    }
                    previous = enu;
                    last_sample_time = sample_time;
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
        xfrobot::GimbalConfig cfg_;
        std::array<double, 3> aim_target_degrees_;
        apm_bridge::GimbalTrackingController tracking_;
        std::string host_, vehicle_, camera_;
        int port_;
        double timeout_;
        std::atomic_bool running_{false};
        std::atomic<double> vehicle_heading_{0.0};
        std::atomic<apm_bridge::GimbalMode> mode_{apm_bridge::GimbalMode::HORIZON};
        double horizon_heading_ = 0;
        bool roll_hold_active_ = false;
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
    void AirsimGimbal::setMode(apm_bridge::GimbalMode mode) { impl_->setMode(mode); }
    void AirsimGimbal::updateTargetLos(const apm_bridge::TargetLos &target) { impl_->updateTargetLos(target); }
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
    void AirsimGimbal::setMode(apm_bridge::GimbalMode) {}
    void AirsimGimbal::updateTargetLos(const apm_bridge::TargetLos &) {}
}
#endif
