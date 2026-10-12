#include "gimbal_interface.h"
#include "airsim_gimbal.h"
#include "gimbal.h"
#include "strapdown_gimbal.h"

#include <array>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#ifdef APM_BRIDGE_AIRSIM
#include <rpc/server.h>
#include <rpc/msgpack.hpp>
#endif

namespace
{
    static_assert(std::is_abstract_v<apm_bridge::Gimbal>);
    static_assert(std::is_base_of_v<apm_bridge::Gimbal, xfrobot::GimbalControl>);
    static_assert(std::is_base_of_v<apm_bridge::Gimbal, custom_link_bridge::AirsimGimbal>);
    static_assert(std::is_base_of_v<apm_bridge::Gimbal, apm_bridge::StrapdownGimbal>);

    void require(bool condition, const char *message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    uint16_t crc16(const uint8_t *bytes, size_t size)
    {
        uint16_t crc = 0;
        for (size_t i = 0; i < size; ++i)
        {
            crc ^= uint16_t(bytes[i]) << 8;
            for (int bit = 0; bit < 8; ++bit)
                crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        }
        return crc;
    }

    class Capture
    {
    public:
        void receive(sensor_msgs::msg::Imu::UniquePtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_ = *msg;
            ++count_;
        }
        size_t count()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return count_;
        }
        sensor_msgs::msg::Imu last()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return last_;
        }
    private:
        std::mutex mutex_;
        sensor_msgs::msg::Imu last_;
        size_t count_ = 0;
    };

    class Pty
    {
    public:
        Pty()
        {
            fd = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
            require(fd >= 0 && ::grantpt(fd) == 0 && ::unlockpt(fd) == 0, "pty setup failed");
            const char *name = ::ptsname(fd);
            require(name != nullptr, "pty slave path unavailable");
            port = name;
        }
        ~Pty() { if (fd >= 0) ::close(fd); }
        int fd = -1;
        std::string port;
    };

    sensor_msgs::msg::Imu vehicle(double heading)
    {
        sensor_msgs::msg::Imu msg;
        msg.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
        msg.linear_acceleration.x = 1.237;
        msg.linear_acceleration.y = 2.345;
        msg.linear_acceleration.z = 9.80665 + .053;
        return msg;
    }

    void factoryDisabledAndErrors()
    {
        auto camera = std::make_shared<rclcpp::Node>("gimbal_default_camera_test");
        camera->declare_parameter<bool>("airsim.enabled", false);
        require(apm_bridge::createGimbal(*camera, [](auto) {}) != nullptr, "empty hardware port did not select strapdown camera");

        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("airsim.enabled", true),
                                     rclcpp::Parameter("airsim.gimbal_enabled", false)});
        auto sim = std::make_shared<rclcpp::Node>("gimbal_sim_disabled_test", options);
        sim->declare_parameter<bool>("airsim.enabled", false);
        require(!apm_bridge::createGimbal(*sim, [](auto) {}), "disabled AirSim gimbal was constructed");

        options.parameter_overrides({rclcpp::Parameter("airsim.enabled", true),
                                     rclcpp::Parameter("gimbal_port", "/invalid")});
        auto invalid = std::make_shared<rclcpp::Node>("gimbal_invalid_test", options);
        invalid->declare_parameter<bool>("airsim.enabled", false);
        bool rejected = false;
        try { apm_bridge::createGimbal(*invalid, [](auto) {}); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "AirSim accepted a hardware gimbal port");

        options.parameter_overrides({rclcpp::Parameter("gimbal.fixed_pitch", -20.0)});
        auto obsolete = std::make_shared<rclcpp::Node>("gimbal_obsolete_pitch_test", options);
        obsolete->declare_parameter<bool>("airsim.enabled", false);
        rejected = false;
        try { apm_bridge::createGimbal(*obsolete, [](auto) {}); }
        catch (const std::invalid_argument &) { rejected = true; }
        require(rejected, "obsolete fixed_pitch was silently ignored");

        options.parameter_overrides({rclcpp::Parameter("gimbal.model", "C20S"),
                                     rclcpp::Parameter("gimbal.pitch_hold", 18.0)});
        auto hardware = std::make_shared<rclcpp::Node>("gimbal_shared_pitch_test", options);
        const auto config = xfrobot::GimbalConfig::fromParameters(*hardware);
        require(config.pitch_hold_deg == 18.0f, "shared pitch_hold changed the hardware control target");
        require(hardware->has_parameter("gimbal.pitch_hold") && !hardware->has_parameter("gimbal.fixed_pitch"),
                "hardware declared more than one pitch parameter");
    }

    void strapdownBackend()
    {
        Capture capture;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("gimbal.pitch_hold", -30.0)});
        auto node = std::make_shared<rclcpp::Node>("strapdown_gimbal_test", options);
        node->declare_parameter<bool>("airsim.enabled", false);
        std::unique_ptr<apm_bridge::Gimbal> gimbal = apm_bridge::createGimbal(
            *node, [&](auto msg) { capture.receive(std::move(msg)); });
        require(node->has_parameter("gimbal.pitch_hold") && !node->has_parameter("gimbal.fixed_pitch"),
                "strapdown declared more than one pitch parameter");
        auto imu = vehicle(0);
        imu.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI / 4));
        imu.header.stamp.sec = 123;
        imu.header.stamp.nanosec = 456;
        imu.angular_velocity.z = 1.0;
        imu.angular_velocity_covariance = {1, 0, 0, 0, 4, 0, 0, 0, 9};
        gimbal->updateVehicleState(imu, 9.80665);
        require(capture.count() == 0, "strapdown emitted before start");
        gimbal->start();
        gimbal->start();
        gimbal->updateVehicleState(imu, 9.80665);
        require(capture.count() == 1, "strapdown did not publish once per vehicle update");
        const auto msg = capture.last();
        tf2::Quaternion q;
        tf2::fromMsg(msg.orientation, q);
        const auto forward = tf2::quatRotate(q, tf2::Vector3(1, 0, 0));
        require(std::abs(forward.x() - std::sqrt(.75)) < 1e-6 &&
                std::abs(forward.y() + std::sqrt(.125)) < 1e-6 &&
                std::abs(forward.z() - std::sqrt(.125)) < 1e-6,
                "strapdown mount was added as Euler angles or composed in the wrong order");
        require(std::abs(msg.angular_velocity.x - .5) < 1e-6 &&
                std::abs(msg.angular_velocity.z - std::sqrt(.75)) < 1e-6,
                "strapdown did not rotate body rates to camera FLU");
        require(std::abs(msg.angular_velocity_covariance[0] - 3) < 1e-6 &&
                std::abs(msg.angular_velocity_covariance[4] - 4) < 1e-6 &&
                std::abs(msg.angular_velocity_covariance[8] - 7) < 1e-6 &&
                std::abs(msg.angular_velocity_covariance[2] - std::sqrt(12.0)) < 1e-6 &&
                std::abs(msg.angular_velocity_covariance[6] - std::sqrt(12.0)) < 1e-6,
                "strapdown angular covariance frame is incorrect");
        require(msg.header.frame_id == "base_link" && msg.header.stamp == imu.header.stamp,
                "strapdown did not preserve sample time and the callback frame contract");
        require(msg.linear_acceleration_covariance[0] == -1, "strapdown fabricated camera acceleration");
        imu = vehicle(M_PI_2);
        gimbal->updateVehicleState(imu, 9.80665);
        tf2::fromMsg(capture.last().orientation, q);
        const auto north_up = tf2::quatRotate(q, tf2::Vector3(1, 0, 0));
        require(std::abs(north_up.x()) < 1e-6 &&
                std::abs(north_up.y() - std::sqrt(.75)) < 1e-6 &&
                std::abs(north_up.z() - .5) < 1e-6, "negative fixed pitch must point above the body forward axis");
        require(std::abs(capture.last().orientation_covariance[1] + M_PI / 6) < 1e-6,
                "strapdown legacy heading-relative pitch has the wrong sign");
        imu.angular_velocity_covariance[0] = -1;
        gimbal->updateVehicleState(imu, 9.80665);
        require(capture.last().angular_velocity_covariance[0] == -1, "strapdown lost angular availability flag");
        const auto count = capture.count();
        sensor_msgs::msg::Imu invalid;
        invalid.orientation.w = 0.0;
        gimbal->updateVehicleState(invalid, 9.80665);
        require(capture.count() == count, "invalid vehicle quaternion was published");
        gimbal->stop();
        gimbal->stop();
        gimbal->updateVehicleState(imu, 9.80665);
        require(capture.count() == count, "strapdown published after stop");
        gimbal->start();
        gimbal->updateVehicleState(imu, 9.80665);
        require(capture.count() == count + 1, "strapdown did not restart");
        gimbal->stop();

        Capture down_capture;
        options.parameter_overrides({rclcpp::Parameter("gimbal.pitch_hold", 30.0)});
        auto down_node = std::make_shared<rclcpp::Node>("strapdown_positive_pitch_test", options);
        down_node->declare_parameter<bool>("airsim.enabled", false);
        auto down_camera = apm_bridge::createGimbal(*down_node, [&](auto msg) { down_capture.receive(std::move(msg)); });
        down_camera->start();
        down_camera->updateVehicleState(vehicle(M_PI_2), 9.80665);
        tf2::fromMsg(down_capture.last().orientation, q);
        require(std::abs(tf2::quatRotate(q, tf2::Vector3(1, 0, 0)).z() + .5) < 1e-6,
                "positive fixed pitch must point below the body forward axis");
        down_camera->stop();

        for (double pitch : {181.0, std::numeric_limits<double>::quiet_NaN()})
        {
            options.parameter_overrides({rclcpp::Parameter("gimbal.pitch_hold", pitch)});
            auto bad = std::make_shared<rclcpp::Node>("strapdown_invalid_pitch_test", options);
            bad->declare_parameter<bool>("airsim.enabled", false);
            bool rejected = false;
            try { apm_bridge::createGimbal(*bad, [](auto) {}); }
            catch (const std::invalid_argument &) { rejected = true; }
            require(rejected, "invalid fixed camera pitch was accepted");
        }
        std::cout << "PASS: strapdown factory, mixed attitude, FLU rates/covariance, lifecycle and input validation\n";
    }

    void hardwareBackend()
    {
        Pty pty;
        Capture capture;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("gimbal_port", pty.port),
                                     rclcpp::Parameter("gimbal.model", "C200T"),
                                     rclcpp::Parameter("gimbal.send_uav_data", true)});
        auto node = std::make_shared<rclcpp::Node>("gimbal_hardware_interface_test", options);
        node->declare_parameter<bool>("airsim.enabled", false);
        std::unique_ptr<apm_bridge::Gimbal> gimbal = apm_bridge::createGimbal(
            *node, [&](auto msg) { capture.receive(std::move(msg)); });
        require(gimbal != nullptr, "hardware factory returned no backend");
        const auto imu = vehicle(M_PI_2);
        gimbal->updateVehicleState(imu, 9.80665);
        gimbal->start();
        gimbal->start();

        std::vector<uint8_t> buffer;
        bool fusion_seen = false;
        unsigned stop_packets = 0;
        auto service = [&]() {
            std::array<uint8_t, 4096> input{};
            const auto count = ::read(pty.fd, input.data(), input.size());
            if (count > 0)
                buffer.insert(buffer.end(), input.begin(), input.begin() + count);
            while (buffer.size() >= sizeof(xfrobot::Gcu2GbcPkt_t))
            {
                if (buffer[0] != 0xA9 || buffer[1] != 0x5B)
                {
                    buffer.erase(buffer.begin());
                    continue;
                }
                const auto checksum = crc16(buffer.data(), 38);
                require(checksum == ((uint16_t(buffer[38]) << 8) | buffer[39]), "hardware control CRC");
                xfrobot::Gcu2GbcPkt_t packet;
                std::memcpy(&packet, buffer.data(), sizeof(packet));
                buffer.erase(buffer.begin(), buffer.begin() + sizeof(packet));
                if (packet.cmd.value == 3)
                    ++stop_packets;
                if (packet.uav.valid)
                {
                    require(packet.uav.angle[0] == 0 && packet.uav.angle[1] == 0 && packet.uav.angle[2] == 0, "vehicle FRD angle conversion");
                    require(packet.uav.accel[0] == 123 && packet.uav.accel[1] == -234 && packet.uav.accel[2] == 5, "vehicle NEU acceleration conversion/quantization");
                    fusion_seen = true;
                }
                xfrobot::Gbc2GcuPkt_t reply{};
                reply.sync[0] = 0xB5;
                reply.sync[1] = 0x9A;
                reply.fw_ver = 36;
                reply.gbc_stat = 4;
                reply.tca_flag = 1;
                reply.cmd.value = 4;
                reply.cmd.stat = 1;
                reply.cam_angle[0] = 800;
                reply.cam_angle[1] = -1200;
                reply.mtr_angle[0] = -2000;
                reply.mtr_angle[1] = 1000;
                reply.mtr_angle[2] = -600;
                reply.cam_rate[0] = 10;
                reply.cam_rate[1] = 20;
                reply.cam_rate[2] = 30;
                const auto crc = crc16(reinterpret_cast<const uint8_t *>(&reply), 24);
                reply.crc[0] = crc >> 8;
                reply.crc[1] = crc & 0xff;
                const auto written = ::write(pty.fd, &reply, sizeof(reply));
                require(written == sizeof(reply), "pty gimbal reply failed");
            }
        };
        auto drive = [&](std::chrono::milliseconds duration, bool update) {
            const auto deadline = std::chrono::steady_clock::now() + duration;
            while (std::chrono::steady_clock::now() < deadline)
            {
                if (update)
                    gimbal->updateVehicleState(imu, 9.80665);
                service();
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        };
        drive(std::chrono::milliseconds(500), true);
        require(capture.count() > 10 && fusion_seen, "hardware abstract callback/fusion missing");
        const auto msg = capture.last();
        tf2::Quaternion actual;
        tf2::fromMsg(msg.orientation, actual);
        const tf2::Quaternion heading(tf2::Vector3(0, 0, 1), M_PI_2);
        const tf2::Quaternion yaw(tf2::Vector3(0, 0, 1), 6 * M_PI / 180);
        const tf2::Quaternion roll(tf2::Vector3(1, 0, 0), 8 * M_PI / 180);
        const tf2::Quaternion pitch(tf2::Vector3(0, 1, 0), 12 * M_PI / 180);
        require(std::abs(actual.dot(heading * yaw * roll * pitch)) > .99999, "hardware ENU/FLU orientation changed");
        require(msg.header.frame_id == "base_link", "hardware frame changed");
        require(std::abs(msg.orientation_covariance[2] - 6 * M_PI / 180) < 1e-6, "hardware legacy Euler slots changed");
        require(msg.angular_velocity.z < 0, "hardware CW -> CCW yaw-rate conversion changed");
        gimbal->stop();
        gimbal->stop();
        const auto stopped_count = capture.count();
        drive(std::chrono::milliseconds(50), false);
        require(capture.count() == stopped_count && stop_packets == 1, "hardware stop is not idempotent or timer survived");
        gimbal->start();
        drive(std::chrono::milliseconds(100), true);
        require(capture.count() > stopped_count, "hardware backend did not restart");
        gimbal->stop();
        std::cout << "PASS: Gimbal hardware lifecycle, ENU pose, legacy fields and FRD/NEU fusion\n";
    }

#ifdef APM_BRIDGE_AIRSIM
    struct RpcOrientation
    {
        double x_val = 0, y_val = 0, z_val = 0, w_val = 1;
        MSGPACK_DEFINE_MAP(x_val, y_val, z_val, w_val);
    };
    struct RpcPose
    {
        RpcOrientation orientation;
        MSGPACK_DEFINE_MAP(orientation);
    };
    struct RpcCamera
    {
        RpcPose pose;
        MSGPACK_DEFINE_MAP(pose);
    };

    void airsimBackend()
    {
        rpc::server server("127.0.0.1", 0);
        server.bind("simGetCameraInfo", [](const std::string &camera, const std::string &vehicle) {
            require(camera == "0" && vehicle == "TestVehicle", "AirSim endpoint parameters changed");
            return RpcCamera{};
        });
        server.async_run(1);
        Capture capture;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("airsim.enabled", true),
                                     rclcpp::Parameter("airsim.port", static_cast<int64_t>(server.port())),
                                     rclcpp::Parameter("airsim.vehicle_name", "TestVehicle"),
                                     rclcpp::Parameter("airsim.timeout", .2)});
        auto node = std::make_shared<rclcpp::Node>("gimbal_airsim_interface_test", options);
        node->declare_parameter<bool>("airsim.enabled", false);
        std::unique_ptr<apm_bridge::Gimbal> gimbal = apm_bridge::createGimbal(
            *node, [&](auto msg) { capture.receive(std::move(msg)); });
        gimbal->updateVehicleState(vehicle(M_PI / 3), 9.80665);
        gimbal->start();
        gimbal->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        require(capture.count() >= 3, "AirSim abstract callback missing");
        auto check = [&](double relative_yaw) {
            const auto msg = capture.last();
            tf2::Quaternion q;
            tf2::fromMsg(msg.orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            require(std::abs(yaw - M_PI_2) < 1e-6, "AirSim world orientation was double-rotated");
            require(std::abs(msg.orientation_covariance[2] - relative_yaw) < 1e-6, "AirSim heading-relative Euler slots changed");
            require(msg.header.frame_id == "base_link", "AirSim frame changed");
        };
        check(M_PI / 6);
        gimbal->updateVehicleState(vehicle(-M_PI / 6), 9.80665);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        check(2 * M_PI / 3);
        gimbal->stop();
        gimbal->stop();
        const auto stopped = capture.count();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        require(capture.count() == stopped, "AirSim callback survived stop");
        gimbal->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        require(capture.count() > stopped, "AirSim backend did not restart");
        gimbal->stop();
        server.stop();
        std::cout << "PASS: Gimbal AirSim lifecycle, ENU pose and heading-relative legacy fields\n";
    }
#else
    void airsimBackend()
    {
        auto node = std::make_shared<rclcpp::Node>("gimbal_no_sdk_test");
        node->declare_parameter<bool>("airsim.enabled", true);
        bool rejected = false;
        try { apm_bridge::createGimbal(*node, [](auto) {}); }
        catch (const std::runtime_error &) { rejected = true; }
        require(rejected, "SDK-free build silently accepted AirSim gimbal");
        std::cout << "PASS: missing AirSim SDK is an explicit error\n";
    }
#endif
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        factoryDisabledAndErrors();
        strapdownBackend();
        hardwareBackend();
        airsimBackend();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
