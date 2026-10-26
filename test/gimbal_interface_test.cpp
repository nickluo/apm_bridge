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
#include <rpc/client.h>
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

    class SerialGimbalSimulator
    {
    public:
        SerialGimbalSimulator()
        {
            reply.sync[0] = 0xB5;
            reply.sync[1] = 0x9A;
            reply.gbc_stat = 4;
            reply.cmd.value = 4;
            reply.cmd.stat = 1;
        }
        void service()
        {
            std::array<uint8_t, 4096> input{};
            const auto count = ::read(pty.fd, input.data(), input.size());
            if (count > 0)
                buffer.insert(buffer.end(), input.begin(), input.begin() + count);
            while (buffer.size() >= sizeof(control))
            {
                if (buffer[0] != 0xA9 || buffer[1] != 0x5B)
                {
                    buffer.erase(buffer.begin());
                    continue;
                }
                const auto crc = crc16(buffer.data(), 38);
                require(crc == ((uint16_t(buffer[38]) << 8) | buffer[39]), "C20S control CRC");
                std::memcpy(&control, buffer.data(), sizeof(control));
                buffer.erase(buffer.begin(), buffer.begin() + sizeof(control));
                const auto checksum = crc16(reinterpret_cast<const uint8_t *>(&reply), 24);
                reply.crc[0] = checksum >> 8;
                reply.crc[1] = checksum & 0xff;
                require(::write(pty.fd, &reply, sizeof(reply)) == sizeof(reply), "C20S feedback write");
            }
        }
        Pty pty;
        xfrobot::Gbc2GcuPkt_t reply{};
        xfrobot::Gcu2GbcPkt_t control{};
    private:
        std::vector<uint8_t> buffer;
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
                                     rclcpp::Parameter("gimbal.pitch_aim", -55.0),
                                     rclcpp::Parameter("gimbal.pitch_hold", 18.0)});
        auto hardware = std::make_shared<rclcpp::Node>("gimbal_shared_pitch_test", options);
        const auto config = xfrobot::GimbalConfig::fromParameters(*hardware);
        require(config.pitch_hold_deg == 18.0f, "shared pitch_hold changed the hardware control target");
        require(config.pitch_aim_deg == -55.0f && config.aimTargetDegrees()[1] == 35.0,
                "pitch_aim was coupled to HORIZON pitch_hold");
        require(hardware->has_parameter("gimbal.pitch_hold") && !hardware->has_parameter("gimbal.fixed_pitch"),
                "hardware declared more than one pitch parameter");
    }

    void aimAndMissionLimits()
    {
        for (const auto model : {xfrobot::GimbalModel::C20S, xfrobot::GimbalModel::C40D, xfrobot::GimbalModel::C200T})
        {
            auto config = xfrobot::GimbalConfig::defaultsFor(model);
            require(config.aimTargetDegrees()[1] == 40.0, "unreachable body +Z AIM was not limited safely");
            config.pitch.max_deg = 120;
            const auto fromPitch = [](double degrees) {
                return tf2::Quaternion(tf2::Vector3(0, 1, 0), -degrees * M_PI / 180.0);
            };
            for (double offset : {-10.0, 0.0, 10.0})
            {
                config.pitch_aim_deg = offset;
                const auto aim = config.aimTargetDegrees();
                const auto desired = fromPitch(aim[1]);
                const auto forward = tf2::quatRotate(desired, tf2::Vector3(1, 0, 0));
                require(std::abs(forward.x() + std::sin(offset * M_PI / 180.0)) < 1e-8 &&
                        std::abs(forward.z() - std::cos(offset * M_PI / 180.0)) < 1e-8,
                        "AIM does not reference body FLU +Z with a nose-up offset");
                require(std::abs(desired.dot(config.constrainMissionOrientation(desired))) > .999999,
                        "joint-limit decomposition flipped the camera at 90-degree pitch");
            }
            const auto upper = config.constrainMissionOrientation(fromPitch(140));
            require(std::abs(upper.dot(fromPitch(120))) > .999999, "MISSION upper pitch limit failed");
            const auto lower = config.constrainMissionOrientation(fromPitch(-150));
            require(std::abs(lower.dot(fromPitch(-135))) > .999999, "MISSION lower pitch limit failed");
            config.pitch_aim_deg = 91;
            bool rejected = false;
            try { config.aimTargetDegrees(); }
            catch (const std::invalid_argument &) { rejected = true; }
            require(rejected, "invalid pitch_aim was accepted");
        }
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

    tf2::Matrix3x3 jointRotation(double roll, double pitch, double yaw)
    {
        const double cr = std::cos(roll), sr = std::sin(roll);
        const double cp = std::cos(pitch), sp = std::sin(pitch);
        const double cy = std::cos(yaw), sy = std::sin(yaw);
        const tf2::Matrix3x3 frd_to_flu(1, 0, 0, 0, -1, 0, 0, 0, -1);
        return frd_to_flu * tf2::Matrix3x3(cy, -sy, 0, sy, cy, 0, 0, 0, 1) *
            tf2::Matrix3x3(1, 0, 0, 0, cr, -sr, 0, sr, cr) *
            tf2::Matrix3x3(cp, 0, sp, 0, 1, 0, -sp, 0, cp) * frd_to_flu;
    }

    void jointKinematics()
    {
        tf2::Quaternion body;
        body.setRPY(.4, -.3, 1.2);
        const double radians = M_PI / 180;
        for (const auto &axes : {apm_bridge::GimbalAxes{false, true, false},
                                 apm_bridge::GimbalAxes{true, true, false},
                                 apm_bridge::GimbalAxes{true, true, true}})
            for (double roll : {-110.0, -90.0, -35.0, 0.0, 89.0, 90.0, 110.0})
                for (double pitch : {-110.0, -90.0, 0.0, 89.0, 90.0, 91.0, 110.0})
                {
                    const apm_bridge::GimbalJointAngles joints{roll * radians, pitch * radians, -.7};
                    const auto expected = tf2::Matrix3x3(body) * jointRotation(axes[0] ? joints[0] : 0,
                        axes[1] ? joints[1] : 0, axes[2] ? joints[2] : 0);
                    const auto actual = apm_bridge::gimbalCameraToWorld(body, joints, axes);
                    for (int axis = 0; axis < 3; ++axis)
                        require((tf2::Matrix3x3(actual).getColumn(axis) - expected.getColumn(axis)).length() < 1e-8,
                                "Joint order or FRD-to-FLU conversion differs from rotation-matrix ground truth");
                    const auto encoders = apm_bridge::gimbalJointAngles(body.inverse() * actual, axes);
                    require((axes[0] || encoders[0] == 0) && (axes[2] || encoders[2] == 0),
                            "Virtual encoder returned an absent axis");
                    const auto reconstructed = apm_bridge::gimbalCameraToWorld(body, encoders, axes);
                    require(std::abs(actual.dot(reconstructed)) > 1 - 1e-10,
                            "Virtual encoder decomposition lost pose at a roll/pitch singularity");
                }
        std::cout << "PASS: all-model joint rotation matrices, axis masks, roll singularities and pitch pole round-trips\n";
    }

    void hardwarePose(xfrobot::GimbalModel model)
    {
        SerialGimbalSimulator simulator;
        Capture capture;
        auto node = std::make_shared<rclcpp::Node>("gimbal_hardware_pose_test");
        auto config = xfrobot::GimbalConfig::defaultsFor(model);
        config.pitch.min_deg = -120;
        config.pitch.max_deg = 120;
        config.pitch_aim_deg = -15;
        config.tracking.max_rate_dps = 140;
        config.tracking.max_accel_dps2 = 600;
        xfrobot::GimbalControl gimbal(*node, simulator.pty.port, config,
            [&](auto msg) { capture.receive(std::move(msg)); });
        gimbal.setMode(apm_bridge::GimbalMode::AIM);
        gimbal.start();
        auto imu = vehicle(0);
        auto drive = [&](int milliseconds, bool update = true) {
            const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
            while (std::chrono::steady_clock::now() < end)
            {
                if (update)
                    gimbal.updateVehicleState(imu, 9.80665);
                simulator.service();
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        };
        drive(50, false);
        require(capture.count() == 0, "C20S published without vehicle attitude");
        simulator.reply.cam_rate[0] = 100;
        simulator.reply.cam_rate[1] = 200;
        simulator.reply.cam_rate[2] = -300;
        // Absolute Euler data must not enter body-relative encoder reconstruction.
        simulator.reply.cam_angle[0] = 17000;
        simulator.reply.cam_angle[1] = -8000;
        simulator.reply.cam_angle[2] = 15000;
        drive(200);
        const std::array<std::array<double, 4>, 7> poses{{
            {0, 0, 0, 0}, {30, 20, 40, 0}, {-35, 25, -70, 40},
            {15, -25, 110, 89}, {15, -25, 110, 91}, {0, 0, 170, 110}, {20, 10, -160, -110}}};
        for (const auto &pose : poses)
        {
            const double radians = M_PI / 180;
            tf2::Quaternion body;
            body.setRPY(pose[0] * radians, pose[1] * radians, pose[2] * radians);
            imu.orientation = tf2::toMsg(body);
            simulator.reply.mtr_angle[0] = std::lround(-pose[3] * 100); // 编码器低头正 (台架实测)
            simulator.reply.mtr_angle[1] = pose[3] < 0 ? -2300 : 3100;
            simulator.reply.mtr_angle[2] = pose[3] < 0 ? 17000 : -6500;
            const auto before = capture.count();
            drive(60);
            require(capture.count() > before, "C20S camera pose missing");
            const auto msg = capture.last();
            tf2::Quaternion actual;
            tf2::fromMsg(msg.orientation, actual);
            const auto expected = tf2::Matrix3x3(body) * jointRotation(
                config.roll.present ? simulator.reply.mtr_angle[1] * .01 * radians : 0,
                pose[3] * radians,
                config.yaw.present ? simulator.reply.mtr_angle[2] * .01 * radians : 0);
            for (int axis = 0; axis < 3; ++axis)
                require((tf2::Matrix3x3(actual).getColumn(axis) - expected.getColumn(axis)).length() < 1e-6,
                        "Encoder reconstruction lost coupled attitude, axis mask, FRD sign or pole branch");
            const apm_bridge::GimbalAxes axes{config.roll.present, config.pitch.present, config.yaw.present};
            tf2::Quaternion relative_expected;
            (tf2::Matrix3x3(body).transpose() * expected).getRotation(relative_expected);
            const auto round_trip = apm_bridge::gimbalCameraToWorld(body,
                apm_bridge::gimbalJointAngles(relative_expected, axes), axes);
            require(std::abs(actual.dot(round_trip)) > .999999, "Virtual encoders do not round-trip serial pose");
            const tf2::Quaternion heading(tf2::Vector3(0, 0, 1), pose[2] * radians);
            double r, p, y;
            tf2::Matrix3x3(heading.inverse() * actual).getRPY(r, p, y);
            require(std::abs(msg.orientation_covariance[0] - r) < 1e-6 &&
                    std::abs(msg.orientation_covariance[1] - p) < 1e-6 &&
                    std::abs(msg.orientation_covariance[2] - y) < 1e-6,
                    "C20S legacy Euler slots disagree with reconstructed pose");
            const auto relative = gimbal.getOrientationFLU();
            require(relative != nullptr, "C20S standalone pose missing");
            tf2::Quaternion relative_q;
            tf2::fromMsg(relative->orientation, relative_q);
            require(std::abs(actual.dot(heading * relative_q)) > .999999, "C20S heading applied twice");
            require(std::abs(msg.angular_velocity.x - 20 * radians) < 1e-5 &&
                    std::abs(msg.angular_velocity.y - 10 * radians) < 1e-5 &&
                    std::abs(msg.angular_velocity.z - 30 * radians) < 1e-5,
                    "C20S pitch/roll/yaw feedback order or FRD signs");
        }
        drive(1500);
        require(std::abs(simulator.control.gbc[1].op_value - 7500) <= 1, "C20S AIM ignored corrected pitch limits");
        drive(260, false);
        const auto stale_count = capture.count();
        drive(40, false);
        require(capture.count() == stale_count && !gimbal.getOrientationFLU(),
                "C20S fabricated pose with stale vehicle attitude");
        drive(60);
        require(capture.count() > stale_count, "C20S did not recover from stale vehicle attitude");
        const auto frozen = gimbal.getOrientationFLU();
        require(frozen != nullptr, "C20S cached pose missing");
        tf2::Quaternion next_body;
        next_body.setRPY(-.4, .2, 1.0);
        auto next_imu = imu;
        next_imu.orientation = tf2::toMsg(next_body);
        gimbal.updateVehicleState(next_imu, 9.80665);
        const auto unchanged = gimbal.getOrientationFLU();
        require(unchanged && unchanged->header.stamp == frozen->header.stamp &&
                unchanged->orientation == frozen->orientation,
                "C20S changed a feedback sample's pose without a new encoder timestamp");
        auto invalid = imu;
        invalid.orientation = geometry_msgs::msg::Quaternion{};
        invalid.orientation.w = 0;
        gimbal.updateVehicleState(invalid, 9.80665);
        require(!gimbal.getOrientationFLU(), "C20S retained an invalidated vehicle attitude");
        gimbal.stop();
        std::cout << "PASS: model=" << static_cast<int>(model)
                  << " PTY mixed attitude, axis mask, pitch poles, gyro signs, limits and stale vehicle rejection\n";
    }

    void serialAngleTransitions()
    {
        SerialGimbalSimulator simulator;
        auto config = xfrobot::GimbalConfig::defaultsFor(xfrobot::GimbalModel::C20S);
        config.pitch.min_deg = -120;
        config.pitch.max_deg = 120;
        config.pitch_aim_deg = -10;
        config.tracking.max_rate_dps = 140;
        config.tracking.max_accel_dps2 = 600;
        xfrobot::GimbalControl gimbal(simulator.pty.port, config);
        gimbal.setMode(apm_bridge::GimbalMode::AIM);
        gimbal.start();
        double previous = 0;
        auto stamp = std::chrono::steady_clock::now();
        auto drive = [&](int milliseconds) {
            const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
            while (std::chrono::steady_clock::now() < end)
            {
                simulator.service();
                const auto &pitch = simulator.control.gbc[1];
                const auto now = std::chrono::steady_clock::now();
                if (simulator.control.cmd.value == 4 && pitch.op_type == 0)
                {
                    const double angle = pitch.op_value * .01;
                    const double dt = std::chrono::duration<double>(now - stamp).count();
                    require(std::abs(angle - previous) <= 140 * (dt + .005) + .03,
                            "Serial angle command stepped instead of following a bounded trajectory");
                    previous = angle;
                    // 实测语义: FPV/LOCK 角度 op 均抬头正, 编码器 = -op (姿态角略含水平偏置, 此处省略
                    // 以保持 op 域跨模式连续, 断言聚焦符号语义)
                    simulator.reply.mtr_angle[0] = static_cast<int16_t>(-pitch.op_value);
                    simulator.reply.cam_angle[1] = pitch.op_value;
                }
                stamp = now;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        };
        drive(2000);
        require(std::abs(previous - 80) < .05, "Serial AIM trajectory did not settle");
        gimbal.setMode(apm_bridge::GimbalMode::HORIZON);
        drive(2000);
        require(std::abs(previous) < .05, "Serial LOCK angle trajectory did not settle");
        gimbal.setMode(apm_bridge::GimbalMode::AIM);
        drive(2000);
        const auto stale_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
        while (std::chrono::steady_clock::now() < stale_deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        simulator.service();
        require(simulator.control.gbc[1].op_type == 2 && simulator.control.gbc[1].op_value == 0,
                "Serial stale feedback did not stop angle trajectory");
        gimbal.stop();
        std::cout << "PASS: serial FPV/LOCK angle transitions are slew-limited; stale feedback stops motion\n";
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
        xfrobot::Gcu2GbcPkt_t latest_control{};
        int16_t pitch_encoder = 2000; // 相机低头 20° (FLU Ry(+20°)) → 编码器低头正 +20°
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
                if (packet.cmd.value == 4)
                    latest_control = packet;
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
                reply.mtr_angle[0] = pitch_encoder;
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
        const tf2::Quaternion roll(tf2::Vector3(1, 0, 0), 10 * M_PI / 180);
        const tf2::Quaternion pitch(tf2::Vector3(0, 1, 0), 20 * M_PI / 180);
        require(std::abs(actual.dot(heading * yaw * roll * pitch)) > .99999, "hardware encoder ENU/FLU orientation");
        require(msg.header.frame_id == "base_link", "hardware frame changed");
        double relative_roll, relative_pitch, relative_yaw;
        tf2::Matrix3x3(yaw * roll * pitch).getRPY(relative_roll, relative_pitch, relative_yaw);
        require(std::abs(msg.orientation_covariance[0] - relative_roll) < 1e-6 &&
                std::abs(msg.orientation_covariance[1] - relative_pitch) < 1e-6 &&
                std::abs(msg.orientation_covariance[2] - relative_yaw) < 1e-6,
                "hardware legacy Euler slots do not match reconstructed orientation");
        require(std::abs(msg.angular_velocity.x - 2 * M_PI / 180) < 1e-6 &&
                std::abs(msg.angular_velocity.y - M_PI / 180) < 1e-6 &&
                std::abs(msg.angular_velocity.z + 3 * M_PI / 180) < 1e-6,
                "hardware FRD -> FLU gyro conversion");
        gimbal->setMode(apm_bridge::GimbalMode::AIM);
        drive(std::chrono::milliseconds(4000), true);
        for (const auto &axis : latest_control.gbc)
            require(axis.wk_mode == 2 && axis.op_type == 0, "AIM is not FPV angle control");
        require(latest_control.gbc[0].op_value == 0 && latest_control.gbc[1].op_value == 4000 &&
                latest_control.gbc[2].op_value == 0, "AIM body +Z target did not respect default pitch limit");
        gimbal->setMode(apm_bridge::GimbalMode::MISSION);
        gimbal->updateTargetLos({true, .2, -.3, .05, -.07});
        drive(std::chrono::milliseconds(200), true);
        require(latest_control.gbc[1].op_value < 0 && latest_control.gbc[2].op_value > 0,
                "MISSION FLU -> vendor pitch/yaw signs");
        for (const auto &axis : latest_control.gbc)
            require(axis.wk_mode == 1 && axis.op_type == 2 && std::abs(axis.op_value) <= 300,
                    "MISSION did not use limited locked real rates");
        pitch_encoder = 13500; // 姿态 -135° (下限) → 编码器 +135° (低头正)
        gimbal->updateTargetLos({true, .2, 0, 0, 0});
        drive(std::chrono::milliseconds(100), true);
        require(latest_control.gbc[1].op_value == 0, "MISSION pushed outward at lower pitch limit");
        gimbal->updateTargetLos({true, -.2, 0, 0, 0});
        drive(std::chrono::milliseconds(250), true);
        require(latest_control.gbc[1].op_value > 0, "MISSION could not return inward from lower pitch limit");
        pitch_encoder = -4000; // 姿态 +40° (上限) → 编码器 -40°
        gimbal->updateTargetLos({true, -.2, 0, 0, 0});
        drive(std::chrono::milliseconds(100), true);
        require(latest_control.gbc[1].op_value == 0, "MISSION pushed outward at upper pitch limit");
        gimbal->updateTargetLos({true, .2, 0, 0, 0});
        drive(std::chrono::milliseconds(250), true);
        require(latest_control.gbc[1].op_value < 0, "MISSION could not return inward from upper pitch limit");
        pitch_encoder = 2000;
        drive(std::chrono::milliseconds(1300), true);
        require(std::abs(latest_control.gbc[1].op_value) <= 1 && std::abs(latest_control.gbc[2].op_value) <= 1,
                "MISSION stale target did not decelerate to hold");
        gimbal->setMode(apm_bridge::GimbalMode::HORIZON);
        drive(std::chrono::milliseconds(100), true);
        require(latest_control.gbc[1].wk_mode == 1 && latest_control.gbc[1].op_value > 0,
                "HORIZON did not restore stabilization");
        gimbal->setMode(apm_bridge::GimbalMode::MISSION);
        gimbal->updateTargetLos({true, .2, -.3, 0, 0});
        drive(std::chrono::milliseconds(100), true);
        const auto no_feedback_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(350);
        while (std::chrono::steady_clock::now() < no_feedback_deadline)
        {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        const auto no_feedback_count = capture.count();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        rclcpp::spin_some(node);
        require(capture.count() == no_feedback_count, "stale hardware feedback fabricated a fresh gimbal pose");
        gimbal->setMode(apm_bridge::GimbalMode::HORIZON);
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
        struct Position
        {
            double x_val = 0, y_val = 0, z_val = 0;
            MSGPACK_DEFINE_MAP(x_val, y_val, z_val);
        } position;
        RpcOrientation orientation;
        MSGPACK_DEFINE_MAP(position, orientation);
    };
    struct RpcCamera
    {
        RpcPose pose;
        MSGPACK_DEFINE_MAP(pose);
    };

    void liveAirsimPose()
    {
        const auto *host_env = std::getenv("APM_BRIDGE_AIRSIM_HOST");
        rpc::client client(host_env ? host_env : "127.0.0.1", 41451);
        client.set_timeout(2000);
        const auto vehicles = client.call("listVehicles").get().as<std::vector<std::string>>();
        require(!vehicles.empty(), "Live AirSim has no vehicles");
        const auto *vehicle_env = std::getenv("APM_BRIDGE_AIRSIM_VEHICLE");
        const std::string vehicle_name = vehicle_env ? vehicle_env : vehicles.front();
        const auto *camera_env = std::getenv("APM_BRIDGE_AIRSIM_CAMERA");
        const std::string camera_name = camera_env ? camera_env : "0";
        const auto fromRpc = [](const RpcOrientation &q) {
            return tf2::Quaternion(q.x_val, q.y_val, q.z_val, q.w_val).normalized();
        };
        const tf2::Matrix3x3 ned_to_enu(0, 1, 0, 1, 0, 0, 0, 0, -1);
        const tf2::Matrix3x3 flu_to_frd(1, 0, 0, 0, -1, 0, 0, 0, -1);
        const auto toEnu = [&](const RpcOrientation &q) {
            tf2::Quaternion result;
            (ned_to_enu * tf2::Matrix3x3(fromRpc(q)) * flu_to_frd).getRotation(result);
            return result.normalized();
        };
        SerialGimbalSimulator simulator;
        Capture capture;
        auto node = std::make_shared<rclcpp::Node>("gimbal_live_airsim_pose_test");
        const auto *model_env = std::getenv("APM_BRIDGE_AIRSIM_MODEL");
        auto config = xfrobot::GimbalConfig::defaultsFor(xfrobot::gimbalModelFromString(model_env ? model_env : "C20S"));
        const apm_bridge::GimbalAxes axes{config.roll.present, config.pitch.present, config.yaw.present};
        xfrobot::GimbalControl serial(*node, simulator.pty.port, config,
            [&](auto msg) { capture.receive(std::move(msg)); });
        serial.setMode(apm_bridge::GimbalMode::AIM);
        serial.start();
        double max_error = 0;
        for (int sample = 0; sample < 20; ++sample)
        {
            const auto info = client.call("simGetCameraInfo", camera_name, vehicle_name).get().as<RpcCamera>();
            const auto pose = client.call("simGetVehiclePose", vehicle_name).get().as<RpcPose>();
            const auto body = toEnu(pose.orientation);
            const auto camera = toEnu(info.pose.orientation);
            const auto joints = apm_bridge::gimbalJointAngles(body.inverse() * camera, axes);
            simulator.reply.mtr_angle[0] = std::lround(-joints[1] * 18000 / M_PI); // 编码器低头正
            simulator.reply.mtr_angle[1] = std::lround(joints[0] * 18000 / M_PI);
            simulator.reply.mtr_angle[2] = std::lround(joints[2] * 18000 / M_PI);
            auto imu = vehicle(0);
            imu.orientation = tf2::toMsg(body);
            const auto previous = capture.count();
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(40);
            while (std::chrono::steady_clock::now() < deadline)
            {
                serial.updateVehicleState(imu, 9.80665);
                simulator.service();
                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            require(capture.count() > previous, "Live AirSim-to-serial reconstruction callback missing");
            tf2::Quaternion actual;
            tf2::fromMsg(capture.last().orientation, actual);
            const auto difference = camera.inverse() * actual;
            const double error = 2 * std::atan2(tf2::Vector3(difference.x(), difference.y(), difference.z()).length(),
                                               std::abs(difference.w()));
            max_error = std::max(max_error, error);
        }
        serial.stop();
        std::cout << "Live AirSim vehicle=" << vehicle_name << " camera=" << camera_name
                  << " samples=20 max serial reconstruction error=" << max_error * 180 / M_PI << " deg\n";
        require(max_error < .1 * M_PI / 180, "Live AirSim camera does not match selected axes/mount or sampling is asynchronous");
        std::cout << "PASS: live AirSim ground truth -> virtual encoder -> serial reconstruction (read-only RPC)\n";
    }

    void airsimTransitionRates()
    {
        rpc::server server("127.0.0.1", 0);
        std::mutex mutex;
        tf2::Quaternion relative(0, 0, 0, 1);
        const tf2::Quaternion body(0, 0, 0, 1);
        const auto toRpc = [](const tf2::Quaternion &q) {
            return RpcOrientation{q.x(), q.y(), q.z(), q.w()};
        };
        server.bind("simGetCameraInfo", [&](const std::string &, const std::string &) {
            std::lock_guard<std::mutex> lock(mutex);
            RpcCamera result;
            result.pose.orientation = toRpc(body * relative);
            return result;
        });
        server.bind("simGetVehiclePose", [&](const std::string &) {
            RpcPose result;
            result.orientation = toRpc(body);
            return result;
        });
        server.bind("simSetCameraPose", [&](const std::string &, const RpcPose &pose, const std::string &) {
            std::lock_guard<std::mutex> lock(mutex);
            relative = tf2::Quaternion(pose.orientation.x_val, pose.orientation.y_val,
                                      pose.orientation.z_val, pose.orientation.w_val);
        });
        server.async_run(1);
        Capture capture;
        std::mutex samples_mutex;
        std::vector<sensor_msgs::msg::Imu> samples;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("airsim.enabled", true),
                                     rclcpp::Parameter("airsim.port", static_cast<int64_t>(server.port())),
                                     rclcpp::Parameter("gimbal.model", "C20S"),
                                     rclcpp::Parameter("gimbal.pitch_max", 120.0),
                                     rclcpp::Parameter("gimbal.pitch_aim", -10.0),
                                     rclcpp::Parameter("gimbal.tracking_max_rate", 140.0),
                                     rclcpp::Parameter("gimbal.tracking_max_accel", 600.0)});
        auto node = std::make_shared<rclcpp::Node>("airsim_transition_rates_test", options);
        node->declare_parameter<bool>("airsim.enabled", false);
        auto gimbal = apm_bridge::createGimbal(*node, [&](auto msg) {
            capture.receive(std::make_unique<sensor_msgs::msg::Imu>(*msg));
            std::lock_guard<std::mutex> lock(samples_mutex);
            samples.push_back(*msg);
        });
        gimbal->setMode(apm_bridge::GimbalMode::AIM);
        gimbal->start();
        auto waitPitch = [&](double degrees) {
            const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (std::chrono::steady_clock::now() < end)
            {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (std::abs(apm_bridge::gimbalJointAngles(
                            tf2::Quaternion(relative.x(), -relative.y(), -relative.z(), relative.w()),
                            {false, true, false})[1] * 180 / M_PI - degrees) < .1)
                        return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            throw std::runtime_error("AirSim transition did not converge");
        };
        waitPitch(80);
        gimbal->setMode(apm_bridge::GimbalMode::MISSION);
        gimbal->updateTargetLos({true, 0, 0, 0, 0});
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        gimbal->setMode(apm_bridge::GimbalMode::HORIZON);
        waitPitch(0);
        gimbal->setMode(apm_bridge::GimbalMode::AIM);
        waitPitch(80);
        gimbal->stop();
        server.stop();
        double peak_rate = 0, peak_pose_rate = 0;
        for (size_t i = 1; i < samples.size(); ++i)
        {
            const auto &msg = samples[i];
            peak_rate = std::max(peak_rate, std::abs(msg.angular_velocity.y) * 180 / M_PI);
            tf2::Quaternion a, b;
            tf2::fromMsg(samples[i - 1].orientation, a);
            tf2::fromMsg(msg.orientation, b);
            const double dt = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(samples[i - 1].header.stamp)).seconds();
            if (dt > 1e-6)
                peak_pose_rate = std::max(peak_pose_rate, a.angleShortestPath(b) / dt * 180 / M_PI);
        }
        std::cout << "AirSim transition gyro peak=" << peak_rate << " pose-derived peak=" << peak_pose_rate << " deg/s\n";
        require(peak_rate <= 145 && peak_pose_rate <= 145,
                "AirSim mode switch teleported camera pose or generated an unbounded gyro spike");
    }

    void airsimBackend()
    {
        rpc::server server("127.0.0.1", 0);
        std::mutex rpc_mutex;
        tf2::Quaternion body_ned(0, 0, 0, 1), camera_relative(0, 0, 0, 1);
        const tf2::Vector3 mount(.3, 0, -.2);
        size_t writes = 0;
        bool apply_commands = true;
        const auto toRpc = [](const tf2::Quaternion &q) {
            return RpcOrientation{q.x(), q.y(), q.z(), q.w()};
        };
        server.bind("simGetCameraInfo", [&](const std::string &camera, const std::string &vehicle) {
            require(camera == "0" && vehicle == "TestVehicle", "AirSim endpoint parameters changed");
            std::lock_guard<std::mutex> lock(rpc_mutex);
            RpcCamera result;
            result.pose.orientation = toRpc(body_ned * camera_relative);
            const auto position = tf2::quatRotate(body_ned, mount);
            result.pose.position = {position.x(), position.y(), position.z()};
            return result;
        });
        server.bind("simGetVehiclePose", [&](const std::string &vehicle) {
            require(vehicle == "TestVehicle", "AirSim vehicle pose endpoint");
            std::lock_guard<std::mutex> lock(rpc_mutex);
            RpcPose result;
            result.orientation = toRpc(body_ned);
            return result;
        });
        server.bind("simSetCameraPose", [&](const std::string &camera, const RpcPose &pose, const std::string &vehicle) {
            require(camera == "0" && vehicle == "TestVehicle", "AirSim camera control endpoint");
            require(std::abs(pose.position.x_val - mount.x()) < 1e-6 &&
                    std::abs(pose.position.y_val - mount.y()) < 1e-6 &&
                    std::abs(pose.position.z_val - mount.z()) < 1e-6, "AirSim camera mount offset changed");
            std::lock_guard<std::mutex> lock(rpc_mutex);
            if (apply_commands)
                camera_relative = tf2::Quaternion(pose.orientation.x_val, pose.orientation.y_val,
                                                  pose.orientation.z_val, pose.orientation.w_val);
            ++writes;
        });
        server.async_run(1);
        Capture capture;
        rclcpp::NodeOptions options;
        options.parameter_overrides({rclcpp::Parameter("airsim.enabled", true),
                                     rclcpp::Parameter("airsim.port", static_cast<int64_t>(server.port())),
                                     rclcpp::Parameter("airsim.vehicle_name", "TestVehicle"),
                                     rclcpp::Parameter("gimbal.pitch_aim", 10.0),
                                     rclcpp::Parameter("gimbal.pitch_max", 120.0),
                                     rclcpp::Parameter("gimbal.pitch_hold", 12.0),
                                     rclcpp::Parameter("gimbal.tracking_max_rate", 140.0),
                                     rclcpp::Parameter("gimbal.tracking_max_accel", 600.0),
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
        {
            std::lock_guard<std::mutex> lock(rpc_mutex);
            body_ned.setRPY(.1, .15, .4);
        }
        gimbal->setMode(apm_bridge::GimbalMode::AIM);
        std::this_thread::sleep_for(std::chrono::milliseconds(1800));
        {
            std::lock_guard<std::mutex> lock(rpc_mutex);
            const tf2::Quaternion desired(tf2::Vector3(0, 1, 0), 100 * M_PI / 180);
            require(writes > 3 && std::abs(camera_relative.dot(desired)) > .999999,
                    "AirSim AIM did not preserve body +Z plus pitch_aim beyond 90 degrees");
        }
        tf2::Quaternion camera_enu;
        tf2::fromMsg(capture.last().orientation, camera_enu);
        const double target_pitch = .4;
        const double target_yaw = -.3;
        const auto world_target = tf2::quatRotate(camera_enu, tf2::Vector3(
            std::cos(target_pitch) * std::cos(target_yaw),
            std::cos(target_pitch) * std::sin(target_yaw), -std::sin(target_pitch)));
        gimbal->setMode(apm_bridge::GimbalMode::MISSION);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        double target_error_pitch = 0, target_error_yaw = 0;
        while (std::chrono::steady_clock::now() < deadline)
        {
            tf2::fromMsg(capture.last().orientation, camera_enu);
            const auto ray = tf2::quatRotate(camera_enu.inverse(), world_target);
            target_error_pitch = std::atan2(-ray.z(), std::hypot(ray.x(), ray.y()));
            target_error_yaw = std::atan2(ray.y(), ray.x());
            gimbal->updateTargetLos({true, target_error_pitch, target_error_yaw, 0, 0});
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        require(std::abs(target_error_pitch) < .03 && std::abs(target_error_yaw) < .03,
                "AirSim MISSION did not center target independently of HORIZON pitch_hold");
        gimbal->updateTargetLos({});
        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
        const auto held = capture.last();
        require(std::abs(held.angular_velocity.y) < .01 && std::abs(held.angular_velocity.z) < .01,
                "AirSim invalid LOS did not stop tracking motion");
        gimbal->setMode(apm_bridge::GimbalMode::HORIZON);
        std::this_thread::sleep_for(std::chrono::milliseconds(1800));
        tf2::fromMsg(capture.last().orientation, camera_enu);
        double roll, pitch, yaw;
        tf2::Matrix3x3(camera_enu).getRPY(roll, pitch, yaw);
        require(std::abs(roll) < 1e-6 && std::abs(pitch + 12 * M_PI / 180) < 1e-6,
                "AirSim HORIZON does not hold world pitch");
        gimbal->stop();
        gimbal->stop();
        const auto stopped = capture.count();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        require(capture.count() == stopped, "AirSim callback survived stop");
        gimbal->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        require(capture.count() > stopped, "AirSim backend did not restart");
        gimbal->stop();

        Capture pole_capture;
        options.parameter_overrides({rclcpp::Parameter("airsim.enabled", true),
                                     rclcpp::Parameter("airsim.port", static_cast<int64_t>(server.port())),
                                     rclcpp::Parameter("airsim.vehicle_name", "TestVehicle"),
                                     rclcpp::Parameter("gimbal.model", "C20S"),
                                     rclcpp::Parameter("gimbal.pitch_aim", 10.0),
                                     rclcpp::Parameter("gimbal.pitch_max", 120.0),
                                     rclcpp::Parameter("gimbal.pitch_hold", -7.0),
                                     rclcpp::Parameter("gimbal.tracking_max_rate", 140.0),
                                     rclcpp::Parameter("gimbal.tracking_max_accel", 600.0)});
        auto pole_node = std::make_shared<rclcpp::Node>("gimbal_c20s_pole_test", options);
        pole_node->declare_parameter<bool>("airsim.enabled", false);
        auto pole_gimbal = apm_bridge::createGimbal(*pole_node,
            [&](auto msg) { pole_capture.receive(std::move(msg)); });
        pole_gimbal->setMode(apm_bridge::GimbalMode::AIM);
        pole_gimbal->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(1800));
        tf2::Quaternion pole_camera;
        tf2::fromMsg(pole_capture.last().orientation, pole_camera);
        const auto pole_target = tf2::quatRotate(pole_camera, tf2::Vector3(std::cos(.2), 0, -std::sin(.2)));
        pole_gimbal->setMode(apm_bridge::GimbalMode::MISSION);
        const auto pole_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
        double pole_error = 0;
        while (std::chrono::steady_clock::now() < pole_deadline)
        {
            tf2::fromMsg(pole_capture.last().orientation, pole_camera);
            const auto ray = tf2::quatRotate(pole_camera.inverse(), pole_target);
            pole_error = std::atan2(-ray.z(), std::hypot(ray.x(), ray.y()));
            pole_gimbal->updateTargetLos({true, pole_error, 0, 0, 0});
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        require(std::abs(pole_error) < .03, "C20S body-Z AIM falsely triggered inverted-roll protection");
        pole_gimbal->stop();

        const tf2::Matrix3x3 ned_to_enu(0, 1, 0, 1, 0, 0, 0, 0, -1);
        const tf2::Matrix3x3 flu_to_frd(1, 0, 0, 0, -1, 0, 0, 0, -1);
        {
            std::lock_guard<std::mutex> lock(rpc_mutex);
            apply_commands = false;
        }
        for (double joint_pitch : {75.0, 89.0, 90.0, 91.0, 110.0})
        {
            Capture reconstructed;
            options.parameter_overrides({rclcpp::Parameter("airsim.enabled", true),
                                         rclcpp::Parameter("airsim.port", static_cast<int64_t>(server.port())),
                                         rclcpp::Parameter("airsim.vehicle_name", "TestVehicle"),
                                         rclcpp::Parameter("gimbal.model", "C20S"),
                                         rclcpp::Parameter("gimbal.pitch_aim", joint_pitch - 90),
                                         rclcpp::Parameter("gimbal.pitch_max", 120.0)});
            auto mixed_node = std::make_shared<rclcpp::Node>("gimbal_c20s_reconstruction_test", options);
            mixed_node->declare_parameter<bool>("airsim.enabled", false);
            auto mixed_gimbal = apm_bridge::createGimbal(*mixed_node,
                [&](auto msg) { reconstructed.receive(std::move(msg)); });
            mixed_gimbal->setMode(apm_bridge::GimbalMode::AIM);
            mixed_gimbal->start();
            for (const auto &attitude : {std::array<double, 3>{.3, .4, 1.1},
                                        std::array<double, 3>{-.5, -.25, -2.0}})
            {
                {
                    std::lock_guard<std::mutex> lock(rpc_mutex);
                    body_ned.setRPY(attitude[0], attitude[1], attitude[2]);
                    camera_relative.setRotation(tf2::Vector3(0, 1, 0), joint_pitch * M_PI / 180);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(120));
                require(reconstructed.count() > 0, "AirSim C20S reconstruction callback missing");
                tf2::Quaternion measured;
                tf2::fromMsg(reconstructed.last().orientation, measured);
                std::lock_guard<std::mutex> lock(rpc_mutex);
                const auto expected = ned_to_enu * tf2::Matrix3x3(body_ned) *
                    tf2::Matrix3x3(camera_relative) * flu_to_frd;
                const auto actual = tf2::Matrix3x3(measured);
                for (int axis = 0; axis < 3; ++axis)
                    require((actual.getColumn(axis) - expected.getColumn(axis)).length() < 1e-6,
                            "AirSim C20S reconstructed pose differs from independent simulator ground truth");
            }
            mixed_gimbal->stop();
        }
        {
            std::lock_guard<std::mutex> lock(rpc_mutex);
            apply_commands = false;
        }
        for (const auto &model : {std::string("C20S"), std::string("C40D"), std::string("C200T")})
        {
            Capture reconstructed;
            options.parameter_overrides({rclcpp::Parameter("airsim.enabled", true),
                                         rclcpp::Parameter("airsim.port", static_cast<int64_t>(server.port())),
                                         rclcpp::Parameter("airsim.vehicle_name", "TestVehicle"),
                                         rclcpp::Parameter("gimbal.model", model)});
            auto model_node = std::make_shared<rclcpp::Node>("gimbal_all_model_pose_test", options);
            model_node->declare_parameter<bool>("airsim.enabled", false);
            auto model_gimbal = apm_bridge::createGimbal(*model_node,
                [&](auto msg) { reconstructed.receive(std::move(msg)); });
            for (double sign : {-1.0, 1.0})
            {
                {
                    std::lock_guard<std::mutex> lock(rpc_mutex);
                    body_ned.setRPY(.4 * sign, -.3, 1.1 * sign);
                    const double roll = model == "C20S" ? 0 : .4 * sign;
                    const double yaw = model == "C200T" ? -.7 * sign : 0;
                    const auto relative_flu = jointRotation(roll, sign * 110 * M_PI / 180, yaw);
                    (flu_to_frd * relative_flu * flu_to_frd).getRotation(camera_relative);
                }
                model_gimbal->start();
                std::this_thread::sleep_for(std::chrono::milliseconds(120));
                require(reconstructed.count() > 0, "All-model AirSim callback missing");
                tf2::Quaternion actual;
                tf2::fromMsg(reconstructed.last().orientation, actual);
                {
                    std::lock_guard<std::mutex> lock(rpc_mutex);
                    const auto expected = ned_to_enu * tf2::Matrix3x3(body_ned * camera_relative) * flu_to_frd;
                    for (int axis = 0; axis < 3; ++axis)
                        require((tf2::Matrix3x3(actual).getColumn(axis) - expected.getColumn(axis)).length() < 1e-6,
                                "AirSim model reconstruction differs from independent simulator ground truth");
                }
                model_gimbal->stop();
            }
        }
        server.stop();
        std::cout << "PASS: all-model AirSim encoder reconstruction, lifecycle, mount and AIM/MISSION/HORIZON tracking\n";
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
#ifdef APM_BRIDGE_AIRSIM
        if (argc > 1 && std::string(argv[1]) == "--airsim-live")
        {
            liveAirsimPose();
            rclcpp::shutdown();
            return 0;
        }
#endif
        factoryDisabledAndErrors();
        aimAndMissionLimits();
        strapdownBackend();
        jointKinematics();
        for (const auto model : {xfrobot::GimbalModel::C20S, xfrobot::GimbalModel::C40D, xfrobot::GimbalModel::C200T})
            hardwarePose(model);
        hardwareBackend();
        serialAngleTransitions();
#ifdef APM_BRIDGE_AIRSIM
        airsimTransitionRates();
#endif
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
