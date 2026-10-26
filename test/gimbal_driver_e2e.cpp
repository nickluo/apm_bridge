// 云台真机驱动端到端验收: 用修复后的 GimbalControl 完整闭环 (模式切换/AIM/HORIZON
// 命令/轨迹限速//fpv/gimbal 位姿发布) 对真实云台运行, 断言发布位姿的物理方向。
//
// 用法: gimbal_driver_e2e <port> [pitch_aim_deg]
// 前提: 云台水平放置 (载机 IMU 喂水平零姿态)。
// 判定: AIM 后发布俯仰 <= -80° (抬头, FLU 俯仰低头正); HORIZON 回 |pitch| <= 6°。

#include "gimbal.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace xfrobot;

namespace
{
struct Capture
{
    std::mutex mtx;
    sensor_msgs::msg::Imu last;
    std::atomic<unsigned> count{0};
    void receive(sensor_msgs::msg::Imu::UniquePtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        last = *msg;
        count++;
    }
};

int failures = 0;
void require(bool ok, const char *what)
{
    std::printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        failures++;
}

// 最近 window 秒发布俯仰的均值 (deg, FLU 低头正)
double meanPitch(Capture &capture, std::chrono::steady_clock::time_point since)
{
    std::lock_guard<std::mutex> lock(capture.mtx);
    (void)since; // 单缓存, 取当前值即可 (各阶段末已稳定)
    double r, p, y;
    tf2::Matrix3x3(tf2::Quaternion(capture.last.orientation.x, capture.last.orientation.y,
                     capture.last.orientation.z, capture.last.orientation.w))
        .getRPY(r, p, y);
    return p * 180.0 / M_PI;
}
} // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::fprintf(stderr, "usage: %s <port> [pitch_aim_deg]\n", argv[0]);
        return 2;
    }
    const std::string port = argv[1];
    const double pitch_aim = argc > 2 ? std::atof(argv[2]) : 0.0;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gimbal_driver_e2e");

    GimbalConfig cfg = GimbalConfig::defaultsFor(GimbalModel::C20S);
    cfg.pitch.min_deg = -120;
    cfg.pitch.max_deg = 120;
    cfg.pitch_aim_deg = static_cast<float>(pitch_aim);
    cfg.tracking.max_rate_dps = 90.0;
    cfg.tracking.max_accel_dps2 = 300.0;

    Capture capture;
    GimbalControl gimbal(*node, port, cfg,
        [&capture](sensor_msgs::msg::Imu::UniquePtr msg) { capture.receive(std::move(msg)); });

    // 水平悬停比力: FLU z = +1g
    sensor_msgs::msg::Imu imu;
    imu.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
    imu.linear_acceleration.x = 0;
    imu.linear_acceleration.y = 0;
    imu.linear_acceleration.z = 9.80665;

    gimbal.setMode(apm_bridge::GimbalMode::HORIZON);
    gimbal.start();
    if (!gimbal.is_running())
    {
        std::fprintf(stderr, "failed to open %s\n", port.c_str());
        return 1;
    }

    const auto drive = [&](double seconds, apm_bridge::GimbalMode mode) {
        gimbal.setMode(mode);
        const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < end)
        {
            gimbal.updateVehicleState(imu, 9.80665);
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    };

    std::printf("== 阶段1: HORIZON 水平保持 (6s) ==\n");
    drive(6.0, apm_bridge::GimbalMode::HORIZON);
    const double p1 = meanPitch(capture, {});
    std::printf("   发布俯仰 = %+.2f deg (期望 |p| <= 6)\n", p1);
    require(capture.count.load() > 50, "HORIZON 期间持续发布相机位姿");
    require(std::abs(p1) <= 6.0, "HORIZON 回平 (允许水平基准漂移)");
    {
        std::lock_guard<std::mutex> lock(capture.mtx);
        require(capture.last.header.frame_id == "base_link", "发布 frame_id 为 base_link");
    }

    std::printf("== 阶段2: AIM 上视 %.1f deg (12s) ==\n", 90.0 + pitch_aim);
    drive(12.0, apm_bridge::GimbalMode::AIM);
    const double p2 = meanPitch(capture, {});
    std::printf("   发布俯仰 = %+.2f deg (期望 <= -80, 抬头)\n", p2);
    require(p2 <= -80.0, "AIM 后相机位姿为上视 (编码器反号修复生效)");
    {
        std::lock_guard<std::mutex> lock(capture.mtx);
        require(capture.last.orientation_covariance[1] * 180.0 / M_PI <= -80.0,
                "AIM 遗留欧拉槽 (heading-relative) 同为上视");
    }

    std::printf("== 阶段3: 回 HORIZON (10s) ==\n");
    drive(10.0, apm_bridge::GimbalMode::HORIZON);
    const double p3 = meanPitch(capture, {});
    std::printf("   发布俯仰 = %+.2f deg (期望 |p| <= 6)\n", p3);
    require(std::abs(p3) <= 6.0, "HORIZON 从上视回平 (域切换连续无跳变发散)");

    gimbal.stop();
    rclcpp::shutdown();
    std::printf("%s\n", failures == 0 ? "E2E ALL PASS" : "E2E FAILED");
    return failures == 0 ? 0 : 1;
}
