#include "custom_link_vehicle_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include "custom_link/sitl_transport.h"

using namespace custom_link_bridge;
using namespace std::chrono_literals;

namespace
{
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;
    constexpr double kSeaLevelPressurePa = 101325.0;
    constexpr double kReferenceTemperatureKelvin = 273.15;
    constexpr int kAbsoluteZeroCdeg = -27315;

    double normalizeAngle(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a <= -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

} // namespace

CustomLinkVehicleNode::CustomLinkVehicleNode()
    : Node("custom_link_bridge")
{
    // ---------------- 参数 ----------------
    this->declare_parameter<double>("gravity_const", 9.80665);
    this->declare_parameter<double>("mass", 2.0);
    this->declare_parameter<int>("lipo_cells", 6);
    this->declare_parameter<double>("battery.capacity_mah", 13000.0); // 整包容量
    this->declare_parameter<int>("channel_trigger", 6);
    this->declare_parameter<bool>("voltage_compensation", false);
    this->declare_parameter<bool>("baro_compensation", true);
    this->declare_parameter<double>("motor_parameters.A", 0.0);
    this->declare_parameter<double>("motor_parameters.B", 0.0);
    this->declare_parameter<double>("motor_parameters.C", 0.0);
    this->declare_parameter<double>("motor_parameters.D", 0.0);
    this->declare_parameter<int>("motor_parameters.n", 1);
    this->declare_parameter<double>("motor_parameters.thrust_limit", 1.0);
    this->declare_parameter<double>("motor_parameters.spin_k", 0.0);
    this->declare_parameter<double>("motor_parameters.vbat_a", 0.0);
    this->declare_parameter<double>("motor_parameters.vbat_b", 1.0);
    airsim_enabled = this->declare_parameter<bool>("airsim.enabled", false);

    this->declare_parameter<std::string>("link.transport", "serial"); // serial | tcp
    this->declare_parameter<std::string>("link.serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("link.baudrate", 921600);
    this->declare_parameter<std::string>("link.tcp_host", "127.0.0.1");
    this->declare_parameter<int>("link.tcp_port", 5763); // SITL: UART3 = 5760 + uart_index
    this->declare_parameter<double>("link.control_rate", 100.0);
    this->declare_parameter<double>("link.sync_rate", 1.0);
    this->declare_parameter<double>("link.diag_rate", 0.2);
    this->declare_parameter<double>("link.rate_limit_dps", 0.0);

    this->declare_parameter<std::string>("imu.frame_id", "base_link");
    this->declare_parameter<double>("imu.orientation_stddev", 0.0);
    this->declare_parameter<double>("imu.angular_velocity_stddev", 0.0);
    this->declare_parameter<double>("imu.linear_acceleration_stddev", 0.0);

    this->declare_parameter<double>("rc.control_timeout", 0.5);
    this->declare_parameter<double>("landed.airborne_altitude", 0.5);
    this->declare_parameter<double>("landed.landing_altitude", 0.2);
    this->declare_parameter<double>("landed.takeoff_confirm_time", 0.5);
    this->declare_parameter<double>("landed.landing_confirm_time", 1.0);

    this->get_parameter("gravity_const", gravity);
    this->get_parameter("mass", mass);
    this->get_parameter("lipo_cells", n_lipo_cells);
    this->get_parameter("battery.capacity_mah", battery_capacity_mah);
    this->get_parameter("channel_trigger", channel_trigger);
    this->get_parameter("voltage_compensation", voltage_compensation);
    this->get_parameter("baro_compensation", baro_compensation);
    if (baro_compensation)
        kp = kReferenceTemperatureKelvin / (kReferenceTemperatureKelvin + baro_temperature_c);
    this->get_parameter("motor_parameters.A", motor_params.A);
    this->get_parameter("motor_parameters.B", motor_params.B);
    this->get_parameter("motor_parameters.C", motor_params.C);
    this->get_parameter("motor_parameters.D", motor_params.D);
    this->get_parameter("motor_parameters.n", motor_params.n_motors);
    this->get_parameter("motor_parameters.thrust_limit", motor_params.thrust_limit);
    this->get_parameter("motor_parameters.spin_k", motor_params.spin_k);
    this->get_parameter("motor_parameters.vbat_a", motor_params.vbat_a);
    this->get_parameter("motor_parameters.vbat_b", motor_params.vbat_b);
    motor_params.volt_max = n_lipo_cells * kBatteryFullVoltagePerCell;
    motor_params.volt_ref = n_lipo_cells * kBatteryNominalVoltagePerCell;

    double ori_stddev = 0.0, gyro_stddev = 0.0, acc_stddev = 0.0;
    this->get_parameter("imu.orientation_stddev", ori_stddev);
    this->get_parameter("imu.angular_velocity_stddev", gyro_stddev);
    this->get_parameter("imu.linear_acceleration_stddev", acc_stddev);
    orientation_variance = ori_stddev * ori_stddev;
    angular_velocity_variance = gyro_stddev * gyro_stddev;
    linear_acceleration_variance = acc_stddev * acc_stddev;
    this->get_parameter("imu.frame_id", imu_frame_id);
    this->get_parameter("rc.control_timeout", control_timeout);
    this->get_parameter("landed.airborne_altitude", airborne_altitude);
    this->get_parameter("landed.landing_altitude", landing_altitude);
    this->get_parameter("landed.takeoff_confirm_time", takeoff_confirm_time_s);
    this->get_parameter("landed.landing_confirm_time", landing_confirm_time_s);
    if (!std::isfinite(airborne_altitude) || !std::isfinite(landing_altitude) ||
        !std::isfinite(takeoff_confirm_time_s) || !std::isfinite(landing_confirm_time_s) ||
        airborne_altitude <= 0.0 || landing_altitude < 0.0 || landing_altitude >= airborne_altitude ||
        takeoff_confirm_time_s <= 0.0 || landing_confirm_time_s <= 0.0)
        throw std::invalid_argument("Invalid landed-state detection parameters");
    if (airsim_enabled && (voltage_compensation || baro_compensation))
        throw std::invalid_argument("AirSim SITL requires voltage_compensation=false and baro_compensation=false");
    if (airsim_enabled &&
        (!std::isfinite(mass) || mass <= 0 || !std::isfinite(gravity) || gravity <= 0 ||
         !std::isfinite(control_timeout) || control_timeout <= 0 || motor_params.n_motors <= 0 ||
         !std::isfinite(motor_params.thrust_limit) || motor_params.thrust_limit <= 0 ||
         motor_params.thrust_limit > 1 ||
         !std::isfinite(motor_params.A) || !std::isfinite(motor_params.B) ||
         !std::isfinite(motor_params.C) || !std::isfinite(motor_params.D)))
        throw std::invalid_argument("Invalid SITL mass, gravity, timeout or motor model");

    RCLCPP_INFO(this->get_logger(),
                "Motor parameters: A=%.6f, B=%.6f, C=%.6f, D=%.6f, n=%d, voltage_max=%.6f, spin_k=%.6f, vbat_a=%.6f, vbat_b=%.6f, volt_ref=%.6f",
                motor_params.A, motor_params.B, motor_params.C, motor_params.D,
                motor_params.n_motors, motor_params.volt_max, motor_params.spin_k,
                motor_params.vbat_a, motor_params.vbat_b, motor_params.volt_ref);

    // 反演曲线预拟合: thrustToForce 运行时以多项式求值替代牛顿迭代
    const double fit_residual = quadratic_thrust_model::initInverseFit(motor_params);
    if (fit_residual >= 0.0)
        RCLCPP_INFO(this->get_logger(),
                    "Thrust inverse fit: domain=[0, %.3f], %d cubic segments, max residual %.6f N (per-call Newton -> segment polyval)",
                    motor_params.inv_t_max, motor_params.inv_segments, fit_residual);
    else
        RCLCPP_WARN(this->get_logger(),
                    "Thrust inverse fit failed (degenerate samples?), thrustToForce falls back to Newton iteration");

    // ---------------- ROS 接口 ----------------
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "/fpv/odom", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    rc_in_pub = this->create_publisher<mavros_msgs::msg::RCIn>(
        "/mavros/rc/in", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    state_pub = this->create_publisher<mavros_msgs::msg::State>("/mavros/state", 10);
    ext_state_pub = this->create_publisher<mavros_msgs::msg::ExtendedState>("/mavros/extended_state", 10);
    battery_pub = this->create_publisher<sensor_msgs::msg::BatteryState>("/mavros/battery", 10);
    // 环境气压/温度 (0x11 携带)：对外统一 /fpv 话题，供推力标定等工具采集 kp
    pressure_pub = this->create_publisher<sensor_msgs::msg::FluidPressure>(
        "/fpv/static_pressure", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    temperature_pub = this->create_publisher<sensor_msgs::msg::Temperature>(
        "/fpv/temperature_baro", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    ap_feedback_pub = this->create_publisher<quadrotor_msgs::msg::LowLevelFeedback>("~/low_level_feedback", 10);
    gimbal_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/fpv/gimbal", 10);
    gimbal_mode_pub = this->create_publisher<std_msgs::msg::String>(
        "/fpv/gimbal_mode", rclcpp::QoS(1).transient_local());
    trigger_pub = this->create_publisher<std_msgs::msg::Header>("/fpv/tracker_trigger", 10);
    altitude_pub = this->create_publisher<std_msgs::msg::Float64>("/fpv/altitude", rclcpp::SensorDataQoS());
    // GPS / 失效保护 (0x12 PayloadSlow, 10 Hz)：沿用 MAVROS 话题名，OSD 与
    // 其他节点可直接复用 mavros 消费习惯
    gpsraw_pub = this->create_publisher<mavros_msgs::msg::GPSRAW>(
        "/mavros/gpsstatus/gpsraw", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    navsat_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/mavros/global_position/global", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    failsafe_pub = this->create_publisher<std_msgs::msg::Bool>("/fpv/failsafe", 10);

    control_command_sub = this->create_subscription<quadrotor_msgs::msg::ControlCommand>(
        "~/control_command", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&CustomLinkVehicleNode::ctrlCommandCallback, this, std::placeholders::_1));
    control_command_raw_sub = this->create_subscription<quadrotor_msgs::msg::ControlCommand>(
        "~/control_command_raw", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(),
        std::bind(&CustomLinkVehicleNode::ctrlCommandRawCallback, this, std::placeholders::_1));
    arm_interface_sub = this->create_subscription<std_msgs::msg::Bool>(
        "~/arm", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&CustomLinkVehicleNode::armCallback, this, std::placeholders::_1));
    offboard_interface_sub = this->create_subscription<std_msgs::msg::Bool>(
        "~/offboard", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&CustomLinkVehicleNode::offboardCallback, this, std::placeholders::_1));

    tracker_state_sub = this->create_subscription<std_msgs::msg::Int8>(
        "/fpv/tracker_state", 1,
        [&](const std_msgs::msg::Int8::SharedPtr msg) {
            in_tracking = (msg->data == 2); // TRACKING
        });
    radar_state_sub = this->create_subscription<std_msgs::msg::Int8>(
        "/fpv/radar_state", 1,
        [&](const std_msgs::msg::Int8::SharedPtr msg) {
            radar_tracking = (msg->data == 2); // TRACKING
        });
    mission_state_sub = this->create_subscription<std_msgs::msg::Bool>(
        "/fpv/mission_state", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            // 拦截器结束任务（True→False）同样联动退出 OFFBOARD（与 OFF 触发帧双保险）。
            if (in_mission.exchange(msg->data) && !msg->data && client)
            {
                client->setOffboardRequest(false);
                RCLCPP_INFO(this->get_logger(), "Mission ended: requesting custom-link MANUAL");
            }
        });

    mission_gimbal_on_ground = this->declare_parameter<bool>("gimbal.mission_on_ground", false);
    gimbal = apm_bridge::createGimbal(*this, [publisher = gimbal_imu_pub](auto msg) {
        publisher->publish(std::move(msg));
    });
    if (gimbal)
    {
        gimbal->setMode(apm_bridge::GimbalMode::AIM);
        gimbal->start();
    }
    if (!this->has_parameter("gimbal.los_timeout"))
        this->declare_parameter<double>("gimbal.los_timeout", 0.3);
    target_los_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/fpv/target_los", rclcpp::QoS(1).best_effort(),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            if (!gimbal)
                return;
            apm_bridge::TargetLos target;
            const auto &angles = msg->twist.linear;
            const auto &rates = msg->twist.angular;
            const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
            const int64_t now = this->now().nanoseconds();
            if (now < last_los_clock)
            {
                last_los_stamp = 0;
                RCLCPP_WARN(this->get_logger(), "ROS clock moved backwards; resetting target LOS sequence");
            }
            last_los_clock = now;
            const double age = (now - stamp) * 1e-9;
            target.valid = msg->header.frame_id == "camera_flu" &&
                std::isfinite(angles.y) && std::isfinite(angles.z) &&
                std::isfinite(rates.y) && std::isfinite(rates.z) &&
                std::abs(angles.y) < M_PI_2 - 1e-6 && std::abs(angles.z) < M_PI_2 - 1e-6 &&
                age >= 0 && age <= this->get_parameter("gimbal.los_timeout").as_double() &&
                stamp > last_los_stamp;
            if (target.valid)
            {
                last_los_stamp = stamp;
                target.pitch = angles.y;
                target.yaw = angles.z;
                target.pitch_rate = rates.y;
                target.yaw_rate = rates.z;
                target.age = age;
            }
            else
            {
                if (!msg->header.frame_id.empty())
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Rejecting target LOS: invalid frame, values, stale or non-increasing timestamp");
            }
            gimbal->updateTargetLos(target);
        });
    gimbal_trigger_sub = this->create_subscription<std_msgs::msg::Header>(
        "/fpv/tracker_trigger", 10, [this](const std_msgs::msg::Header::SharedPtr msg) {
            if (msg->frame_id == "ON")
                gimbal_aim_requested = true;
            else if (msg->frame_id == "OFF" || msg->frame_id == "DISMISS")
            {
                gimbal_abort_requested = true;
                // 退出 MISSION 联动：退出 OFFBOARD（modeReq=0），杆量权限即刻交还飞手。
                if (client)
                {
                    client->setOffboardRequest(false);
                    RCLCPP_INFO(this->get_logger(), "Tracker OFF: requesting custom-link MANUAL");
                }
            }
            else if (msg->frame_id == "MISSION")
            {
                // 进入 MISSION 联动：请求飞控进入 OFFBOARD（modeReq=2，HOST_CONTROL
                // 帧随控制流携带），拦截弹控制帧即刻获得权限，不再依赖 AUX2 开关。
                if (client)
                {
                    client->setOffboardRequest(true);
                    RCLCPP_INFO(this->get_logger(), "MISSION: requesting custom-link OFFBOARD");
                }
            }
        });
    std_msgs::msg::String initial_mode;
    initial_mode.data = "AIM";
    gimbal_mode_pub->publish(initial_mode);
    gimbal_mode_timer = this->create_wall_timer(20ms, [this]() { updateGimbalMode(); });

    // ---------------- 链路 ----------------
    using custom_link::Client;
    using custom_link::SerialTransport;
    using custom_link::TcpTransport;

    std::string transport_kind, serial_port, tcp_host;
    int baudrate = 921600, tcp_port = 5763;
    double control_rate = 100.0, sync_rate = 1.0, diag_rate = 0.2;
    this->get_parameter("link.transport", transport_kind);
    this->get_parameter("link.serial_port", serial_port);
    this->get_parameter("link.baudrate", baudrate);
    this->get_parameter("link.tcp_host", tcp_host);
    this->get_parameter("link.tcp_port", tcp_port);
    this->get_parameter("link.control_rate", control_rate);
    this->get_parameter("link.sync_rate", sync_rate);
    this->get_parameter("link.diag_rate", diag_rate);
    this->get_parameter("link.rate_limit_dps", rate_limit_dps);
    const auto control_period = custom_link::periodFromRate(control_rate);

    std::unique_ptr<custom_link::Transport> transport;
    if (airsim_enabled)
    {
        if (transport_kind != "tcp")
            throw std::invalid_argument("AirSim SITL requires link.transport=tcp");
        const auto bind_host = this->declare_parameter<std::string>("airsim.link_bind_host", "127.0.0.1");
        const int bind_port = this->declare_parameter<int>("airsim.link_port", 5764);
        if (bind_port <= 0 || bind_port > 65535 || tcp_port <= 0 || tcp_port > 65535 || bind_port == tcp_port)
            throw std::invalid_argument("AirSim relay and FC TCP ports must be valid and distinct");
        transport = std::make_unique<custom_link::SitlTransport>(
            tcp_host, static_cast<uint16_t>(tcp_port), bind_host, static_cast<uint16_t>(bind_port), this->get_logger());
    }
    else if (transport_kind == "tcp")
    {
        RCLCPP_INFO(this->get_logger(), "Custom link over TCP %s:%d", tcp_host.c_str(), tcp_port);
        transport = std::make_unique<TcpTransport>(tcp_host, static_cast<uint16_t>(tcp_port));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Custom link over serial %s @ %d", serial_port.c_str(), baudrate);
        transport = std::make_unique<SerialTransport>(serial_port, static_cast<uint32_t>(baudrate));
    }

    client = std::make_unique<Client>(std::move(transport));
    client->setRates(control_rate, sync_rate);

    client->setLinkStateCallback([this](bool up) {
        if (up)
            RCLCPP_INFO(this->get_logger(), "Custom link up");
        else
            RCLCPP_WARN(this->get_logger(), "Custom link down");
    });

    client->setTelemetryCallback(
        [this](const custom_link::protocol::Frame &frame, uint32_t fcUs, int64_t hostUs) {
            using namespace custom_link::protocol;
            switch (frame.msgId)
            {
            case MSG_FC_FAST:
            {
                PayloadFast p;
                if (frame.as(p))
                    handleFast(p, hostUs);
                break;
            }
            case MSG_FC_MEDIUM:
            {
                PayloadMedium p;
                if (frame.as(p))
                    handleMedium(p, hostUs);
                break;
            }
            case MSG_FC_SLOW:
            {
                PayloadSlow p;
                if (frame.as(p))
                    handleSlow(p, hostUs);
                break;
            }
            case MSG_FC_ODOMETRY:
            {
                PayloadOdometry p;
                if (frame.as(p))
                    handleOdometry(p, hostUs);
                else
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Invalid custom-link odometry payload size: %u", frame.len);
                break;
            }
            default:
                break;
            }
            (void)fcUs;
        });

    client->start();

    // 控制量推送：回调只写缓存，这里统一判新鲜度并转换体轴后下发。
    // 超时后只作废控制量，arm 请求不动（链路恢复后自动重试解锁边沿）。
    cmd_push_timer = this->create_wall_timer(control_period, [this]() { pushControl(); });

    diag_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(0.05, diag_rate))), [this]() {
            const auto sync = client->timeSync();
            const auto stats = client->stats();
            if (!sync.valid)
                return;
            RCLCPP_INFO(this->get_logger(),
                        "Custom link: rtt %.2f ms (jitter %.2f), skew %+.0f ppm, FC offset %+.2f ms, "
                        "rx %lu (F:%u M:%u S:%u O:%u), tx %lu, reconnects %lu",
                        sync.rtt_ms, sync.jitter_ms, sync.skew_ppm, sync.offset_ms,
                        static_cast<unsigned long>(stats.rx_frames),
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_FAST & 7u],
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_MEDIUM & 7u],
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_SLOW & 7u],
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_ODOMETRY & 7u],
                        static_cast<unsigned long>(stats.tx_frames),
                        static_cast<unsigned long>(stats.reconnects));
        });

}

CustomLinkVehicleNode::~CustomLinkVehicleNode()
{
    if (gimbal)
        gimbal->stop();
    if (client)
        client->stop();
}

// ======================================================================
// 遥测（读取线程上下文）
// ======================================================================

void CustomLinkVehicleNode::handleFast(const custom_link::protocol::PayloadFast &p, int64_t hostUs)
{
    // Betaflight 原生传感器/姿态与 ROS FLU 完全同轴同号，全部直通即可
    //（SITL 刚体闭环实证 src/test/sitl/sitl_custom_link_frames_test.py，
    // 固件三源互证 mavlink.c/sitl_gyro.h/pid.c pidLevel）：
    //   acc   = FLU（x 前、y 左、z 上，水平 z=+1g）
    //   gyro  = FLU 角速度（roll 右滚+、pitch 低头+ 即 FLU +y、yaw 逆时针+）
    //   姿态  = FLU 代数 RPY（roll 右滚+、pitch 低头+），yaw 右手系数学角
    //（逆时针+，0=磁北）。tf2 setRPY 即标准 ZYX 代数，pitch 参数同为低头正。
    // 此前代码误将 BF 当作左手系而取反 gyro/acc 的 y 与上行 pitch/yaw 速率，
    // 导致闭环 pitch 发散（真机 OFFBOARD 失控根因）。
    const double ax = p.acc[0] * 0.001 * gravity;
    const double ay = p.acc[1] * 0.001 * gravity;
    const double az = p.acc[2] * 0.001 * gravity;
    const double gx = p.gyro[0] * 0.1 * kDegToRad;
    const double gy = p.gyro[1] * 0.1 * kDegToRad;
    const double gz = p.gyro[2] * 0.1 * kDegToRad;

    {
        std::lock_guard<std::mutex> lk(mtx_att);
        accel_flu = {ax, ay, az};
        gyro_flu = {gx, gy, gz};
        // 姿态直通；yaw 仅做参考角旋转："0=磁北" → ENU 数学角 (0=东)：
        // yaw_enu = yaw + pi/2。
        roll_flu = p.attitude[0] * 0.01 * kDegToRad;
        pitch_flu = p.attitude[1] * 0.01 * kDegToRad;
        yaw_enu = normalizeAngle(p.attitude[2] * 0.01 * kDegToRad + M_PI_2);
        att_valid = true;
    }
    publishImu(hostUs);
}

void CustomLinkVehicleNode::publishImu(int64_t hostUs)
{
    sensor_msgs::msg::Imu msg;
    tf2::Quaternion q;
    {
        std::lock_guard<std::mutex> lk(mtx_att);
        if (!att_valid)
            return;
        q.setRPY(roll_flu, pitch_flu, yaw_enu);
        msg.angular_velocity.x = gyro_flu[0];
        msg.angular_velocity.y = gyro_flu[1];
        msg.angular_velocity.z = gyro_flu[2];
        msg.linear_acceleration.x = accel_flu[0];
        msg.linear_acceleration.y = accel_flu[1];
        msg.linear_acceleration.z = accel_flu[2];
    }
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(hostUs) * 1000, RCL_ROS_TIME);
    msg.header.frame_id = imu_frame_id;
    msg.orientation = tf2::toMsg(q);
    if (orientation_variance > 0.0)
        msg.orientation_covariance[0] = msg.orientation_covariance[4] =
            msg.orientation_covariance[8] = orientation_variance;
    if (angular_velocity_variance > 0.0)
        msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] =
            msg.angular_velocity_covariance[8] = angular_velocity_variance;
    if (linear_acceleration_variance > 0.0)
        msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] =
            msg.linear_acceleration_covariance[8] = linear_acceleration_variance;
    if (gimbal)
        gimbal->updateVehicleState(msg, gravity);
    imu_pub->publish(msg);

    // 地面且未解锁时估计当地重力，逻辑同 apm_bridge::VehicleNode::imuCallback。
    static uint8_t last_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
    const uint8_t current_state = landed_state.load();
    if (last_state != current_state)
    {
        last_state = current_state;
        ema.reset();
    }
    if (current_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND && !armed.load())
    {
        const double acc = std::sqrt(msg.linear_acceleration.x * msg.linear_acceleration.x +
                                     msg.linear_acceleration.y * msg.linear_acceleration.y +
                                     msg.linear_acceleration.z * msg.linear_acceleration.z);
        ema.filter(acc, 0);
    }
}

void CustomLinkVehicleNode::handleMedium(const custom_link::protocol::PayloadMedium &p, int64_t hostUs)
{
    mavros_msgs::msg::RCIn msg;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(hostUs) * 1000, RCL_ROS_TIME);
    msg.rssi = p.rssi_pct; // RC 链路质量 0..100（真机 rx RSSI 源 / SITL 帧新鲜度）
    msg.channels.assign(p.rc, p.rc + 16);
    rc_in_pub->publish(msg);

    rcTriggerUpdate(p.rc, 16);

    const double alt = p.baro_alt_cm * 0.01; // cm -> m
    if (airsim_enabled)
    {
        // SITL reports absolute barometric altitude, not height above the launch surface.
        const double filtered = sitl_altitude_ema.filter(alt);
        if (armed_feedback_valid && !armed.load())
        {
            sitl_ground_altitude = filtered;
            sitl_ground_valid = true;
        }
        altitude_m.store(filtered - sitl_ground_altitude);
    }
    else
    {
        altitude_m.store(alt);
    }
    const double relative_altitude = altitude_m.load();
    if (previous_altitude_us != 0 && hostUs > previous_altitude_us) {
        const double dt = static_cast<double>(hostUs - previous_altitude_us) * 1.0e-6;
        const double velocity = (relative_altitude - previous_altitude_m) / dt;
        if (std::isfinite(velocity) && dt <= 1.0)
            vertical_velocity_mps.store(std::clamp(velocity, -10.0, 10.0));
    }
    previous_altitude_m = relative_altitude;
    previous_altitude_us = hostUs;
    std_msgs::msg::Float64 altitude;
    altitude.data = alt;
    altitude_pub->publish(altitude);
    if (baro_compensation)
    {
        std::lock_guard<std::mutex> lk(mtx_kp);
        if (p.temp_cdeg > kAbsoluteZeroCdeg)
        {
            baro_temperature_c = p.temp_cdeg * 0.01;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Ignoring invalid barometric temperature %.2f C; retaining the previous temperature",
                                p.temp_cdeg * 0.01);
        }
        if (p.baro_pa > 0)
        {
            kp = (static_cast<double>(p.baro_pa) / kSeaLevelPressurePa) *
                 (kReferenceTemperatureKelvin / (kReferenceTemperatureKelvin + baro_temperature_c));
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Ignoring zero barometric pressure; retaining the previous pressure compensation factor");
        }
    }

    const rclcpp::Time stamp(static_cast<int64_t>(hostUs) * 1000, RCL_ROS_TIME);
    sensor_msgs::msg::FluidPressure pressure;
    pressure.header.stamp = stamp;
    pressure.fluid_pressure = static_cast<double>(p.baro_pa);
    pressure.variance = 0.0;
    pressure_pub->publish(std::move(pressure));

    sensor_msgs::msg::Temperature temperature;
    temperature.header.stamp = stamp;
    temperature.temperature = p.temp_cdeg * 0.01; // 0.01 degC -> degC
    temperature.variance = 0.0;
    temperature_pub->publish(std::move(temperature));
}

void CustomLinkVehicleNode::handleSlow(const custom_link::protocol::PayloadSlow &p, int64_t hostUs)
{
    const double voltage = p.vbat_mv * 0.001;
    const double amperage = p.current_ma * 0.001;

    {
        std::lock_guard<std::mutex> lk(mtx_kp);
        battery_voltage = voltage;
    }

    const bool now_armed = (p.status & custom_link::protocol::STATUS_ARMED) != 0;
    const bool was_armed = armed.exchange(now_armed);
    armed_feedback_valid = true;
    client->setArmedFeedback(now_armed);

    // 起桨/落桨边沿驱动下游：从未起桨到起桨发布 ON 触发 Tracker，从起桨到
    // 未起桨发布 OFF 停止 Tracker 与 Interceptor（0x12 10 Hz，边沿单次发布；
    // 自身的 gimbal_trigger_sub 订阅会同步产生云台 AIM/中止请求）。
    if (now_armed != was_armed)
    {
        std_msgs::msg::Header trigger;
        trigger.stamp = this->now();
        trigger.frame_id = now_armed ? "ON" : "OFF";
        trigger_pub->publish(std::move(trigger));
        RCLCPP_INFO(this->get_logger(), "Props %s: tracker trigger %s",
                    now_armed ? "spinning (armed)" : "stopped (disarmed)",
                    trigger.frame_id.c_str());
    }

    publishState(p.mode_flags, now_armed);

    sensor_msgs::msg::BatteryState bat;
    bat.header.stamp = rclcpp::Time(static_cast<int64_t>(hostUs) * 1000, RCL_ROS_TIME);
    bat.voltage = static_cast<float>(voltage);
    bat.current = static_cast<float>(-amperage); // ROS 约定放电为负
    // mAh 累计 (0x12 携带)。固件计数值单位为 mAh (int16)；
    // 换算隔离在此常量，若固件侧改为其他单位只需调整这一处。
    constexpr double kMahPerCount = 1.0;
    const double consumed_ah = p.mah * kMahPerCount * 0.001;
    bat.charge = static_cast<float>(consumed_ah); // 已耗 Ah (MAVROS sys_status 惯例)
    if (battery_capacity_mah > 0.0)
    {
        const double capacity_ah = battery_capacity_mah * 0.001;
        bat.capacity = static_cast<float>(capacity_ah);
        bat.design_capacity = bat.capacity;
        bat.percentage = static_cast<float>(
            std::clamp(1.0 - consumed_ah / capacity_ah, 0.0, 1.0));
    }
    else
    {
        bat.capacity = std::numeric_limits<float>::quiet_NaN();
        bat.design_capacity = std::numeric_limits<float>::quiet_NaN();
        bat.percentage = std::numeric_limits<float>::quiet_NaN();
    }
    bat.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    bat.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    bat.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    bat.present = voltage > 1.0;
    // cell_voltage[0] 放整包电压，与 MAVROS 在本工程中的用法保持一致。
    bat.cell_voltage = {static_cast<float>(voltage)};
    battery_pub->publish(bat);

    auto fb = std::make_unique<quadrotor_msgs::msg::LowLevelFeedback>();
    fb->header.stamp = bat.header.stamp;
    fb->battery_voltage = voltage;
    fb->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_INVALID;
    if (voltage > n_lipo_cells * kBatteryLowVoltagePerCell)
        fb->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_GOOD;
    else if (voltage > n_lipo_cells * kBatteryCriticalVoltagePerCell)
        fb->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_LOW;
    else if (voltage > n_lipo_cells * kBatteryInvalidVoltagePerCell)
        fb->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_CRITICAL;

    if (rc_manual.load())
        fb->control_mode = quadrotor_msgs::msg::LowLevelFeedback::RC_MANUAL;
    else
        fb->control_mode = use_rate.load() ? quadrotor_msgs::msg::LowLevelFeedback::BODY_RATES
                                           : quadrotor_msgs::msg::LowLevelFeedback::ATTITUDE;
    ap_feedback_pub->publish(std::move(fb));

    // ---- GPS (0x12 PayloadSlow)：GPSRAW 全量 + NavSatFix 经纬度 ----
    mavros_msgs::msg::GPSRAW gps;
    gps.header.stamp = bat.header.stamp;
    gps.fix_type = p.fix ? mavros_msgs::msg::GPSRAW::GPS_FIX_TYPE_3D_FIX
                         : mavros_msgs::msg::GPSRAW::GPS_FIX_TYPE_NO_FIX;
    gps.satellites_visible = p.sats;
    gps.lat = p.lat_e7;   // degE7 直传
    gps.lon = p.lon_e7;
    gps.alt = p.alt_msl_cm * 10; // cm -> mm
    gps.eph = std::numeric_limits<uint16_t>::max(); // 固件未提供精度信息
    gps.epv = std::numeric_limits<uint16_t>::max();
    gps.vel = p.gspeed_cms;      // cm/s
    gps.cog = p.course_cdeg;     // 0.01 deg
    gpsraw_pub->publish(std::move(gps));

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = bat.header.stamp;
    fix.header.frame_id = "base_link";
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.status.status = p.fix ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                              : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix.latitude = p.lat_e7 * 1e-7;
    fix.longitude = p.lon_e7 * 1e-7;
    fix.altitude = p.alt_msl_cm * 0.01;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    navsat_pub->publish(std::move(fix));

    // ---- 失效保护标志 ----
    std_msgs::msg::Bool failsafe;
    failsafe.data = (p.status & custom_link::protocol::STATUS_FAILSAFE) != 0;
    failsafe_pub->publish(std::move(failsafe));
}

void CustomLinkVehicleNode::handleOdometry(const custom_link::protocol::PayloadOdometry &p, int64_t hostUs)
{
    if ((p.flags & custom_link::protocol::ODOM_VALID_FLAGS) != custom_link::protocol::ODOM_VALID_FLAGS)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Ignoring odometry without GPS 3D fix, ARM origin or valid armed estimate");
        return;
    }
    if (p.arm_yaw_cdeg < -18000 || p.arm_yaw_cdeg > 18000)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Ignoring odometry with invalid ARM heading");
        return;
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(p.position_variance[axis]) || p.position_variance[axis] < 0.0f ||
            !std::isfinite(p.velocity_variance[axis]) || p.velocity_variance[axis] < 0.0f)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring odometry with invalid covariance");
            return;
        }
    }
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = rclcpp::Time(hostUs * 1000, RCL_ROS_TIME);
    msg.header.frame_id = "odom";
    msg.child_frame_id = imu_frame_id;
    const tf2::Quaternion arm_heading(tf2::Vector3(0, 0, 1),
        normalizeAngle(p.arm_yaw_cdeg * 0.01 * kDegToRad + M_PI_2));
    const tf2::Matrix3x3 world_to_odom(arm_heading.inverse());
    const tf2::Vector3 position = world_to_odom *
        tf2::Vector3(p.position_cm[0] * 0.01, p.position_cm[1] * 0.01, p.position_cm[2] * 0.01);
    msg.pose.pose.position.x = position.x();
    msg.pose.pose.position.y = position.y();
    msg.pose.pose.position.z = position.z();
    tf2::Quaternion q;
    q.setRPY(p.attitude_cdeg[0] * 0.01 * kDegToRad,
             p.attitude_cdeg[1] * 0.01 * kDegToRad,
             normalizeAngle(p.attitude_cdeg[2] * 0.01 * kDegToRad + M_PI_2));
    msg.pose.pose.orientation = tf2::toMsg(arm_heading.inverse() * q);
    // Twist stays in body FLU; only the pose frame changes from ENU to ARM heading.
    const tf2::Matrix3x3 world_to_body(q.inverse());
    const tf2::Vector3 velocity = world_to_body *
        tf2::Vector3(p.velocity_cms[0] * 0.01, p.velocity_cms[1] * 0.01, p.velocity_cms[2] * 0.01);
    msg.twist.twist.linear.x = velocity.x();
    msg.twist.twist.linear.y = velocity.y();
    msg.twist.twist.linear.z = velocity.z();
    msg.twist.twist.angular.x = p.gyro_dps_x10[0] * 0.1 * kDegToRad;
    msg.twist.twist.angular.y = p.gyro_dps_x10[1] * 0.1 * kDegToRad;
    msg.twist.twist.angular.z = p.gyro_dps_x10[2] * 0.1 * kDegToRad;
    for (int row = 0; row < 3; ++row)
    {
        // Conservative uncertainty when IMU covariance is not configured (zero means unknown).
        msg.pose.covariance[(row + 3) * 6 + row + 3] = orientation_variance > 0 ? orientation_variance : M_PI * M_PI;
        msg.twist.covariance[(row + 3) * 6 + row + 3] = angular_velocity_variance > 0 ? angular_velocity_variance : 1.0e6;
        for (int col = 0; col < 3; ++col)
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                msg.pose.covariance[row * 6 + col] += world_to_odom[row][axis] *
                    p.position_variance[axis] * world_to_odom[col][axis];
                msg.twist.covariance[row * 6 + col] += world_to_body[row][axis] *
                    p.velocity_variance[axis] * world_to_body[col][axis];
            }
        }
    }
    odom_pub->publish(msg);
}

void CustomLinkVehicleNode::publishState(uint32_t mode_flags, bool is_armed)
{
    namespace mode = custom_link::protocol::mode;

    // OFFBOARD 是自定义链路的接管模式，等价于 ArduPilot GUIDED / 旧 MSP_OVERRIDE。
    const bool offboard = mode_flags & mode::OFFBOARD;
    const bool was_offboard = !rc_manual.load();

    rc_manual.store(!offboard);

    if (in_mission.load() && was_offboard && !offboard)
    {
        // Send a trigger to indicate that offboard mode was lost during a mission
        std_msgs::msg::Header trigger;
        trigger.stamp = this->now();
        trigger.frame_id = "DISMISS";
        trigger_pub->publish(std::move(trigger));
    }

    const double alt = altitude_m.load();
    const double vertical_velocity = vertical_velocity_mps.load();
    const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const uint8_t in_air = mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
    const uint8_t on_ground = mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;

    if (airsim_enabled && !sitl_ground_valid)
    {
        landed_state.store(mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED);
        landed_candidate_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
        landed_candidate_since_us = 0;
        RCLCPP_WARN_ONCE(this->get_logger(), "SITL ground reference unavailable; landed state is undefined");
    }
    else if (!is_armed)
    {
        landed_state.store(on_ground);
        landed_candidate_state = on_ground;
        landed_candidate_since_us = 0;
    }
    else
    {
        uint8_t candidate = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
        if (landed_state.load() == in_air) {
            if (alt <= landing_altitude && std::abs(vertical_velocity) <= 0.35)
                candidate = on_ground;
        }
        else if (alt <= landing_altitude && std::abs(vertical_velocity) <= 0.35) {
            candidate = on_ground;
        }
        else if (alt >= airborne_altitude &&
                 (vertical_velocity >= 0.05 || alt >= airborne_altitude + 0.2)) {
            candidate = in_air;
        }

        if (candidate != landed_candidate_state) {
            landed_candidate_state = candidate;
            landed_candidate_since_us = candidate == mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED
                ? 0 : now_us;
        }
        if (candidate != mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED &&
            landed_candidate_since_us != 0) {
            const double confirmation_s = candidate == in_air ? takeoff_confirm_time_s : landing_confirm_time_s;
            if (now_us - landed_candidate_since_us >= static_cast<int64_t>(confirmation_s * 1.0e6))
                landed_state.store(candidate);
        }
    }

    mavros_msgs::msg::State state;
    state.header.stamp = this->now();
    state.connected = client->isUp();
    state.armed = is_armed;
    state.guided = offboard;
    state.manual_input = !offboard;

    if (offboard) {
        state.mode = "OFFBOARD";
    }
    else {
        bool plus = false;
        for(auto i = 0; i < mode_strings.size(); ++i) {
            if (mode_flags & (1u << i)) {
                if (plus)
                    state.mode += "|";
                else 
                    plus = true;
                state.mode += mode_strings[i];
            }
        }
        if (!plus)
            state.mode = "ACRO";
    }
    state_pub->publish(std::move(state));

    mavros_msgs::msg::ExtendedState ext;
    ext.header.stamp = this->now();
    ext.landed_state = landed_state.load();
    ext.vtol_state = mavros_msgs::msg::ExtendedState::VTOL_STATE_UNDEFINED;
    ext_state_pub->publish(std::move(ext));
}

// ======================================================================
// 控制
// ======================================================================

void CustomLinkVehicleNode::pushControl()
{
    custom_link::ControlInput input;
    bool fresh = false;
    {
        std::lock_guard<std::mutex> lk(mtx_cmd);
        if (cmd_valid)
        {
            if ((this->now() - cmd_stamp).seconds() < control_timeout)
            {
                // ROS FLU rad/s -> Betaflight 体轴 deg/s：同轴同号，直通换算
                //（BF 陀螺仪域即 FLU 角速度，见 handleFast 注释）。
                input.rate_dps[0] = cmd_bodyrates[0] * kRadToDeg;
                input.rate_dps[1] = cmd_bodyrates[1] * kRadToDeg;
                input.rate_dps[2] = cmd_bodyrates[2] * kRadToDeg;
                input.throttle = cmd_thrust;
                fresh = true;
            }
            else
            {
                cmd_valid = false; // 只作废控制量，arm 请求独立保留
            }
        }
    }
    // 发送前角速度限幅（deg/s，0 = 不限制）。独立于固件侧
    // custom_link_rate_limit_dps，作为客户端防御层；生效值取两者中更小。
    if (airsim_enabled && fresh &&
        (!std::isfinite(input.throttle) || !std::all_of(std::begin(input.rate_dps), std::end(input.rate_dps),
                                                      [](double rate) { return std::isfinite(rate); })))
    {
        fresh = false;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Rejected non-finite SITL control");
    }
    if (fresh && rate_limit_dps > 0.0)
    {
        for (int i = 0; i < 3; i++)
        {
            input.rate_dps[i] = std::clamp(input.rate_dps[i], -rate_limit_dps, rate_limit_dps);
        }
    }
    client->setControl(input, fresh);
}

void CustomLinkVehicleNode::updateGimbalMode()
{
    const auto next = gimbal_mode_selector.update(armed.load(),
        landed_state.load() == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR ||
            (mission_gimbal_on_ground && in_mission.load()),
        in_mission.load(), rc_manual.load(),
        gimbal_aim_requested.exchange(false), gimbal_abort_requested.exchange(false));
    if (next == gimbal_mode)
        return;
    gimbal_mode = next;
    if (gimbal)
        gimbal->setMode(next);
    std_msgs::msg::String msg;
    msg.data = apm_bridge::gimbalModeName(next);
    gimbal_mode_pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Gimbal mode -> %s", msg.data.c_str());
}


void CustomLinkVehicleNode::ctrlCommandCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command)
{
    if (set_unarm && armed.load() &&
        landed_state.load() == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND)
    {
        client->setArmRequest(false);
        set_unarm = false;
        return;
    }

    // 未解锁时不受理；解锁后的控制流是否接管由 FC 的 OFFBOARD 授权仲裁决定。
    if (command->armed == 0 || !armed.load())
        return;

    if (command->collective_thrust < 1e-6) // 准备落地上锁
        set_unarm = true;

    in_hover.store(command->is_hover_state != 0);

    if (command->control_mode == quadrotor_msgs::msg::ControlCommand::ATTITUDE)
    {
        // 自定义链路当前只承载角速度通道；姿态环留在上游。
        RCLCPP_WARN_ONCE(this->get_logger(),
                         "ATTITUDE control mode is not carried by the custom link, using bodyrates only");
        use_rate.store(false);
    }
    else
    {
        use_rate.store(true);
    }

    double thrust = 0.0;
    {
        std::lock_guard<std::mutex> lk(mtx_kp);
        const double force = command->collective_thrust * mass;
        if (voltage_compensation)
            thrust = quadratic_thrust_model::forceToThrust(motor_params, force, kp, battery_voltage);
        else
            thrust = quadratic_thrust_model::forceToThrust(motor_params, force, kp);
    }

    std::lock_guard<std::mutex> lk(mtx_cmd);
    cmd_bodyrates = {command->bodyrates.x, command->bodyrates.y, command->bodyrates.z};
    cmd_thrust = std::clamp(thrust, 0.0, 1.0);
    cmd_stamp = this->now();
    cmd_valid = true;
}

void CustomLinkVehicleNode::ctrlCommandRawCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command)
{
    if (command->armed == 0 || !armed.load())
        return;

    use_rate.store(command->control_mode == quadrotor_msgs::msg::ControlCommand::BODY_RATES);

    std::lock_guard<std::mutex> lk(mtx_cmd);
    cmd_bodyrates = {command->bodyrates.x, command->bodyrates.y, command->bodyrates.z};
    cmd_thrust = std::clamp(command->collective_thrust, 0.0, 1.0);
    cmd_stamp = this->now();
    cmd_valid = true;
}

void CustomLinkVehicleNode::armCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    // 解锁请求与模式请求独立；FC 仍执行 OFFBOARD 授权与 tryArm 安全检查。
    client->setArmRequest(msg->data);
}

void CustomLinkVehicleNode::offboardCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
        gimbal_abort_requested = true;
    client->setOffboardRequest(msg->data);
    RCLCPP_INFO(this->get_logger(), "Requested custom-link mode: %s",
                msg->data ? "OFFBOARD" : "MANUAL");
}

void CustomLinkVehicleNode::rcTriggerUpdate(const uint16_t *channels, size_t count)
{
    // RC 触发脉冲（>0.5 s 保持判定、单次锁存、释放复位）只负责 MISSION 的
    // 启动与停止：任务中脉冲 = 停止（DISMISS，仅解除拦截控制，Tracker 保持
    // 运行，联动退出 OFFBOARD）；空闲且视觉/雷达锁定时脉冲 = 触发（MISSION，
    // 联动进入 OFFBOARD）。Tracker 的启停由起桨/落桨边沿自动发布的 ON/OFF
    // 负责（handleSlow），不再由脉冲翻转。AirSim 未解锁时的脉冲仅启动视觉预览。
    static rclcpp::Time last_triggered_time;
    static bool toggled = false;

    if (static_cast<int>(count) <= channel_trigger)
        return;

    if (!toggled)
    {
        if (channels[channel_trigger] > 1900)
        {
            auto current_time = this->now();
            if (last_triggered_time.nanoseconds() == 0)
            {
                last_triggered_time = current_time;
            }
            else if ((current_time - last_triggered_time).seconds() > 0.5)
            {
                last_triggered_time = rclcpp::Time(0);
                toggled = true;
                auto msg = std::make_unique<std_msgs::msg::Header>();
                msg->stamp = this->now();
                if (airsim_enabled && !armed.load())
                {
                    // RC-only, non-arming vision preview for AirSim aim checks.
                    // Even a locked tracker must not start MISSION while disarmed.
                    msg->frame_id = "ON";
                }
                else if (in_mission.load())
                {
                    msg->frame_id = "DISMISS";
                }
                else if (in_tracking.load() || radar_tracking.load())
                {
                    msg->frame_id = "MISSION";
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Trigger pulse ignored: no vision/radar lock and no mission running");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Tracker Triggered : %s (vision=%d radar=%d)",
                            msg->frame_id.c_str(), in_tracking.load(), radar_tracking.load());
                trigger_pub->publish(std::move(msg));
            }
        }
        else
        {
            last_triggered_time = rclcpp::Time(0);
        }
    }
    else if (channels[channel_trigger] < 1100)
    {
        toggled = false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<custom_link_bridge::CustomLinkVehicleNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
