#include "custom_link_vehicle_node.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

using namespace custom_link_bridge;
using namespace std::chrono_literals;

namespace
{
    constexpr double kDegToRad = M_PI / 180.0;
    constexpr double kRadToDeg = 180.0 / M_PI;

    double normalizeAngle(double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a <= -M_PI)
            a += 2.0 * M_PI;
        return a;
    }

    /// ISA 标准大气：由 BF 估计高度反推气压比。0x11 已直接携带气压，
    /// 但推力补偿沿用高度模型以保持与 APM/MSP 桥一致的 kp 语义。
    double isaPressureRatio(double altitude_m)
    {
        const double h = std::clamp(altitude_m, -500.0, 11000.0);
        return std::pow(1.0 - 2.25577e-5 * h, 5.25588);
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
    this->declare_parameter<std::string>("gimbal_port", "");

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

    this->get_parameter("gravity_const", gravity);
    this->get_parameter("mass", mass);
    this->get_parameter("lipo_cells", n_lipo_cells);
    this->get_parameter("battery.capacity_mah", battery_capacity_mah);
    this->get_parameter("channel_trigger", channel_trigger);
    this->get_parameter("voltage_compensation", voltage_compensation);
    this->get_parameter("baro_compensation", baro_compensation);
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

    // ---------------- 云台 (可选, 支持 C-20S/C-40D/C-200T) ----------------
    std::string gimbal_port;
    this->get_parameter("gimbal_port", gimbal_port);
    if (!gimbal_port.empty())
    {
        xfrobot::GimbalConfig gimbal_cfg = xfrobot::GimbalConfig::fromParameters(*this);
        gimbal_uav_fusion = gimbal_cfg.send_uav_data;
        gimbal = std::make_unique<xfrobot::GimbalControl>(gimbal_port, gimbal_cfg);
        gimbal->setStatusCallback([this](const xfrobot::GimbalStatus &s) {
            if (s.hw_err != 0)
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                      "Gimbal hardware error: 0x%02X (需返厂检修)", s.hw_err);
            if (s.gbc_stat != gimbal_last_stat)
            {
                gimbal_last_stat = s.gbc_stat;
                static const char *kStatNames[] = {"UNDEFINED", "INIT", "STOPPED",
                                                   "PROTECTION", "MANUAL", "POINT_SHIFT"};
                if (s.gbc_stat == 3) // 倾角保护: 云台回中且不可控
                    RCLCPP_ERROR(this->get_logger(),
                                 "Gimbal entered PROTECTION state (倾角保护触发, 云台不可控)");
                else
                    RCLCPP_INFO(this->get_logger(), "Gimbal state -> %s",
                                kStatNames[s.gbc_stat < 6 ? s.gbc_stat : 0]);
            }
            if (s.tca_ready != gimbal_last_tca)
            {
                gimbal_last_tca = s.tca_ready;
                if (s.tca_ready)
                    RCLCPP_INFO(this->get_logger(), "Gimbal temperature control ready (可校准陀螺仪)");
            }
        });
        gimbal->run();
        RCLCPP_INFO(this->get_logger(), "Gimbal enabled on %s (model index %d, axes R/P/Y: %d/%d/%d, uav_fusion: %s)",
                    gimbal_port.c_str(), static_cast<int>(gimbal_cfg.model),
                    gimbal_cfg.roll.present ? 1 : 0, gimbal_cfg.pitch.present ? 1 : 0,
                    gimbal_cfg.yaw.present ? 1 : 0, gimbal_uav_fusion ? "on" : "off");
    }

    // ---------------- ROS 接口 ----------------
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
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
    trigger_pub = this->create_publisher<std_msgs::msg::Header>("/fpv/tracker_trigger", 10);
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
        [&](const std_msgs::msg::Bool::SharedPtr msg) {
            in_mission = msg->data;
        });

    if (gimbal)
    {
        gimbal_timer = this->create_wall_timer(std::chrono::milliseconds(10), [&]() {
            if (!gimbal->is_running())
                return;
            auto imu_msg = gimbal->getOrientationFLU();
            if (!imu_msg)
                return;
            tf2::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y,
                              imu_msg->orientation.z, imu_msg->orientation.w);
            tf2::Quaternion q_heading(tf2::Vector3(0, 0, 1), current_heading.load());
            tf2::Quaternion q_final = q_heading * q;
            imu_msg->orientation.x = q_final.x();
            imu_msg->orientation.y = q_final.y();
            imu_msg->orientation.z = q_final.z();
            imu_msg->orientation.w = q_final.w();
            imu_msg->header.frame_id = "base_link";
            gimbal_imu_pub->publish(std::move(imu_msg));
        });
    }

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

    std::unique_ptr<custom_link::Transport> transport;
    if (transport_kind == "tcp")
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
            default:
                break;
            }
            (void)fcUs;
        });

    client->start();

    // 控制量推送：回调只写缓存，这里统一判新鲜度并转换体轴后下发。
    // 超时后只作废控制量，arm 请求不动（链路恢复后自动重试解锁边沿）。
    cmd_push_timer = this->create_wall_timer(std::chrono::milliseconds(20), [this]() { pushControl(); });

    diag_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(0.05, diag_rate))), [this]() {
            const auto sync = client->timeSync();
            const auto stats = client->stats();
            if (!sync.valid)
                return;
            RCLCPP_INFO(this->get_logger(),
                        "Custom link: rtt %.2f ms (jitter %.2f), skew %+.0f ppm, FC offset %+.2f ms, "
                        "rx %lu (F:%u M:%u S:%u), tx %lu, reconnects %lu",
                        sync.rtt_ms, sync.jitter_ms, sync.skew_ppm, sync.offset_ms,
                        static_cast<unsigned long>(stats.rx_frames),
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_FAST & 7u],
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_MEDIUM & 7u],
                        stats.rx_per_msg[custom_link::protocol::MSG_FC_SLOW & 7u],
                        static_cast<unsigned long>(stats.tx_frames),
                        static_cast<unsigned long>(stats.reconnects));
        });

    rclcpp::on_shutdown([&]() {
        gimbal.reset();
    });
}

CustomLinkVehicleNode::~CustomLinkVehicleNode()
{
    if (client)
        client->stop();
}

// ======================================================================
// 遥测（读取线程上下文）
// ======================================================================

void CustomLinkVehicleNode::handleFast(const custom_link::protocol::PayloadFast &p, int64_t hostUs)
{
    // Betaflight 机体系 x 前 / y 右 / z 上（MultiWii 遗留，左手系）：
    // 转 ROS FLU：y 取反；角速度的 pitch/yaw 再取反（FLU 下抬头为负 pitch）。
    const double ax = p.acc[0] * 0.001 * gravity;
    const double ay = p.acc[1] * 0.001 * gravity;
    const double az = p.acc[2] * 0.001 * gravity;
    const double gx = p.gyro[0] * 0.1 * kDegToRad;
    const double gy = p.gyro[1] * 0.1 * kDegToRad;
    const double gz = p.gyro[2] * 0.1 * kDegToRad;

    {
        std::lock_guard<std::mutex> lk(mtx_att);
        accel_flu = {ax, -ay, az};
        gyro_flu = {gx, -gy, -gz};
        // 协议姿态即 FLU 右手系 (REP-103)：roll 左滚为正、pitch 抬头为负
        //（Betaflight 原生欧拉角，实测验证）、yaw 逆时针为正。roll/pitch 直接
        // 透传；yaw 从"0=磁北"旋转到 ENU 数学角 (0=东)：yaw_enu = yaw + pi/2。
        roll_flu = p.attitude[0] * 0.01 * kDegToRad;
        pitch_flu = p.attitude[1] * 0.01 * kDegToRad;
        yaw_enu = normalizeAngle(p.attitude[2] * 0.01 * kDegToRad + M_PI_2);
        att_valid = true;
    }
    current_heading.store(yaw_enu);

    // 云台载机惯导数据融合 (协议 V1.0.4, 默认关闭)。
    // 姿态角符号 (据协议附录1/2推导): 云台载机系为 FRD, 其"滚转右滚为正、
    // 俯仰抬头为正、偏航顺时针为正"; 而 BF 实测姿态为 roll 左滚为正、
    // pitch 抬头为负、yaw 逆时针为正 → 三轴均取反。
    // 偏航为 BF 磁北/上电航向, 与云台世界系零位存在未验证的偏移。
    // 加速度: 机体系(含重力)去重力后旋转到 NEU (协议要求大地系, 0.01m/s2)。
    if (gimbal && gimbal_uav_fusion)
    {
        const int16_t angle_cdeg[3] = {
            static_cast<int16_t>(-p.attitude[0]),
            static_cast<int16_t>(-p.attitude[1]),
            static_cast<int16_t>(-p.attitude[2])};

        tf2::Quaternion q_att;
        q_att.setRPY(roll_flu, pitch_flu, yaw_enu);
        const tf2::Vector3 a_flu(accel_flu[0], accel_flu[1], accel_flu[2]);
        const tf2::Vector3 a_enu = tf2::quatRotate(q_att, a_flu) + tf2::Vector3(0.0, 0.0, -gravity);
        const auto to_cms2 = [](double v) {
            return static_cast<int16_t>(std::clamp(v * 100.0, -32767.0, 32767.0));
        };
        const int16_t accel_cms2[3] = {
            to_cms2(a_enu.y()), // 北
            to_cms2(a_enu.x()), // 东
            to_cms2(a_enu.z())};// 天
        gimbal->setUavData(true, angle_cdeg, accel_cms2);
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
    msg.channels.assign(p.rc, p.rc + 16);
    rc_in_pub->publish(msg);

    rcTriggerUpdate(p.rc, 16);

    const double alt = p.baro_alt_cm * 0.01; // cm -> m
    altitude_m.store(alt);
    if (baro_compensation)
    {
        std::lock_guard<std::mutex> lk(mtx_kp);
        kp = isaPressureRatio(alt);
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
    armed.store(now_armed);
    client->setArmedFeedback(now_armed);

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

void CustomLinkVehicleNode::publishState(uint32_t mode_flags, bool is_armed)
{
    namespace mode = custom_link::protocol::mode;

    // OFFBOARD 是自定义链路的接管模式，等价于 ArduPilot GUIDED / 旧 MSP_OVERRIDE。
    if (mode_flags & mode::OFFBOARD)
        hoverable.store(2);
    else if ((mode_flags & mode::ANGLE) || (mode_flags & mode::HORIZON))
        hoverable.store(1);
    else
        hoverable.store(0);
    rc_manual.store(hoverable.load() != 2);

    const double alt = altitude_m.load();
    if (!is_armed)
        landed_state.store(mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND);
    else
        landed_state.store(alt > airborne_altitude
                               ? mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR
                               : mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND);

    mavros_msgs::msg::State state;
    state.header.stamp = this->now();
    state.connected = client->isUp();
    state.armed = is_armed;
    state.guided = (hoverable.load() == 2);
    state.manual_input = rc_manual.load();
    switch (hoverable.load())
    {
    case 2:
        state.mode = "OFFBOARD";
        break;
    case 1:
        state.mode = (mode_flags & mode::ANGLE) ? "ANGLE" : "HORIZON";
        break;
    default:
        state.mode = "ACRO";
        break;
    }
    state_pub->publish(state);

    mavros_msgs::msg::ExtendedState ext;
    ext.header.stamp = state.header.stamp;
    ext.landed_state = landed_state.load();
    ext.vtol_state = mavros_msgs::msg::ExtendedState::VTOL_STATE_UNDEFINED;
    ext_state_pub->publish(ext);
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
                // ROS FLU rad/s -> Betaflight 体轴 deg/s：pitch/yaw 取反
                input.rate_dps[0] = cmd_bodyrates[0] * kRadToDeg;
                input.rate_dps[1] = -cmd_bodyrates[1] * kRadToDeg;
                input.rate_dps[2] = -cmd_bodyrates[2] * kRadToDeg;
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
    if (fresh && rate_limit_dps > 0.0)
    {
        for (int i = 0; i < 3; i++)
        {
            input.rate_dps[i] = std::clamp(input.rate_dps[i], -rate_limit_dps, rate_limit_dps);
        }
    }
    client->setControl(input, fresh);
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

    // 未解锁或未进入 OFFBOARD 时不受理：OFFBOARD 的进入本身需要本节点持续
    // 发送控制流 + 飞手保持 BOXOFFBOARD 开关，所以解锁后首帧命令即可接管。
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
    // 解锁最终由飞手侧 BOXOFFBOARD 开关把关：开关未开时 FC 不会受理
    // 主机解锁（tryArm 全量安全检查照走）。
    client->setArmRequest(msg->data);
}

void CustomLinkVehicleNode::rcTriggerUpdate(const uint16_t *channels, size_t count)
{
    static rclcpp::Time last_triggered_time;
    static bool toggled = false;
    static bool trigger_state = false;

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
                if ((in_tracking || radar_tracking) && trigger_state && !in_mission)
                {
                    msg->frame_id = "MISSION";
                }
                else
                {
                    trigger_state = !trigger_state;
                    msg->frame_id = trigger_state ? "ON" : "OFF";
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
