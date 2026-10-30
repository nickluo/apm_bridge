#include "gimbal.h"
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <stdexcept>

using namespace xfrobot;

// 协议常量
#define GCU_SYNC_1 0xA9
#define GCU_SYNC_2 0x5B
#define GBC_SYNC_1 0xB5
#define GBC_SYNC_2 0x9A

// 命令码 (协议 V1.0.4, P3 注释3)
#define CMD_NONE            0 // 未定义
#define CMD_GYRO_CALIB      1 // 陀螺仪校准 (需温控就绪且云台静止)
#define CMD_START           2 // 启动云台
#define CMD_STOP            3 // 停止云台
#define CMD_MANUAL_CONTROL  4 // 手动控制
#define CMD_POINT_SHIFT     5 // 指点平移
#define CMD_TARGET_TRACK    6 // 目标跟踪

// 工作模式 (P4 注释6)
#define WK_MODE_FOLLOW 0 // 跟随模式
#define WK_MODE_LOCK 1   // 锁定模式
#define WK_MODE_FPV 2    // FPV 模式

// 控制模式 (P4 注释7)
#define OP_TYPE_ANGLE 0           // 角度控制
#define OP_TYPE_RATIO_SPEED 1   // 比例角速度
#define OP_TYPE_REAL_SPEED 2    // 真实角速度

#define SERIAL_READ_TIMEOUT_MS 10
#define UAV_DATA_TIMEOUT_MS 200 // 载机惯导数据过期阈值

static uint16_t CalculateCrc16(uint8_t *ptr, uint8_t len)
{
    uint16_t crc = 0;
    uint8_t da;
    const uint16_t crc_ta[16] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    };

    while (len--)
    {
        da = crc >> 12;
        crc <<= 4;
        crc ^= crc_ta[da ^ (*ptr >> 4)];
        da = crc >> 12;
        crc <<= 4;
        crc ^= crc_ta[da ^ (*ptr & 0x0F)];
        ptr++;
    }
    return (crc);
}

GimbalModel xfrobot::gimbalModelFromString(const std::string &name)
{
    // 大小写与连字符/下划线/空格不敏感: "C-20S" == "c20s"
    std::string s;
    for (char c : name)
    {
        if (c == '-' || c == '_' || c == ' ')
            continue;
        s += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (s == "c20s")
        return GimbalModel::C20S;
    if (s == "c40d")
        return GimbalModel::C40D;
    if (s == "c200t")
        return GimbalModel::C200T;
    throw std::invalid_argument("未知的云台机型: " + name + " (支持 C20S/C40D/C200T)");
}

GimbalConfig GimbalConfig::defaultsFor(GimbalModel model)
{
    GimbalConfig cfg;
    cfg.model = model;

    // 行程限位: 手册正文未给数值(在图纸)，默认沿用三轴机的现场调参数值，
    // 可通过 ROS 参数按机型覆盖。飞行前务必按图纸实测确认。
    cfg.pitch.min_deg = -135.0f;
    cfg.pitch.max_deg = 40.0f;
    cfg.roll.min_deg = -50.0f;
    cfg.roll.max_deg = 50.0f;
    cfg.yaw.min_deg = 0.0f; // 当前控制律未使用偏航限位
    cfg.yaw.max_deg = 0.0f;

    switch (model)
    {
    case GimbalModel::C20S: // 单轴: 仅俯仰
        cfg.roll.present = false;
        cfg.yaw.present = false;
        break;
    case GimbalModel::C40D: // 卧式两轴: 俯仰 + 滚转
        cfg.yaw.present = false;
        break;
    case GimbalModel::C200T: // 三轴
        break;
    }
    return cfg;
}

double GimbalAxisConfig::constrainDegrees(double angle) const
{
    if (!std::isfinite(angle) || !std::isfinite(min_deg) || !std::isfinite(max_deg) || min_deg > max_deg)
        throw std::invalid_argument("Invalid gimbal joint angle or mechanical limits");
    if (!present)
        return 0.0;
    return min_deg == max_deg ? angle : std::clamp(angle, double(min_deg), double(max_deg));
}

std::array<double, 3> GimbalConfig::aimTargetDegrees() const
{
    if (!std::isfinite(pitch_aim_deg) || std::abs(pitch_aim_deg) > 90)
        throw std::invalid_argument("gimbal.pitch_aim must be within [-90,90] degrees, positive nose-up");
    const std::array<double, 3> target = {roll.constrainDegrees(0),
        pitch.constrainDegrees(90.0 + pitch_aim_deg), yaw.constrainDegrees(0)};
    for (double value : target)
        if (std::abs(value) > 327.67)
            throw std::invalid_argument("AIM joint angle exceeds the vendor angle command range");
    return target;
}

tf2::Quaternion GimbalConfig::constrainMissionOrientation(const tf2::Quaternion &relative) const
{
    // Physical joint order matches the vendor pose: yaw * roll * pitch.
    // Unlike ZYX, ZXY has no pitch singularity at the new body-Z AIM reference.
    const auto desired = relative.normalized();
    const tf2::Matrix3x3 matrix(desired);
    const double r = std::asin(std::clamp(matrix[2][1], -1.0, 1.0));
    const bool singular = std::abs(std::cos(r)) < 1e-6;
    const double p = singular ? 0.0 : std::atan2(-matrix[2][0], matrix[2][2]);
    const double y = singular ? std::atan2(matrix[1][0], matrix[0][0]) :
        std::atan2(-matrix[0][1], matrix[1][1]);
    const auto wrap = [](double angle) { return std::atan2(std::sin(angle), std::cos(angle)); };
    const auto limited = [this](double roll_angle, double pitch_angle, double yaw_angle) {
        const double deg = 180.0 / M_PI;
        return apm_bridge::gimbalCameraToWorld(tf2::Quaternion(0, 0, 0, 1),
            {roll.constrainDegrees(roll_angle * deg) / deg, pitch.constrainDegrees(-pitch_angle * deg) / deg,
             yaw.constrainDegrees(-yaw_angle * deg) / deg}, {roll.present, pitch.present, yaw.present});
    };
    const auto first = limited(r, p, y);
    const auto second = limited(r >= 0 ? M_PI - r : -M_PI - r, wrap(p + M_PI), wrap(y + M_PI));
    return std::abs(desired.dot(second)) > std::abs(desired.dot(first)) + 1e-12 ? second : first;
}

GimbalConfig GimbalConfig::fromParameters(rclcpp::Node &node)
{
    // 第一段: 机型名决定轴存在性与默认限位
    node.declare_parameter<std::string>("gimbal.model", "C200T");
    std::string model_name;
    node.get_parameter("gimbal.model", model_name);
    GimbalConfig cfg = defaultsFor(gimbalModelFromString(model_name));

    // 第二段: 其余参数按机型默认值声明，允许 yaml/launch 逐项覆盖
    auto param_f = [&node](const std::string &name, float def) -> float {
        node.declare_parameter<double>(name, static_cast<double>(def));
        return static_cast<float>(node.get_parameter(name).as_double());
    };
    // 机械行程限位: 编码器(关节)域, FRD 抬头正, 按图纸实测配置。
    // 与姿态角/pitch_hold/pitch_aim 无关, 仅作为编码器角度门控与 AIM 目标钳制。
    cfg.roll.min_deg = param_f("gimbal.roll_travel_min", cfg.roll.min_deg);
    cfg.roll.max_deg = param_f("gimbal.roll_travel_max", cfg.roll.max_deg);
    cfg.pitch.min_deg = param_f("gimbal.pitch_travel_min", cfg.pitch.min_deg);
    cfg.pitch.max_deg = param_f("gimbal.pitch_travel_max", cfg.pitch.max_deg);
    cfg.yaw.min_deg = param_f("gimbal.yaw_travel_min", cfg.yaw.min_deg);
    cfg.yaw.max_deg = param_f("gimbal.yaw_travel_max", cfg.yaw.max_deg);
    cfg.pitch_roll_speed_scale = param_f("gimbal.pitch_roll_speed_scale", cfg.pitch_roll_speed_scale);
    cfg.yaw_speed_scale = param_f("gimbal.yaw_speed_scale", cfg.yaw_speed_scale);
    cfg.pitch_aim_deg = param_f("gimbal.pitch_aim", cfg.pitch_aim_deg);
    cfg.pitch_mission_min_deg = param_f("gimbal.pitch_mission_min", cfg.pitch_mission_min_deg);
    cfg.pitch_mission_max_deg = param_f("gimbal.pitch_mission_max", cfg.pitch_mission_max_deg);
    cfg.aim_follow_gain = param_f("gimbal.aim_follow_gain", cfg.aim_follow_gain);
    if (!std::isfinite(cfg.aim_follow_gain) || cfg.aim_follow_gain <= 0 || cfg.aim_follow_gain > 100)
        throw std::invalid_argument("Invalid gimbal AIM follow gain (0,100]");
    cfg.pitch_hold_deg = param_f("gimbal.pitch_hold", cfg.pitch_hold_deg);
    cfg.roll_hold_enter_deg = param_f("gimbal.roll_hold_enter", cfg.roll_hold_enter_deg);
    cfg.roll_hold_exit_deg = param_f("gimbal.roll_hold_exit", cfg.roll_hold_exit_deg);
    // 启动回中 (机械零点): 节点默认开 (启动后先回零再进 AIM); harness/工具用
    // C++ 默认构造 (false) 保持原语义。详见 GimbalConfig 注释。
    node.declare_parameter<bool>("gimbal.home_on_connect", true);
    node.get_parameter("gimbal.home_on_connect", cfg.home_on_connect);
    cfg.home_timeout_s = param_f("gimbal.home_timeout_s", cfg.home_timeout_s);
    cfg.home_tolerance_deg = param_f("gimbal.home_tolerance_deg", cfg.home_tolerance_deg);
    cfg.tracking.gain = param_f("gimbal.tracking_gain", cfg.tracking.gain);
    cfg.tracking.rate_feedforward = param_f("gimbal.tracking_rate_feedforward", cfg.tracking.rate_feedforward);
    cfg.tracking.max_rate_dps = param_f("gimbal.tracking_max_rate", cfg.tracking.max_rate_dps);
    cfg.tracking.max_accel_dps2 = param_f("gimbal.tracking_max_accel", cfg.tracking.max_accel_dps2);
    cfg.tracking.filter_tau = param_f("gimbal.tracking_filter_tau", cfg.tracking.filter_tau);
    cfg.tracking.los_timeout = param_f("gimbal.los_timeout", cfg.tracking.los_timeout);
    cfg.tracking.validate();
    if (!std::isfinite(cfg.pitch_hold_deg) || std::abs(cfg.pitch_hold_deg) > 90 ||
        !std::isfinite(cfg.roll_hold_enter_deg) || !std::isfinite(cfg.roll_hold_exit_deg) ||
        cfg.roll_hold_exit_deg < 0 || cfg.roll_hold_enter_deg < cfg.roll_hold_exit_deg)
        throw std::invalid_argument("Invalid gimbal pitch hold or roll protection thresholds");
    if (!std::isfinite(cfg.home_timeout_s) || cfg.home_timeout_s <= 0 || cfg.home_timeout_s > 300 ||
        !std::isfinite(cfg.home_tolerance_deg) || cfg.home_tolerance_deg < 0 || cfg.home_tolerance_deg > 45)
        throw std::invalid_argument("Invalid gimbal homing timeout (0,300] s or tolerance [0,45] deg");
    for (const auto &axis : {cfg.roll, cfg.pitch, cfg.yaw})
        if (!std::isfinite(axis.min_deg) || !std::isfinite(axis.max_deg) ||
            axis.min_deg > axis.max_deg)
            throw std::invalid_argument("Invalid gimbal mechanical limits");
    if (!std::isfinite(cfg.pitch_mission_min_deg) || !std::isfinite(cfg.pitch_mission_max_deg) ||
        cfg.pitch_mission_min_deg >= cfg.pitch_mission_max_deg)
        throw std::invalid_argument("Invalid gimbal MISSION pitch window (min < max required)");
    if (90.0 + cfg.pitch_mission_min_deg > cfg.pitch.max_deg ||
        90.0 + cfg.pitch_mission_max_deg < cfg.pitch.min_deg)
        throw std::invalid_argument("Gimbal MISSION pitch window does not intersect mechanical travel");
    cfg.aimTargetDegrees();

    node.declare_parameter<bool>("gimbal.send_uav_data", cfg.send_uav_data);
    node.get_parameter("gimbal.send_uav_data", cfg.send_uav_data);

    return cfg;
}

GimbalControl::GimbalControl(const std::string &port)
    : GimbalControl(port, GimbalConfig::defaultsFor(GimbalModel::C200T))
{
}

GimbalControl::GimbalControl(const std::string &port, const GimbalConfig &gimbal_cfg)
    : port_name(port), is_open(false), running(false),
      homing_done(!gimbal_cfg.home_on_connect), cfg(gimbal_cfg),
      aim_target_degrees(cfg.aimTargetDegrees()), tracking_controller(cfg.tracking),
      angle_trajectory(cfg.tracking), cmd_trig_counter(0)
{
    memset(ctrl_op_type, 0, sizeof(ctrl_op_type));
    memset(ctrl_op_value, 0, sizeof(ctrl_op_value));

    // 初始化串口对象，但不打开
    serial_port.setPort(port_name);
    serial_port.setBaudrate(115200);
    serial_port.setBytesize(serial::eightbits);
    serial_port.setParity(serial::parity_none);
    serial_port.setStopbits(serial::stopbits_one);
    serial_port.setFlowcontrol(serial::flowcontrol_none);
    auto timeout = serial::Timeout::simpleTimeout(SERIAL_READ_TIMEOUT_MS);
    serial_port.setTimeout(timeout);

    memset(&last_response, 0, sizeof(Gbc2GcuPkt_t));
    memset(&last_status, 0, sizeof(GimbalStatus));
    open();
}

/**
 * @brief 析构函数
 */
GimbalControl::GimbalControl(rclcpp::Node &ros_node, const std::string &port,
                            const GimbalConfig &gimbal_cfg, Callback callback)
    : GimbalControl(port, gimbal_cfg)
{
    if (!callback)
        throw std::invalid_argument("Gimbal orientation callback is required");
    node = &ros_node;
    orientation_callback = std::move(callback);
    const auto logger = ros_node.get_logger();
    const auto clock = ros_node.get_clock();
    setStatusCallback([logger, clock, last_stat = uint8_t{0}, last_tca = false](const GimbalStatus &s) mutable {
        if (s.hw_err != 0)
            RCLCPP_ERROR_THROTTLE(logger, *clock, 5000, "Gimbal hardware error: 0x%02X (需返厂检修)", s.hw_err);
        if (s.gbc_stat != last_stat)
        {
            last_stat = s.gbc_stat;
            static const char *names[] = {"UNDEFINED", "INIT", "STOPPED", "PROTECTION", "MANUAL", "POINT_SHIFT"};
            if (s.gbc_stat == 3)
                RCLCPP_ERROR(logger, "Gimbal entered PROTECTION state (倾角保护触发, 云台不可控)");
            else
                RCLCPP_INFO(logger, "Gimbal state -> %s", names[s.gbc_stat < 6 ? s.gbc_stat : 0]);
        }
        if (s.tca_ready != last_tca)
        {
            last_tca = s.tca_ready;
            if (s.tca_ready)
                RCLCPP_INFO(logger, "Gimbal temperature control ready (可校准陀螺仪)");
        }
    });
}

GimbalControl::~GimbalControl()
{
    stop();
}

void GimbalControl::stop()
{
    if (orientation_timer)
    {
        orientation_timer->cancel();
        orientation_timer.reset();
    }
    if (work_thread.joinable())
    {
        running = false;
        work_thread.join();
    }
    start_stop(false); // 停止云台
    close();
}

void GimbalControl::start()
{
    if (running.load())
        return;
    if (!open())
        RCLCPP_ERROR(node ? node->get_logger() : rclcpp::get_logger("xfrobot_gimbal"),
                     "Gimbal serial port not available yet: %s (worker will keep retrying)",
                     port_name.c_str());
    run();
    if (node)
    {
        orientation_timer = node->create_wall_timer(std::chrono::milliseconds(10), [this]() {
            if (!is_running())
                return;
            auto msg = makeOrientationFLU(true);
            if (msg)
                orientation_callback(std::move(msg));
        });
        RCLCPP_INFO(node->get_logger(),
                    "Gimbal enabled on %s (model index %d, axes R/P/Y: %d/%d/%d, uav_fusion: %s)",
                    port_name.c_str(), static_cast<int>(cfg.model),
                    cfg.roll.present ? 1 : 0, cfg.pitch.present ? 1 : 0, cfg.yaw.present ? 1 : 0,
                    cfg.send_uav_data ? "on" : "off");
    }
}

void GimbalControl::updateVehicleState(const sensor_msgs::msg::Imu &imu, double gravity)
{
    tf2::Quaternion q;
    tf2::fromMsg(imu.orientation, q);
    if (!std::isfinite(q.length2()) || q.length2() < 1e-12)
    {
        RCLCPP_ERROR(node ? node->get_logger() : rclcpp::get_logger("xfrobot_gimbal"),
                     "Ignoring invalid vehicle orientation for gimbal");
        {
            std::lock_guard<std::mutex> lock(mtx);
            vehicle_stamp = {};
        }
        if (cfg.send_uav_data)
        {
            const int16_t invalid[3] = {};
            setUavData(false, invalid, invalid);
        }
        return;
    }
    q.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    {
        std::lock_guard<std::mutex> lock(mtx);
        vehicle_heading = yaw;
        vehicle_orientation = q;
        // BF 陀螺 FLU 俯仰速率低头正; 转抬头域供 AIM 前馈
        vehicle_pitch_rate_up = -imu.angular_velocity.y * 180.0 / M_PI;
        vehicle_stamp = std::chrono::steady_clock::now();
    }
    if (!cfg.send_uav_data)
        return;
    const auto &a = imu.linear_acceleration;
    if (!std::isfinite(gravity) || gravity <= 0 ||
        !std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z))
    {
        RCLCPP_ERROR(node ? node->get_logger() : rclcpp::get_logger("xfrobot_gimbal"),
                     "Ignoring invalid vehicle acceleration/gravity for gimbal fusion");
        const int16_t invalid[3] = {};
        setUavData(false, invalid, invalid);
        return;
    }
    const auto encode = [](double value) {
        return static_cast<int16_t>(std::clamp(std::round(value), -32767.0, 32767.0));
    };
    double north_heading = M_PI_2 - yaw;
    while (north_heading > M_PI)
        north_heading -= 2 * M_PI;
    while (north_heading <= -M_PI)
        north_heading += 2 * M_PI;
    const double cdeg = 18000.0 / M_PI;
    const int16_t angle[3] = {encode(roll * cdeg), encode(-pitch * cdeg), encode(north_heading * cdeg)};
    const auto world = tf2::quatRotate(q, tf2::Vector3(a.x, a.y, a.z)) + tf2::Vector3(0, 0, -gravity);
    const auto encodeAcceleration = [](double value) {
        return static_cast<int16_t>(std::clamp(value * 100, -32767.0, 32767.0));
    };
    const int16_t acceleration[3] = {encodeAcceleration(world.y()), encodeAcceleration(world.x()), encodeAcceleration(world.z())};
    setUavData(true, angle, acceleration);
}

void GimbalControl::run()
{
    if (running.load())
        return;

    running = true;

    work_thread = std::thread([&](){
        // 连接(含断线重连)初始化: 丢弃旧缓冲、启动云台、复位轨迹与反馈状态。
        // 打开失败 (设备未插/被占用/非串口设备) 由线程每秒重试, 不再一次性放弃。
        bool homing_edge_sent = false; // 本次连接是否已发出 go_zero 边沿 (工作线程局部)
        std::chrono::steady_clock::time_point homing_deadline{};
        bool go_zero_pulse_active = false;           // 边沿脉冲发送中 (发送后须清位)
        std::chrono::steady_clock::time_point go_zero_pulse_until{};
        auto establish_link = [&](bool first) {
            try
            {
                serial_port.flushInput();
            }
            catch (const serial::IOException &)
            {
            }
            catch (const serial::SerialException &)
            {
            }
            start_stop(true);
            control_stamp = std::chrono::steady_clock::now();
            angle_trajectory_ready = false;
            rx_state = SYNC1;
            rx_index = 0;
            homing_edge_sent = false;
            go_zero_pulse_active = false;
            {
                std::lock_guard<std::mutex> lock(mtx);
                last_status.rx_ok = false;
                feedback_stamp = {};
            }
            if (!homing_done.load())
            {
                // 回中未完成时 (重)连接: 清 go_zero 位, 保证下一反馈包
                // 重新产生干净的 0->1 边沿触发回中; 同时从连接时刻起算完成
                // 超时——若云台从不回包或回中途中静默, 循环顶的超时判定也能
                // 放行模式控制, 不会永久卡在 HOMING
                std::lock_guard<std::mutex> lock(cmd_mtx);
                go_zero_bits = 0;
                homing_deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(cfg.home_timeout_s));
            }
            RCLCPP_INFO(node ? node->get_logger() : rclcpp::get_logger("xfrobot_gimbal"),
                        first ? "Gimbal connected on %s" : "Gimbal reconnected on %s",
                        port_name.c_str());
        };

        bool was_open = false;
        int open_failures = 0;
        auto last_attempt = std::chrono::steady_clock::time_point{};
        while (running.load())
        {
            if (!is_open)
            {
                const auto t = std::chrono::steady_clock::now();
                if (t - last_attempt < std::chrono::seconds(1))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                last_attempt = t;
                if (!open())
                {
                    if (++open_failures == 1 || open_failures % 10 == 0)
                        RCLCPP_ERROR(node ? node->get_logger() : rclcpp::get_logger("xfrobot_gimbal"),
                                     "Gimbal serial open failed (%d attempts): %s - retrying every second",
                                     open_failures, port_name.c_str());
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                open_failures = 0;
            }
            if (is_open != was_open)
                establish_link(!was_open);
            was_open = is_open;
            const auto now = std::chrono::steady_clock::now();
            const double dt = std::clamp(std::chrono::duration<double>(now - control_stamp).count(), 1e-4, 0.1);
            control_stamp = now;
            const auto requested = mode.load();
            if (requested != control_mode)
            {
                tracking_controller.reset();
                control_mode = requested;
                roll_hold_active = false;
                angle_trajectory_ready = false;
                angle_axes = {};
                for (size_t i = 0; i < 3; ++i)
                {
                    ctrl_op_type[i] = OP_TYPE_REAL_SPEED;
                    ctrl_op_value[i] = 0;
                }
            }
            // AIM: LOCK+真实角速度, P(位置)+FF(载机俯仰速率前馈)。角度通道方案实测
            // 滞后 ~0.15s (固件角度环内轨迹 + 驱动轨迹 8x 误差项), 速率通道台架实测
            // 严格 1:1 即时响应, 滞后降至更新周期量级。目标角 = 机身抬头角 +
            // 相对角 (90+pitch_aim); 不使用云台 FPV 跟随/uav 融合 (固件无 uav 数据时
            // 不随机身转动, 且约定 send_uav_data 恒 false)。滚转/偏航轴锁定当前姿态。
            ctrl_wk_mode = WK_MODE_LOCK;
            const bool homing = !homing_done.load();
            if (homing)
            {
                // 启动回中: FPV+角度 op=0 主动把各轴编码器拉回机械零点。台架实测
                // (2026-10-26/30): FPV 相对角零点=编码器零点, mtr=-op 精确驱动,
                // 且需 fl_sens=15 才有有效角度跟踪 (sendControlPacket 按 FPV 自动置);
                // 固件 go_zero 回中位在真机 C20S (fw38) 上为无效操作
                // (test/gimbal_bench/probe_go_zero.py), 边沿仅作其它机型的兼容保留。
                // 模式控制律一律不介入 (完成判定见下方反馈包处理)。
                ctrl_wk_mode = WK_MODE_FPV;
                for (int i = 0; i < 3; ++i)
                {
                    ctrl_op_type[i] = OP_TYPE_ANGLE;
                    ctrl_op_value[i] = 0;
                }
            }
            else if (control_mode == apm_bridge::GimbalMode::AIM)
            {
                double body_rate_ff = 0;
                bool feedback_fresh = false;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    feedback_fresh = feedback_stamp != std::chrono::steady_clock::time_point{} &&
                        now - feedback_stamp <= std::chrono::milliseconds(UAV_DATA_TIMEOUT_MS);
                    if (vehicle_stamp != std::chrono::steady_clock::time_point{} &&
                        now - vehicle_stamp <= std::chrono::milliseconds(UAV_DATA_TIMEOUT_MS))
                    {
                        double roll_c, pitch_c, yaw_c;
                        tf2::Matrix3x3(vehicle_orientation).getRPY(roll_c, pitch_c, yaw_c);
                        aim_body_pitch_deg = -pitch_c * 180.0 / M_PI; // FLU 俯仰低头正 → 抬头角
                        body_rate_ff = vehicle_pitch_rate_up;
                    }
                }
                for (int i = 0; i < 3; ++i)
                {
                    ctrl_op_type[i] = OP_TYPE_REAL_SPEED;
                    ctrl_op_value[i] = 0;
                }
                const double target = aim_target_degrees[1] + aim_body_pitch_deg;
                const double error = target - last_response.cam_angle[1] * 0.01; // 同抬头域
                double speed = cfg.aim_follow_gain * error + body_rate_ff;
                const double joint = -last_response.mtr_angle[0] * 0.01; // 编码器低头正 → 抬头域
                if (cfg.pitch.min_deg < cfg.pitch.max_deg &&
                    ((joint <= cfg.pitch.min_deg && speed < 0) || (joint >= cfg.pitch.max_deg && speed > 0)))
                    speed = 0; // 机械限位门: 只拦朝限位方向
                speed = std::clamp(speed, -cfg.tracking.max_rate_dps, cfg.tracking.max_rate_dps);
                if (!feedback_fresh)
                    speed = 0; // 反馈过期: 锁定当前姿态
                ctrl_op_value[1] = static_cast<int16_t>(std::round(speed * 10.0));
            }
            // go_zero 边沿脉冲 (~100ms) 发完即清位: 协议语义为位变化触发,
            // 脉冲化避免长期占用该位 (真机实测该位无副作用)
            if (go_zero_pulse_active && now >= go_zero_pulse_until)
            {
                go_zero_pulse_active = false;
                std::lock_guard<std::mutex> lock(cmd_mtx);
                go_zero_bits = 0;
            }
            // 回中超时在循环顶判定 (不依赖反馈包到达): 云台从不回包或回中途中
            // 静默时同样放行模式控制, 避免永久卡在 HOMING
            if (!homing_done.load() && now > homing_deadline)
            {
                homing_done = true;
                RCLCPP_WARN(node ? node->get_logger() : rclcpp::get_logger("xfrobot_gimbal"),
                            "Gimbal homing timeout after %.1f s; resuming mode control anyway",
                            cfg.home_timeout_s);
            }
            // 尝试读取返回包
            if (readPacket())
            {
                // ---- 启动回中: 本次连接的首个反馈包产生 go_zero 0->1 边沿 ----
                // (establish_link 已清位。真机 C20S fw38 实测 (2026-10-30,
                // probe_go_zero.py): go_zero 位既不触发回中动作, 也不影响控制命令;
                // 边沿仅为其它机型/固件兼容保留, 以 ~100ms 脉冲发出后清位保持语义整洁)
                if (!homing_done.load() && !homing_edge_sent)
                {
                    homing_edge_sent = true;
                    go_zero_pulse_active = true;
                    go_zero_pulse_until = now + std::chrono::milliseconds(100);
                    uint8_t mask = 0;
                    if (cfg.roll.present)
                        mask |= 1u << 0;
                    if (cfg.pitch.present)
                        mask |= 1u << 1;
                    if (cfg.yaw.present)
                        mask |= 1u << 2;
                    {
                        std::lock_guard<std::mutex> lock(cmd_mtx);
                        go_zero_bits = mask;
                    }
                    RCLCPP_INFO(node ? node->get_logger() : rclcpp::get_logger("xfrobot_gimbal"),
                                "Gimbal homing: go_zero axes 0x%X, tolerance %.1f deg, timeout %.1f s",
                                mask, cfg.home_tolerance_deg, cfg.home_timeout_s);
                }
                // ---- 回中完成判定: 全部在轴编码器进入零点容差 (超时在循环顶) ----
                if (!homing_done.load() && homing_edge_sent)
                {
                    const double tol = cfg.home_tolerance_deg;
                    const bool at_zero =
                        (!cfg.roll.present || std::abs(last_response.mtr_angle[1] * 0.01) <= tol) &&
                        (!cfg.pitch.present || std::abs(last_response.mtr_angle[0] * 0.01) <= tol) &&
                        (!cfg.yaw.present || std::abs(last_response.mtr_angle[2] * 0.01) <= tol);
                    if (at_zero)
                    {
                        homing_done = true;
                        RCLCPP_INFO(node ? node->get_logger() : rclcpp::get_logger("xfrobot_gimbal"),
                                    "Gimbal homed to mechanical zero; mode control enabled");
                    }
                }
                // ---- 控制律 (按机型门控; C-40D/C-200T 与原实现逐字节一致) ----
                if (homing)
                {
                    // 回中期间保持循环顶的 FPV+角度0 回零命令, 不运行模式控制律
                }
                else if (control_mode == apm_bridge::GimbalMode::MISSION)
                    updateMissionControl(dt);
                else if (control_mode == apm_bridge::GimbalMode::AIM)
                {
                    // The body-relative AIM command is independent of gimbal feedback.
                }
                else if (cfg.model == GimbalModel::C20S)
                {
                    // C-20S 单轴逻辑: 俯仰轴由云台内置角度环闭环。
                    // 常态: 锁定+角度控制, 保持期望俯仰姿态角;
                    // |IMU滚转角| > enter (载机大坡度/倒飞, 单轴无法维持水平且
                    // 可能触发倾角保护): 切锁定+真实角速度 op=0 锁定当前俯仰角;
                    // |滚转| < exit 后恢复角度控制 (exit 可设小于 enter 构成迟滞)
                    const float imu_roll_deg = last_response.cam_angle[0] * 0.01f;
                    if (!roll_hold_active && std::abs(imu_roll_deg) > cfg.roll_hold_enter_deg)
                        roll_hold_active = true;
                    else if (roll_hold_active && std::abs(imu_roll_deg) < cfg.roll_hold_exit_deg)
                        roll_hold_active = false;

                    // 不存在的滚转/偏航轴: 锁定+零速 (固件忽略)
                    ctrl_op_type[0] = OP_TYPE_REAL_SPEED;
                    ctrl_op_value[0] = 0;
                    ctrl_op_type[2] = OP_TYPE_REAL_SPEED;
                    ctrl_op_value[2] = 0;

                    if (!roll_hold_active)
                    {
                        // 锁定+角度控制: op_value 单位 0.01deg
                        ctrl_op_type[1] = OP_TYPE_ANGLE;
                        ctrl_op_value[1] = static_cast<int16_t>(cfg.pitch_hold_deg * 100.0f);
                    }
                    else
                    {
                        // 锁定+真实角速度控制: op_value=0 锁定当前俯仰角
                        ctrl_op_type[1] = OP_TYPE_REAL_SPEED;
                        ctrl_op_value[1] = 0;
                    }
                }
                else
                {
                    float yaw_exp = 0.0f;
                    float pitch_exp = 0.0f;
                    float roll_exp = 0.0f;
                    bool max = false;
                    float r_ratio = 1.0f;
                    float p_ratio = 1.0f;
                    // 统一每轴角速度上限 (原 max_angle_speed 并入 tracking_max_rate)
                    const float max_rate_dps = static_cast<float>(cfg.tracking.max_rate_dps);

                    // 滚转轴: P 控制器驱动相机姿态角回水平
                    if (cfg.roll.present)
                    {
                        roll_exp = -last_response.cam_angle[0] * 0.01f * cfg.pitch_roll_speed_scale;
                        roll_exp = std::clamp(roll_exp, -max_rate_dps, max_rate_dps);

                        float mtr_roll = last_response.mtr_angle[1] * 0.01f;
                        if (std::abs(mtr_roll) >= cfg.roll.max_deg)
                        {
                            roll_exp = 0.0f;
                            max = true;
                        }
                        r_ratio = 1.0f - std::abs(mtr_roll) / cfg.roll.max_deg;
                    }

                    // 俯仰轴: P 控制器驱动相机姿态角回水平
                    if (cfg.pitch.present)
                    {
                        pitch_exp = (cfg.pitch_hold_deg - last_response.cam_angle[1] * 0.01f) * cfg.pitch_roll_speed_scale;
                        pitch_exp = std::clamp(pitch_exp, -max_rate_dps, max_rate_dps);

                        // 编码器低头正 (台架实测): 取反到抬头域再与限位比较,
                        // 否则门控方向相反会把云台推进机械限位。C-40D/C-200T 待实机复验。
                        const float mtr_pitch = -last_response.mtr_angle[0] * 0.01f;
                        if (std::abs(mtr_pitch - last_response.cam_angle[1] * 0.01f) < cfg.pitch.max_deg && (mtr_pitch <= cfg.pitch.min_deg || mtr_pitch >= cfg.pitch.max_deg))
                        {
                            pitch_exp = 0.0f;
                            max = true;
                        }
                        p_ratio = 1.0f - mtr_pitch / (std::signbit(mtr_pitch) ? cfg.pitch.min_deg : cfg.pitch.max_deg);
                    }

                    float scale = std::min(r_ratio, p_ratio);
                    if (scale < 0.01f)
                        scale = 0.01f;
                    else if (scale > 1.0f)
                        scale = 1.0f;

                    // 偏航轴: 驱动偏航编码器回中 (软缩放 + 限位逼近低速蠕行)
                    if (cfg.yaw.present)
                    {
                        float current_yaw_encoder = last_response.mtr_angle[2] * 0.01f;

                        if (max)
                        {
                            // Keep low speed when reaching limit
                            yaw_exp = std::signbit(current_yaw_encoder) ? max_rate_dps * 0.01f : -max_rate_dps * 0.01f;
                        }
                        else
                        {
                            yaw_exp = -current_yaw_encoder * cfg.yaw_speed_scale * scale;
                            if (std::abs(yaw_exp) < 0.1f)
                                yaw_exp = 0.0f;
                            else
                                yaw_exp = std::clamp(yaw_exp, -max_rate_dps, max_rate_dps);
                        }
                    }

                    // 发送控制命令: ctrl 顺序 [0-滚转, 1-俯仰, 2-偏航], 真实角速度 0.1deg/s
                    const float speed_gbc[3] = {roll_exp, pitch_exp, yaw_exp};
                    for (int i = 0; i < 3; i++)
                    {
                        ctrl_op_type[i] = OP_TYPE_REAL_SPEED;
                        ctrl_op_value[i] = static_cast<int16_t>(speed_gbc[i] * 10.0f);
                    }
                }
                // 回中命令是 FPV 编码器域 (mtr=-op), 与 smoothAngleCommands 的
                // LOCK 相机姿态角域不同, 不得混入轨迹平滑 (会以错误域改写 op)
                if (!homing)
                    smoothAngleCommands(dt, true);
                sendControlPacket();

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    feedback_stamp = std::chrono::steady_clock::now();
                    orientation_stamp = node ? node->now() : rclcpp::Clock().now();
                    camera_pose_valid = vehicle_stamp != std::chrono::steady_clock::time_point{} &&
                        feedback_stamp - vehicle_stamp <= std::chrono::milliseconds(UAV_DATA_TIMEOUT_MS);
                    if (camera_pose_valid)
                    {
                        const double radians = 0.01 * M_PI / 180.0;
                        // 台架实测 (C-20S 立装, 2026-10-26): 俯仰编码器低头为正
                        // (cam_angle[1] ≈ -mtr_angle[0] + 慢漂移水平偏置), 需取反成
                        // 运动学约定的 FRD 抬头正。C-20S 无滚转/偏航电机 (mtr 恒 0), 其编码器符号
                        // 在本机不可测, 沿用原值 (C-40D/C-200T 待验)。
                        camera_orientation = apm_bridge::gimbalCameraToWorld(vehicle_orientation,
                            {last_response.mtr_angle[1] * radians, -last_response.mtr_angle[0] * radians,
                             last_response.mtr_angle[2] * radians},
                            {cfg.roll.present, cfg.pitch.present, cfg.yaw.present});
                        camera_heading = vehicle_heading;
                    }
                    // 陀螺域 (2026-10-26 台架手动激励实测, 三轮复验): cam_rate[0]+ = 低头 (=FLU 俯仰速率+,
                    // 与 cam_angle[1] 反号); cam_rate[1]+ = 右滚 (=FLU 滚转速率+, 与 cam_angle[0] 同号);
                    // cam_rate[2]+ = 右偏航 (罗盘式, FLU 偏航取反, 与 cam_angle[2] 同号)。
                    ema_filter.filter( last_response.cam_rate[1] * 0.1f, 0);    // Roll rate
                    ema_filter.filter( last_response.cam_rate[0] * 0.1f, 1);    // Pitch rate
                    ema_filter.filter(-last_response.cam_rate[2] * 0.1f, 2);    // Yaw rate
                }

                updateStatus();
            }
            else
            {
                if (!homing && control_mode == apm_bridge::GimbalMode::MISSION)
                    updateMissionControl(dt);
                // Angle trajectories must not advance without fresh encoder feedback.
                if (!homing)
                    smoothAngleCommands(dt, false);
                sendControlPacket();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void GimbalControl::smoothAngleCommands(double dt, bool fresh_feedback)
{
    const bool present[3] = {cfg.roll.present, cfg.pitch.present, cfg.yaw.present};
    std::array<bool, 3> axes{};
    std::array<double, 3> target{};
    for (size_t i = 0; i < axes.size(); ++i)
    {
        axes[i] = present[i] && ctrl_op_type[i] == OP_TYPE_ANGLE;
        target[i] = ctrl_op_value[i] * .01;
    }
    const int encoder[3] = {1, 0, 2};
    auto seed = angle_trajectory.positions();
    auto rates = angle_trajectory.rates();
    for (size_t i = 0; i < axes.size(); ++i)
        if (!angle_trajectory_ready || !angle_axes[i])
        {
            // 角度命令域: AIM/HORIZON 均为 LOCK 相机姿态角域 (抬头正), 俯仰从 cam_angle[1] 播种。
            // (FPV 相对角域与编码器反号: FPV op -> mtr = -op, 台架实测; 驱动现已不使用 FPV。)
            seed[i] = -last_response.mtr_angle[encoder[i]] * .01;
            if (i == 1)
                seed[i] = last_response.cam_angle[1] * .01;
            rates[i] = 0;
        }
    if (fresh_feedback)
    {
        angle_trajectory.reset(seed, rates);
        angle_trajectory_ready = true;
        angle_axes = axes;
        angle_trajectory.step(target, dt, axes);
        const GimbalAxisConfig limits[3] = {cfg.roll, cfg.pitch, cfg.yaw};
        for (size_t i = 0; i < axes.size(); ++i)
            if (control_mode == apm_bridge::GimbalMode::AIM)
                angle_trajectory.constrain(i, limits[i].min_deg, limits[i].max_deg);
    }
    if (!fresh_feedback && std::chrono::steady_clock::now() - feedback_stamp >
            std::chrono::milliseconds(UAV_DATA_TIMEOUT_MS))
    {
        angle_trajectory_ready = false;
        angle_axes = {};
        for (size_t i = 0; i < axes.size(); ++i)
        {
            ctrl_op_type[i] = OP_TYPE_REAL_SPEED;
            ctrl_op_value[i] = 0;
        }
        return;
    }
    for (size_t i = 0; i < axes.size(); ++i)
        if (axes[i])
        {
            if (!angle_trajectory_ready || !angle_axes[i])
            {
                ctrl_op_type[i] = OP_TYPE_REAL_SPEED;
                ctrl_op_value[i] = 0;
            }
            else
                ctrl_op_value[i] = static_cast<int16_t>(std::lround(angle_trajectory.positions()[i] * 100));
        }
}

void GimbalControl::setMode(apm_bridge::GimbalMode requested)
{
    std::lock_guard<std::mutex> lock(cmd_mtx);
    tracking_on = false;
    const auto previous = mode.exchange(requested);
    if (requested == apm_bridge::GimbalMode::AIM && previous != requested &&
        std::abs(aim_target_degrees[1] - (90.0 + cfg.pitch_aim_deg)) > 1e-6)
        RCLCPP_WARN(node ? node->get_logger() : rclcpp::get_logger("xfrobot_gimbal"),
            "AIM requested %.2f deg relative pitch, limited to %.2f deg; body FLU +Z is not reachable with this offset/limit",
            90.0 + cfg.pitch_aim_deg, aim_target_degrees[1]);
}

void GimbalControl::updateTargetLos(const apm_bridge::TargetLos &target)
{
    tracking_controller.update(target);
}

void GimbalControl::updateMissionControl(double dt)
{
    const double roll = std::abs(last_response.cam_angle[0] * 0.01);
    if (!roll_hold_active && roll > cfg.roll_hold_enter_deg)
        roll_hold_active = true;
    else if (roll_hold_active && roll < cfg.roll_hold_exit_deg)
        roll_hold_active = false;
    const bool no_feedback = std::chrono::steady_clock::now() - feedback_stamp >
        std::chrono::milliseconds(UAV_DATA_TIMEOUT_MS);
    const bool protection = cfg.model == GimbalModel::C20S && roll_hold_active;
    if (no_feedback || protection)
    {
        tracking_controller.reset();
        for (int i = 0; i < 3; ++i)
        {
            ctrl_op_type[i] = OP_TYPE_REAL_SPEED;
            ctrl_op_value[i] = 0;
        }
        if (node)
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                "MISSION gimbal stopped: feedback missing/stale or single-axis roll protection");
        return;
    }
    auto rates = tracking_controller.step(
        cfg.roll.present ? -last_response.cam_angle[0] * 0.01 * M_PI / 180.0 : 0.0, dt,
        apm_bridge::GimbalTrackingController::Clock::now(), {cfg.roll.present, cfg.pitch.present, cfg.yaw.present});
    if (node && !tracking_controller.fresh())
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "MISSION gimbal: target LOS missing/stale; decelerating to locked hold");
    const GimbalAxisConfig axes[3] = {cfg.roll, cfg.pitch, cfg.yaw};
    const int encoder[3] = {1, 0, 2};
    const double signs[3] = {1.0, -1.0, -1.0}; // FLU -> vendor roll / nose-up pitch / clockwise yaw.
    for (int i = 0; i < 3; ++i)
    {
        double speed = rates[i] * signs[i] * 180.0 / M_PI;
        // 编码器低头正: 取反到抬头域, 与限位 (FRD 抬头正) 和速度符号同域比较,
        // 否则门控方向相反会把云台推进机械限位 (台架实测)。
        const double angle = -last_response.mtr_angle[encoder[i]] * 0.01;
        // 俯仰用 MISSION 相对机身窗口 (FLU +Z 为 90°, 基准同 pitch_aim), 夹紧到机械行程;
        // 滚转/偏航用机械行程。门控只拦朝界外方向, 允许回界内。
        const double limit_lo = i == 1 ?
            std::max(90.0 + cfg.pitch_mission_min_deg, (double)cfg.pitch.min_deg) : axes[i].min_deg;
        const double limit_hi = i == 1 ?
            std::min(90.0 + cfg.pitch_mission_max_deg, (double)cfg.pitch.max_deg) : axes[i].max_deg;
        if (!axes[i].present || protection ||
            (limit_lo < limit_hi &&
             ((angle <= limit_lo && speed < 0) || (angle >= limit_hi && speed > 0))))
            speed = 0;
        speed = std::clamp(speed, -cfg.tracking.max_rate_dps, cfg.tracking.max_rate_dps);
        ctrl_op_type[i] = OP_TYPE_REAL_SPEED;
        ctrl_op_value[i] = static_cast<int16_t>(std::round(speed * 10.0));
    }
}

sensor_msgs::msg::Imu::UniquePtr GimbalControl::getOrientationFLU()
{
    return makeOrientationFLU(false);
}

sensor_msgs::msg::Imu::UniquePtr GimbalControl::makeOrientationFLU(bool world_frame)
{
    float omega[3];
    tf2::Quaternion camera_to_world;
    double heading_angle;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    {
        std::lock_guard<std::mutex> lock(mtx);
        const auto now = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::milliseconds(UAV_DATA_TIMEOUT_MS);
        if (!last_status.rx_ok || now - feedback_stamp > timeout || !camera_pose_valid ||
            vehicle_stamp == std::chrono::steady_clock::time_point{} || now - vehicle_stamp > timeout)
            return nullptr;
        omega[0] = ema_filter[0];
        omega[1] = ema_filter[1];
        omega[2] = ema_filter[2];
        camera_to_world = camera_orientation;
        heading_angle = camera_heading;
        stamp = orientation_stamp;
    }

    sensor_msgs::msg::Imu::UniquePtr imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = stamp;
    imu_msg->header.frame_id = world_frame ? "base_link" : "gimbal_link";
    const tf2::Quaternion heading(tf2::Vector3(0, 0, 1), heading_angle);
    const auto relative = heading.inverse() * camera_to_world;
    tf2::Matrix3x3(relative).getRPY(imu_msg->orientation_covariance[0],
        imu_msg->orientation_covariance[1], imu_msg->orientation_covariance[2]);
    imu_msg->orientation = tf2::toMsg(world_frame ? camera_to_world : relative);
    imu_msg->angular_velocity.x = omega[0] * M_PIf / 180.0f; // rad/s
    imu_msg->angular_velocity.y = omega[1] * M_PIf / 180.0f;
    imu_msg->angular_velocity.z = omega[2] * M_PIf / 180.0f;
    return imu_msg;
}

/**
 * @brief 打开并配置串口
 * @return true 成功, false 失败
 */
bool GimbalControl::open()
{
    if (is_open) return true;
    try
    {
        serial_port.open();
        is_open = serial_port.isOpen();
    }
    catch (const serial::IOException &e)
    {
        is_open = false;
    }
    catch (const std::invalid_argument &e)
    {
        is_open = false;
    }
    return is_open;
}

/**
 * @brief 关闭串口
 */
void GimbalControl::close()
{
    if (is_open)
    {
        serial_port.close();
        is_open = false;
    }
}

/**
 * @brief 检查串口是否打开
 */
bool GimbalControl::is_running() const
{
    return is_open && running.load();
}

void GimbalControl::start_stop(bool start)
{
    if (!is_open) return;

    Gcu2GbcPkt_t packet;
    memset(&packet, 0, sizeof(packet)); // 清空结构体

    // 1. 填充帧头
    packet.sync[0] = GCU_SYNC_1;
    packet.sync[1] = GCU_SYNC_2;

    // 2. 填充命令
    packet.cmd.value = start ? CMD_START : CMD_STOP;
    packet.cmd.trig = 0;

    uint16_t crc = CalculateCrc16((uint8_t *)&packet, sizeof(packet) - 2);
    packet.crc[0] = (crc >> 8) & 0xFF;
    packet.crc[1] = crc & 0xFF;

    // 3. 发送数据
    try
    {
        serial_port.write((uint8_t *)&packet, sizeof(packet));
    }
    catch (const serial::IOException &)
    {
    }
    catch (const serial::SerialException &)
    {
    }
}

// -------------------- 状态 --------------------

void GimbalControl::setStatusCallback(StatusCallback cb)
{
    std::lock_guard<std::mutex> lock(cmd_mtx);
    status_cb = std::move(cb);
}

GimbalStatus GimbalControl::getStatus()
{
    std::lock_guard<std::mutex> lock(mtx);
    return last_status;
}

void GimbalControl::updateStatus()
{
    GimbalStatus st;
    st.fw_ver = last_response.fw_ver;
    st.hw_err = last_response.hw_err;
    st.gbc_stat = last_response.gbc_stat;
    st.cmd_stat = last_response.cmd.stat;
    st.cmd_value = last_response.cmd.value;
    st.tca_ready = last_response.tca_flag;
    st.inv_flag = last_response.inv_flag;
    st.rx_ok = true;

    bool changed;
    {
        std::lock_guard<std::mutex> lock(mtx);
        changed = (st.gbc_stat != last_status.gbc_stat ||
                   st.hw_err != last_status.hw_err ||
                   st.tca_ready != last_status.tca_ready ||
                   st.inv_flag != last_status.inv_flag ||
                   st.cmd_stat != last_status.cmd_stat ||
                   st.cmd_value != last_status.cmd_value ||
                   !last_status.rx_ok);
        last_status = st;
    }
    if (changed)
    {
        std::lock_guard<std::mutex> lock(cmd_mtx);
        if (status_cb)
            status_cb(st);
    }
}

// -------------------- 命令 (V1.0.4) --------------------

bool GimbalControl::sendCommand(uint8_t cmd)
{
    if (cmd == CMD_NONE || cmd > CMD_TARGET_TRACK)
        return false;
    if (cmd == CMD_GYRO_CALIB)
    {
        // 协议要求温控就绪后方可校准陀螺仪; 尚未收到返回包时不拦截
        std::lock_guard<std::mutex> lock(mtx);
        if (last_status.rx_ok && !last_status.tca_ready)
            return false;
    }
    std::lock_guard<std::mutex> lock(cmd_mtx);
    pending_cmd = static_cast<int8_t>(cmd);
    return true;
}

void GimbalControl::requestGoZero(uint8_t axis_mask)
{
    uint8_t mask = axis_mask;
    if (!cfg.roll.present)
        mask &= ~(1u << 0);
    if (!cfg.pitch.present)
        mask &= ~(1u << 1);
    if (!cfg.yaw.present)
        mask &= ~(1u << 2);
    if (!mask)
        return;
    // 协议注释5: go_zero 位"变化"触发回中 → 每次请求恰好翻转一次
    std::lock_guard<std::mutex> lock(cmd_mtx);
    go_zero_bits ^= mask;
}

void GimbalControl::sendPointShift(float horiz_deg, float vert_deg)
{
    std::lock_guard<std::mutex> lock(cmd_mtx);
    pending_cmd = CMD_POINT_SHIFT;
    pending_target[0] = horiz_deg;
    pending_target[1] = vert_deg;
}

void GimbalControl::setTargetTracking(bool enable, float horiz_deg, float vert_deg)
{
    std::lock_guard<std::mutex> lock(cmd_mtx);
    tracking_on = enable;
    track_target[0] = horiz_deg;
    track_target[1] = vert_deg;
}

void GimbalControl::setUavData(bool valid, const int16_t angle_cdeg[3], const int16_t accel_cms2[3])
{
    std::lock_guard<std::mutex> lock(cmd_mtx);
    uav_valid = valid;
    if (valid)
    {
        memcpy(uav_angle, angle_cdeg, sizeof(uav_angle));
        memcpy(uav_accel, accel_cms2, sizeof(uav_accel));
        uav_stamp = std::chrono::steady_clock::now();
    }
}

// -------------------- 发送路径 --------------------

/**
 * @brief 工作线程的主发送路径: 按命令槽/跟踪状态/当前控制速度打包发送
 */
void GimbalControl::sendControlPacket()
{
    if (!is_open) return;

    // 快照命令槽 (一次性命令 > 目标跟踪 > 手动控制)
    int8_t one_shot = -1;
    float target[2] = {0.0f, 0.0f};
    bool tracking = false;
    uint8_t go_zero = 0;
    bool uav_send = false;
    int16_t uav_ang[3] = {0, 0, 0};
    int16_t uav_acc[3] = {0, 0, 0};
    {
        std::lock_guard<std::mutex> lock(cmd_mtx);
        if (pending_cmd >= 0)
        {
            one_shot = pending_cmd;
            pending_cmd = -1;
            target[0] = pending_target[0];
            target[1] = pending_target[1];
        }
        tracking = tracking_on;
        if (tracking)
        {
            target[0] = track_target[0];
            target[1] = track_target[1];
        }
        go_zero = go_zero_bits;
        if (cfg.send_uav_data && uav_valid &&
            std::chrono::steady_clock::now() - uav_stamp < std::chrono::milliseconds(UAV_DATA_TIMEOUT_MS))
        {
            uav_send = true;
            memcpy(uav_ang, uav_angle, sizeof(uav_ang));
            memcpy(uav_acc, uav_accel, sizeof(uav_acc));
        }
    }

    Gcu2GbcPkt_t packet;
    memset(&packet, 0, sizeof(packet)); // 清空结构体

    // 1. 填充帧头
    packet.sync[0] = GCU_SYNC_1;
    packet.sync[1] = GCU_SYNC_2;

    // 2. 填充命令
    if (one_shot >= 0)
        packet.cmd.value = static_cast<uint8_t>(one_shot);
    else if (tracking)
        packet.cmd.value = CMD_TARGET_TRACK;
    else
        packet.cmd.value = CMD_MANUAL_CONTROL;
    packet.cmd.trig = cmd_trig_counter++; // 递增触发计数器

    // FPV 跟随灵敏度 (台架实测: 0 时 ~0.2°/s 爬行, 15 干脆收敛)。AIM 已改用 LOCK
    // 同步实现, 驱动当前不再使用 FPV 模式; 保留条件以备将来启用。
    packet.aux.fl_sens = ctrl_wk_mode == WK_MODE_FPV ? 15 : 0;

    // 3. 填充三轴控制: 锁定模式; 手动控制路径按控制律给定的 op_type/op_value
    //    (C-40D/C-200T 真实角速度; C-20S 角度/速度保持切换)。
    //    命令5/6 期间速度清零 (云台用内置算法闭环);
    //    不存在的轴 op_value 恒 0 (语义=锁定, 固件忽略)
    const bool manual = (one_shot < 0) && !tracking;
    for (int i = 0; i < 3; i++)
    {
        packet.gbc[i].wk_mode = ctrl_wk_mode;
        packet.gbc[i].op_type = manual ? ctrl_op_type[i] : OP_TYPE_REAL_SPEED;
        packet.gbc[i].op_value = manual ? ctrl_op_value[i] : 0;
        packet.gbc[i].go_zero = (go_zero >> i) & 0x1;
    }

    // 4. 填充载机数据 (协议要求: 不提供准确数据时务必置无效)
    if (uav_send)
    {
        packet.uav.valid = 1;
        for (int i = 0; i < 3; i++)
        {
            packet.uav.angle[i] = uav_ang[i];
            packet.uav.accel[i] = uav_acc[i];
        }
    }

    // 5. 填充相机数据
    packet.cam.vert_fov1x = 30;    // 假设 30 度
    packet.cam.zoom_value = 1000;  // 1.000x
    packet.cam.target_angle[0] = target[0]; // 命令5/6 的控制量 (协议 V1.0.4 注释17)
    packet.cam.target_angle[1] = target[1];

    // 6. 计算并填充 CRC (大端序)
    uint16_t crc = CalculateCrc16((uint8_t *)&packet, sizeof(packet) - 2);
    packet.crc[0] = (crc >> 8) & 0xFF;
    packet.crc[1] = crc & 0xFF;

    // 7. 发送数据
    try
    {
        serial_port.write((uint8_t *)&packet, sizeof(packet));
    }
    catch (const serial::IOException &)
    {
    }
    catch (const serial::SerialException &)
    {
    }
}

/**
 * @brief 尝试读取并解析一个云台返回包
 * @return true 如果成功解析一个包, false 如果超时或数据错误
 */
bool GimbalControl::readPacket()
{
    if (!is_open) return false;

    uint8_t byte_in;
    try
    {
        // 循环读取字节，直到拼成一个完整的包或超时
        while (true)
        {
            // 1. 读取 1 字节
            if (serial_port.read(&byte_in, 1) == 0)
            {
                // 读取超时
                return false;
            }

            // 2. 状态机解析
            switch (rx_state)
            {
            case SYNC1:
                // 等待 0xB5
                if (byte_in == GBC_SYNC_1)
                {
                    rx_buffer[0] = byte_in;
                    rx_index = 1;
                    rx_state = SYNC2;
                }
                break;

            case SYNC2:
                // 等待 0x9A
                if (byte_in == GBC_SYNC_2)
                {
                    rx_buffer[1] = byte_in;
                    rx_index = 2;
                    rx_state = PAYLOAD;
                }
                else if (byte_in == GBC_SYNC_1)
                {
                    // 连续两个 0xB5: 当前字节可能才是帧头，停留在 SYNC2 重判
                    rx_buffer[0] = byte_in;
                    rx_index = 1;
                }
                else
                {
                    // 序列错误，重置
                    rx_state = SYNC1;
                }
                break;

            case PAYLOAD:
                // 接收剩余的包数据
                rx_buffer[rx_index++] = byte_in;

                // 检查是否已收满一个包
                if (rx_index >= sizeof(Gbc2GcuPkt_t))
                {
                    // 重置状态机
                    rx_index = 0;
                    rx_state = SYNC1;

                    // 3. 校验 CRC
                    Gbc2GcuPkt_t *pkt = (Gbc2GcuPkt_t *)rx_buffer;
                    uint16_t crc_calc = CalculateCrc16(rx_buffer, sizeof(Gbc2GcuPkt_t) - 2);
                    uint16_t crc_recv = (static_cast<uint16_t>(pkt->crc[0]) << 8) | pkt->crc[1];

                    if (crc_calc == crc_recv)
                    {
                        // CRC 校验成功
                        memcpy(&last_response, pkt, sizeof(Gbc2GcuPkt_t));
                        return true; // 成功解析
                    }
                    else
                    {
                        // std::cerr << "CRC 校验失败!" << std::endl;
                        // 继续等待下一个包
                    }
                }
                break;
            } // end switch
        } // end while
    }
    catch (const serial::IOException &e)
    {
        close(); // 发生严重错误，关闭串口 (由工作线程重连)
        return false;
    }
    catch (const serial::SerialException &e)
    {
        close(); // 发生严重错误，关闭串口 (由工作线程重连)
        return false;
    }
    return false;
}
