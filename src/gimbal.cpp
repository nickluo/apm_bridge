#include "gimbal.h"

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

GimbalConfig GimbalConfig::fromParameters(rclcpp::Node &node)
{
    // 第一段: 机型名决定轴存在性与默认限位
    node.declare_parameter<std::string>("gimbal_model", "C200T");
    std::string model_name;
    node.get_parameter("gimbal_model", model_name);
    GimbalConfig cfg = defaultsFor(gimbalModelFromString(model_name));

    // 第二段: 其余参数按机型默认值声明，允许 yaml/launch 逐项覆盖
    auto param_f = [&node](const std::string &name, float def) -> float {
        node.declare_parameter<double>(name, static_cast<double>(def));
        return static_cast<float>(node.get_parameter(name).as_double());
    };
    cfg.roll.min_deg = param_f("gimbal.roll_min", cfg.roll.min_deg);
    cfg.roll.max_deg = param_f("gimbal.roll_max", cfg.roll.max_deg);
    cfg.pitch.min_deg = param_f("gimbal.pitch_min", cfg.pitch.min_deg);
    cfg.pitch.max_deg = param_f("gimbal.pitch_max", cfg.pitch.max_deg);
    cfg.yaw.min_deg = param_f("gimbal.yaw_min", cfg.yaw.min_deg);
    cfg.yaw.max_deg = param_f("gimbal.yaw_max", cfg.yaw.max_deg);
    cfg.pitch_roll_speed_scale = param_f("gimbal.pitch_roll_speed_scale", cfg.pitch_roll_speed_scale);
    cfg.yaw_speed_scale = param_f("gimbal.yaw_speed_scale", cfg.yaw_speed_scale);
    cfg.max_angle_speed = param_f("gimbal.max_angle_speed", cfg.max_angle_speed);
    cfg.pitch_hold_deg = param_f("gimbal.pitch_hold", cfg.pitch_hold_deg);
    cfg.roll_hold_enter_deg = param_f("gimbal.roll_hold_enter", cfg.roll_hold_enter_deg);
    cfg.roll_hold_exit_deg = param_f("gimbal.roll_hold_exit", cfg.roll_hold_exit_deg);

    node.declare_parameter<bool>("gimbal.send_uav_data", cfg.send_uav_data);
    node.get_parameter("gimbal.send_uav_data", cfg.send_uav_data);

    return cfg;
}

GimbalControl::GimbalControl(const std::string &port)
    : GimbalControl(port, GimbalConfig::defaultsFor(GimbalModel::C200T))
{
}

GimbalControl::GimbalControl(const std::string &port, const GimbalConfig &gimbal_cfg)
    : port_name(port), cmd_trig_counter(0), is_open(false), running(false), cfg(gimbal_cfg)
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
GimbalControl::~GimbalControl()
{
    if (work_thread.joinable())
    {
        running = false;
        work_thread.join();
    }
    start_stop(false); // 停止云台
    close();
}

void GimbalControl::run()
{
    if (!is_open || running.load())
        return;

    running = true;

    work_thread = std::thread([&](){

        // 丢弃上次会话遗留在内核缓冲中的旧应答，避免启动瞬间消化陈旧状态
        try
        {
            serial_port.flushInput();
        }
        catch (const serial::IOException &e)
        {
        }

        start_stop(true);
        while (running.load())
        {
            // 尝试读取返回包
            if (readPacket())
            {
                // ---- 控制律 (按机型门控; C-40D/C-200T 与原实现逐字节一致) ----
                if (cfg.model == GimbalModel::C20S)
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

                    // 滚转轴: P 控制器驱动相机姿态角回水平
                    if (cfg.roll.present)
                    {
                        roll_exp = -last_response.cam_angle[0] * 0.01f * cfg.pitch_roll_speed_scale;
                        roll_exp = std::clamp(roll_exp, -cfg.max_angle_speed, cfg.max_angle_speed);

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
                        pitch_exp = -last_response.cam_angle[1] * 0.01f * cfg.pitch_roll_speed_scale;
                        pitch_exp = std::clamp(pitch_exp, -cfg.max_angle_speed, cfg.max_angle_speed);

                        float mtr_pitch = last_response.mtr_angle[0] * 0.01f;
                        if (std::abs(last_response.mtr_angle[0] - last_response.cam_angle[1]) < static_cast<int>(cfg.pitch.max_deg * 100) && (mtr_pitch <= cfg.pitch.min_deg || mtr_pitch >= cfg.pitch.max_deg))
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
                            yaw_exp = std::signbit(current_yaw_encoder) ? cfg.max_angle_speed * 0.01f : -cfg.max_angle_speed * 0.01f;
                        }
                        else
                        {
                            yaw_exp = -current_yaw_encoder * cfg.yaw_speed_scale * scale;
                            if (std::abs(yaw_exp) < 0.1f)
                                yaw_exp = 0.0f;
                            else
                                yaw_exp = std::clamp(yaw_exp, -cfg.max_angle_speed, cfg.max_angle_speed);
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
                sendControlPacket();

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    euler_angles_FLU[0] =   last_response.cam_angle[0] * 0.01f;
                    euler_angles_FLU[1] = - last_response.cam_angle[1] * 0.01f;
                    // 无偏航电机 (C-20S/C-40D) 时相机偏航随载体, 编码器不存在
                    euler_angles_FLU[2] = cfg.yaw.present ? -last_response.mtr_angle[2] * 0.01f : 0.0f;
                    ema_filter.filter( last_response.cam_rate[1] * 0.1f, 0);    // Roll rate
                    ema_filter.filter( last_response.cam_rate[0] * 0.1f, 1);    // Pitch rate
                    ema_filter.filter(-last_response.cam_rate[2] * 0.1f, 2);    // Yaw rate
                }

                updateStatus();
            }
            else
            {
                // 无法读取到包，保持当前期望值不变 (维持通信频率)
                sendControlPacket();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

sensor_msgs::msg::Imu::UniquePtr GimbalControl::getOrientationFLU()
{
    // 锁内拷贝局部量, 避免锁外再读共享数组
    float euler[3], omega[3];
    {
        std::lock_guard<std::mutex> lock(mtx);
        euler[0] = euler_angles_FLU[0];
        euler[1] = euler_angles_FLU[1];
        euler[2] = euler_angles_FLU[2];
        omega[0] = ema_filter[0];
        omega[1] = ema_filter[1];
        omega[2] = ema_filter[2];
    }

    // 1. 将角度从度转换为弧度 (半角)
    float yaw_rad_half   = euler[2] * M_PIf / 180.0f * 0.5f;
    float roll_rad_half  = euler[0] * M_PIf / 180.0f * 0.5f;
    float pitch_rad_half = euler[1] * M_PIf / 180.0f * 0.5f;

    // 2. 计算半角的 sin 和 cos
    float cy = std::cos(yaw_rad_half);
    float sy = std::sin(yaw_rad_half);
    float cr = std::cos(roll_rad_half);
    float sr = std::sin(roll_rad_half);
    float cp = std::cos(pitch_rad_half);
    float sp = std::sin(pitch_rad_half);
    // 3. 根据 Z-X-Y 顺序计算四元数
    // q = q_z * q_x * q_y
    tf2::Quaternion q;
    q.setW(cp * cy * cr - sp * sy * sr);
    q.setX(cp * cy * sr - sp * sy * cr);
    q.setY(sp * cy * cr + cp * sy * sr);
    q.setZ(sp * cy * sr + cp * sy * cr);

    sensor_msgs::msg::Imu::UniquePtr imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = rclcpp::Clock().now();
    imu_msg->header.frame_id = "gimbal_link";
    imu_msg->orientation.x = q.x();
    imu_msg->orientation.y = q.y();
    imu_msg->orientation.z = q.z();
    imu_msg->orientation.w = q.w();
    // Use orientation_covariance to store raw FLU euler angles in RPY order for further processing
    imu_msg->orientation_covariance[0] = euler[0] * M_PIf / 180.0f;
    imu_msg->orientation_covariance[1] = euler[1] * M_PIf / 180.0f;
    imu_msg->orientation_covariance[2] = euler[2] * M_PIf / 180.0f;
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
    catch (const serial::IOException &e)
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

    packet.aux.fl_sens = 0;

    // 3. 填充三轴控制: 锁定模式; 手动控制路径按控制律给定的 op_type/op_value
    //    (C-40D/C-200T 真实角速度; C-20S 角度/速度保持切换)。
    //    命令5/6 期间速度清零 (云台用内置算法闭环);
    //    不存在的轴 op_value 恒 0 (语义=锁定, 固件忽略)
    const bool manual = (one_shot < 0) && !tracking;
    for (int i = 0; i < 3; i++)
    {
        packet.gbc[i].wk_mode = WK_MODE_LOCK;
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
    catch (const serial::IOException &e)
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
    catch (const serial::SerialException &e)
    {
        close(); // 发生严重错误，关闭串口
        return false;
    }
    return false;
}
