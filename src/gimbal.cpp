#include "gimbal.h"

#include <rclcpp/rclcpp.hpp>

using namespace xfrobot;

// 协议常量
#define GCU_SYNC_1 0xA9
#define GCU_SYNC_2 0x5B
#define GBC_SYNC_1 0xB5
#define GBC_SYNC_2 0x9A

// 命令码 (P8, 注释3)
#define CMD_MANUAL_CONTROL 4 // 手动控制
#define CMD_START 2
#define CMD_STOP 3;

// 工作模式 (P8, 注释6)
#define WK_MODE_FOLLOW 0 // 跟随模式
#define WK_MODE_LOCK 1   // 锁定模式
#define WK_MODE_FPV 2    // FPV 模式

// 控制模式 (P9, 注释7)
#define OP_TYPE_ANGLE 0           // 角度控制
#define OP_TYPE_RATIO_SPEED 1   // 比例角速度
#define OP_TYPE_REAL_SPEED 2    // 真实角速度

#define SERIAL_READ_TIMEOUT_MS 10
#define PITCH_ROLL_SPEED_SCALE  12.0f // 速度控制比例系数
#define YAW_SPEED_SCALE         10.0f // 速度控制比例系数

#define MAX_ANGLE_SPEED         150.0f // deg/s
#define MIN_PITCH_LIMIT        -135.0f // deg
#define MAX_PITCH_LIMIT         40.0f  // deg
#define MAX_ROLL_LIMIT          50.0f  // deg 

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

GimbalControl::GimbalControl(const std::string &port)
        : port_name(port), cmd_trig_counter(0), is_open(false), running(false)
{
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

        // printf("====================== Gimbal control thread started.\n");
        float yaw_exp = 0.0f;
        float pitch_exp = 0.0f;
        float roll_exp = 0.0f;

        std::chrono::high_resolution_clock::time_point last_time;
        bool first_run = true;
        float last_yaw_encoder = 0.0f;
        float omega_yaw = 0.0f;
        float delta = 0.01f;

        start_stop(true);
        while (running.load())
        {
            // 尝试读取返回包
            if (readPacket())
            {
                auto current_time = std::chrono::high_resolution_clock::now();
                
                roll_exp = -last_response.cam_angle[0]  * 0.01f * PITCH_ROLL_SPEED_SCALE;
                if (roll_exp > MAX_ANGLE_SPEED)
                    roll_exp = MAX_ANGLE_SPEED;
                else if (roll_exp < -MAX_ANGLE_SPEED)
                    roll_exp = -MAX_ANGLE_SPEED;
                pitch_exp = -last_response.cam_angle[1] * 0.01f * PITCH_ROLL_SPEED_SCALE;
                if (pitch_exp > MAX_ANGLE_SPEED)
                    pitch_exp = MAX_ANGLE_SPEED;
                else if (pitch_exp < -MAX_ANGLE_SPEED)
                    pitch_exp = -MAX_ANGLE_SPEED;

                float current_yaw_encoder = last_response.mtr_angle[2] * 0.01f;

                float mtr_roll = last_response.mtr_angle[1] * 0.01f;
                bool max = false;
                if (std::abs(mtr_roll) >= MAX_ROLL_LIMIT)
                {
                    roll_exp = 0.0f;
                    max = true;
                }
                float mtr_pitch = last_response.mtr_angle[0] * 0.01f;
                if (std::abs(last_response.mtr_angle[0] - last_response.cam_angle[1]) < MAX_PITCH_LIMIT*100 && (mtr_pitch <= MIN_PITCH_LIMIT || mtr_pitch >= MAX_PITCH_LIMIT))
                {
                    pitch_exp = 0.0f;
                    max = true;
                }
                
                // printf("MTR_ROLL: %.2f MTR_PITCH: %.2f | CMD_ROLL_EXP: %.2f CMD_PITCH_EXP: %.2f\n",
                //     mtr_roll, mtr_pitch, roll_exp, pitch_exp);
                
                float r_ratio = 1.0f - std::abs(mtr_roll) / MAX_ROLL_LIMIT;
                float p_ratio = 1.0f - mtr_pitch / (std::signbit(mtr_pitch) ? MIN_PITCH_LIMIT : MAX_PITCH_LIMIT);
                float scale = std::min(r_ratio, p_ratio);
                // scale *= scale; // 二次降低控制增益
                if (scale < 0.01f)
                    scale = 0.01f;
                else if (scale > 1.0f)
                    scale = 1.0f;

                if (first_run)
                {
                    first_run = false;
                }
                else
                {
                    delta = std::chrono::duration_cast<std::chrono::duration<float, std::chrono::seconds::period>>(current_time - last_time).count();
                    omega_yaw = -(current_yaw_encoder - last_yaw_encoder) / delta; // deg/s
                }
                last_time = current_time;
                last_yaw_encoder = current_yaw_encoder;
                
                if (max)
                {
                    // Keep low speed when reaching limit
                    yaw_exp = std::signbit(current_yaw_encoder) ? MAX_ANGLE_SPEED*0.01f : -MAX_ANGLE_SPEED*0.01f;
                }
                else
                {
                    yaw_exp = -current_yaw_encoder * YAW_SPEED_SCALE * scale;
                    if (std::abs(yaw_exp) < 0.1f)
                        yaw_exp = 0.0f;
                    else if (yaw_exp > MAX_ANGLE_SPEED)
                        yaw_exp = MAX_ANGLE_SPEED;
                    else if (yaw_exp < -MAX_ANGLE_SPEED)
                        yaw_exp = -MAX_ANGLE_SPEED;
                }

                // printf("Current Yaw rate: %.2f deg/s  Expected Yaw rate: %.2f deg/s\n", omega_yaw, yaw_exp);

                // 发送控制命令
                sendManualControl2(yaw_exp, pitch_exp, roll_exp);
                
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    euler_angles_FLU[0] =   last_response.cam_angle[0] * 0.01f;
                    euler_angles_FLU[1] = - last_response.cam_angle[1] * 0.01f;
                    euler_angles_FLU[2] = - current_yaw_encoder;
                    ema_filter.filter( last_response.cam_rate[0] * 0.1f, 0);
                    ema_filter.filter(-last_response.cam_rate[1] * 0.1f, 1);
                    ema_filter.filter(omega_yaw, 2);
                    // printf("Roll rate: %.2f Pitch rate: %.2f Yaw rate: %.2f deg/s\n", 
                    //     ema_filter[0], ema_filter[1], ema_filter[2]);
                }

                // printf("%-6d | %-12.2f | %-12.2f | %-12.2f\n",
                //        (int)last_response.gbc_stat,
                //        last_response.cam_angle[0] * 0.01f,
                //        last_response.cam_angle[1] * 0.01f,
                //        last_response.cam_angle[2] * 0.01f);
            }
            else
            {
                // 无法读取到包，保持当前期望值不变
                sendManualControl2(yaw_exp, pitch_exp, roll_exp);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

sensor_msgs::msg::Imu::UniquePtr GimbalControl::getOrientationFLU()
{
    float yaw_rad_half, roll_rad_half, pitch_rad_half;
    float omega_yaw, omega_pitch, omega_roll;

    {
        std::lock_guard<std::mutex> lock(mtx);
        // 1. 将角度从度转换为弧度
        yaw_rad_half = euler_angles_FLU[2] * M_PIf / 180.0f  * 0.5f;
        roll_rad_half = euler_angles_FLU[0] * M_PIf / 180.0f * 0.5f;
        pitch_rad_half = euler_angles_FLU[1] * M_PIf / 180.0f * 0.5f;
        omega_yaw = ema_filter[2];
        omega_pitch = ema_filter[1];
        omega_roll = ema_filter[0];
    }

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
    // Use orientation_covariance to store raw euler angles in YRP order for further processing
    imu_msg->orientation_covariance[0] = euler_angles_FLU[0] * M_PIf / 180.0f;
    imu_msg->orientation_covariance[1] = euler_angles_FLU[1] * M_PIf / 180.0f;
    imu_msg->orientation_covariance[2] = euler_angles_FLU[2] * M_PIf / 180.0f;
    imu_msg->angular_velocity.x = omega_roll * M_PIf / 180.0f; // rad/s
    imu_msg->angular_velocity.y = omega_pitch * M_PIf / 180.0f;
    imu_msg->angular_velocity.z = omega_yaw * M_PIf / 180.0f;
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
        // std::cerr << "无法打开串口 " << port_name << ": " << e.what() << std::endl;
        is_open = false;
    }
    catch (const std::invalid_argument &e)
    {
        // std::cerr << "串口参数无效: " << e.what() << std::endl;
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

    // 7. 发送数据
    try
    {
        serial_port.write((uint8_t *)&packet, sizeof(packet));
    }
    catch (const serial::IOException &e)
    {
        // std::cerr << "串口写入失败: " << e.what() << std::endl;
    }
}

/**
 * @brief 发送一个控制包
 * @param yaw_angle_deg 偏航轴期望角度 (单位: deg)
 * @param pitch_angle_deg 俯仰轴期望角度 (单位: deg)
 * @param roll_angle_deg 滚转轴期望角度 (单位: deg)
 */
void GimbalControl::sendManualControl(float yaw_angle_deg, float pitch_angle_deg, float roll_angle_deg)
{
    if (!is_open) return;

    Gcu2GbcPkt_t packet;
    memset(&packet, 0, sizeof(packet)); // 清空结构体

    // 1. 填充帧头
    packet.sync[0] = GCU_SYNC_1;
    packet.sync[1] = GCU_SYNC_2;

    // 2. 填充命令
    packet.cmd.value = CMD_MANUAL_CONTROL;
    packet.cmd.trig = cmd_trig_counter++; // 递增触发计数器

    packet.aux.fl_sens = -5; // 假设 FPV 灵敏度为 10

    // 3. 填充三轴控制 (参考 P19 和 P26 示例)
    
    // 滚转轴: 锁定模式 + 角度控制
    packet.gbc[0].wk_mode = WK_MODE_FPV; // WK_MODE_LOCK;
    packet.gbc[0].op_type = OP_TYPE_ANGLE;
    packet.gbc[0].op_value = static_cast<int16_t>(roll_angle_deg * 100.0f);

    // 俯仰轴: 锁定模式 + 角度控制 (P19 示例用角速度，这里改为角度)
    packet.gbc[1].wk_mode = WK_MODE_FPV;
    packet.gbc[1].op_type = OP_TYPE_ANGLE;
    packet.gbc[1].op_value = static_cast<int16_t>(pitch_angle_deg * 100.0f);

    // 偏航轴: 跟随模式 + 真实角速度控制 (P19 示例)
    // (注: 跟随模式下，速度为0表示跟随载机，非0表示在载机基础上转动)
    // (如果想地面锁定，偏航轴也应用锁定模式)
    packet.gbc[2].wk_mode = WK_MODE_FPV; // 改为锁定模式
    packet.gbc[2].op_type = OP_TYPE_ANGLE; // OP_TYPE_REAL_SPEED;
    packet.gbc[2].op_value = static_cast<int16_t>(yaw_angle_deg * 100.0f);

    // packet.gbc[2].wk_mode = WK_MODE_LOCK; // 改为锁定模式
    // packet.gbc[2].op_type = OP_TYPE_ANGLE;
    // packet.gbc[2].op_value = static_cast<int16_t>(yaw_speed_dps * 10.0f);

    // 4. 填充载机数据 (重要: 示例中设为无效)
    packet.uav.valid = 0; // 0 = 无效
    // 如果 valid = 1, 则必须填充正确的 angle 和 accel

    // 5. 填充相机数据 (可选)
    packet.cam.vert_fov1x = 30; // 假设 30 度
    packet.cam.zoom_value = 1000; // 1.000x

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
        // std::cerr << "串口写入失败: " << e.what() << std::endl;
    }
}

void GimbalControl::sendManualControl2(float yaw_dps, float pitch_dps, float roll_dps)
{
    if (!is_open) return;

    Gcu2GbcPkt_t packet;
    memset(&packet, 0, sizeof(packet)); // 清空结构体

    // 1. 填充帧头
    packet.sync[0] = GCU_SYNC_1;
    packet.sync[1] = GCU_SYNC_2;

    // 2. 填充命令
    packet.cmd.value = CMD_MANUAL_CONTROL;
    packet.cmd.trig = cmd_trig_counter++; // 递增触发计数器

    packet.aux.fl_sens = 0;

    // 3. 填充三轴控制 (参考 P19 和 P26 示例)
    
    // 滚转轴
    packet.gbc[0].wk_mode = WK_MODE_LOCK;
    packet.gbc[0].op_type = OP_TYPE_REAL_SPEED; //OP_TYPE_ANGLE;
    packet.gbc[0].op_value = static_cast<int16_t>(roll_dps * 10.0f);

    // 俯仰轴
    packet.gbc[1].wk_mode = WK_MODE_LOCK;
    packet.gbc[1].op_type = OP_TYPE_REAL_SPEED;
    packet.gbc[1].op_value = static_cast<int16_t>(pitch_dps * 10.0f);

    // 偏航轴
    packet.gbc[2].wk_mode = WK_MODE_LOCK;
    packet.gbc[2].op_type = OP_TYPE_REAL_SPEED;
    packet.gbc[2].op_value = static_cast<int16_t>(yaw_dps * 10.0f);

    // 4. 填充载机数据 (重要: 示例中设为无效)
    packet.uav.valid = 0; // 0 = 无效
    // 如果 valid = 1, 则必须填充正确的 angle 和 accel

    // 5. 填充相机数据 (可选)
    packet.cam.vert_fov1x = 30; // 假设 30 度
    packet.cam.zoom_value = 1000; // 1.000x

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
        // std::cerr << "串口写入失败: " << e.what() << std::endl;
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
        // std::cerr << "串口读取异常: " << e.what() << std::endl;
        close(); // 发生严重错误，关闭串口
        return false;
    }
    return false;
}

/**
 * @brief 获取最后一次成功接收到的云台状态
 */
// Gbc2GcuPkt_t GimbalControl::getResponse() const
// {
//     return last_response;
// }

