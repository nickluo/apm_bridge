#pragma once

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <tf2/LinearMath/Quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "serial/serial.h"
#include "ema_filter.h"


namespace xfrobot
{

// ======================================================================
// 协议定义 (来自 云台私有协议-XF(A5)V1.0.3.pdf P7, P10)
// ======================================================================

// 使用 1 字节对齐
#pragma pack(push, 1)

/**
 * @brief 上位机发送给云台的数据包 (GCU -> GBC)
 * @note 总长度: 40 字节
 */
typedef struct
{
    uint8_t sync[2]; // 0xA9 0x5B

    struct
    {
        uint8_t trig : 3; // 命令触发计数
        uint8_t value : 5; // 命令码
    } cmd;

    struct
    {
        uint8_t : 3;
        int8_t fl_sens : 5; // FPV 跟随灵敏度
    } aux;

    struct
    {
        uint8_t : 3;
        uint8_t go_zero : 1; // 回中触发
        uint8_t wk_mode : 2; // 工作模式 (0-跟随, 1-锁定, 2-FPV)
        uint8_t op_type : 2; // 控制模式 (0-角度, 1-比例角速度, 2-真实角速度)
        int16_t op_value;    // 云台控制量 (单位 0.01deg 或 0.1deg/s)
    } gbc[3];   // 0-滚转, 1-俯仰, 2-偏航

    struct
    {
        uint8_t : 7;
        uint8_t valid : 1;    // 载机惯导数据有效标志 (0-无效, 1-有效)
        int16_t angle[3]; // 载机姿态角 (0.01deg) [滚转, 俯仰, 偏航]
        int16_t accel[3]; // 载机加速度 (0.01m/s2) [北, 东, 天]
    } uav;

    struct
    {
        uint32_t vert_fov1x : 7; // 相机 1x 垂直视场角 (1deg)
        uint32_t zoom_value : 24; // 相机倍率 (0.001x)
        uint32_t reserved : 1;
        float target_angle[2]; // 目标偏移角度 (1deg) [水平, 垂直]
    } cam;

    uint8_t crc[2]; // CRC16 (大端序)
} Gcu2GbcPkt_t;

/**
 * @brief 云台返回给上位机的数据包 (GBC -> GCU)
 * @note 总长度: 26 字节
 */
typedef struct
{
    uint8_t sync[2]; // 0xB5 0x9A
    uint8_t fw_ver;  // 固件版本号
    uint8_t hw_err;  // 硬件故障

    uint8_t inv_flag : 1; // 吊装(0)/立装(1)标志
    uint8_t gbc_stat : 3; // 云台状态 (1-初始化, 2-停止, 3-保护, 4-手动控制)
    uint8_t tca_flag : 1; // 温控就绪标志 (0-未就绪, 1-已就绪)
    uint8_t : 3;

    struct
    {
        uint8_t stat : 3;  // 命令执行状态 (0-执行中, 1-成功, 2-失败)
        uint8_t value : 5; // 命令码回报
    } cmd;

    int16_t cam_rate[3];  // 相机本体角速度 (0.1deg/s) [俯仰, 滚转, 偏航]
    int16_t cam_angle[3]; // 相机姿态角 (0.01deg) [滚转, 俯仰, 偏航]
    int16_t mtr_angle[3]; // 相机相对角 (0.01deg) [俯仰, 滚转, 偏航]
    uint8_t crc[2];       // CRC16 (大端序)
} Gbc2GcuPkt_t;

// 恢复默认对齐
#pragma pack(pop)

class GimbalControl
{
public:
    /**
     * @brief 构造函数
     * @param port 串口端口号 (例如 "/dev/ttyUSB0" 或 "COM1")
     */
    explicit GimbalControl(const std::string &port);
    ~GimbalControl();

    void run();

    bool is_running() const;

    sensor_msgs::msg::Imu::UniquePtr getOrientationFLU();

private:
    std::string port_name;
    serial::Serial serial_port;
    bool is_open;
    std::atomic_bool running;

    // 发送相关
    uint8_t cmd_trig_counter;

    // 接收状态机
    enum RxState { SYNC1, SYNC2, PAYLOAD };
    RxState rx_state = SYNC1;
    uint8_t rx_buffer[sizeof(Gbc2GcuPkt_t)];
    size_t rx_index = 0;

    Gbc2GcuPkt_t last_response;

    std::mutex mtx;
    float euler_angles_FLU[3]; // 0-滚转, 1-俯仰, 2-偏航
    float omega_FLU[3];

    std::thread work_thread;

    apm_bridge::EMAFilter<float, 3> ema_filter {5};

    /**
     * @brief 打开并配置串口
     * @return true 成功, false 失败
     */
    bool open();
    /**
     * @brief 关闭串口
     */
    void close();

    void start_stop(bool start);

    void sendManualControl(float yaw_angle_deg, float pitch_angle_deg, float roll_angle_deg);

    void sendManualControl2(float yaw_dps, float pitch_dps, float roll_dps);

    /**
     * @brief 尝试读取并解析一个云台返回包
     * @return true 如果成功解析一个包, false 如果超时或数据错误
     */
    bool readPacket();
};

} // namespace xfrobot