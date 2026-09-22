#pragma once

#include <string>
#include <cstdint>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "serial/serial.h"
#include "ema_filter.h"


namespace xfrobot
{

// ======================================================================
// 协议定义 (来自 云台私有协议-XF(A5)V1.0.4.pdf P2, P6)
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
        uint32_t vert_fov1x : 7; // 相机 1x 倍率的垂直视场角 (1deg)
        uint32_t zoom_value : 24; // 相机倍率 (0.001x)
        uint32_t reserved : 1;
        float target_angle[2]; // 目标偏离画面中心的角度 (1deg) [水平右正, 垂直下正] (V1.0.4 注释17, 命令5/6 的控制量)
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
    uint8_t gbc_stat : 3; // 云台状态 (1-初始化, 2-停止, 3-保护, 4-手动控制, 5-指点平移)
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

// ======================================================================
// 云台机型 (协议相同, 区别在轴数)
// ======================================================================

/**
 * @brief 支持的云台机型
 *        C-20S: 单轴 (仅俯仰)
 *        C-40D: 卧式两轴 (俯仰 + 滚转)
 *        C-200T: 三轴 (滚转 + 俯仰 + 偏航)
 */
enum class GimbalModel { C20S, C40D, C200T };

/**
 * @brief 机型名解析 ("C-20S"/"C20S"/"c20s" 等大小写与连字符不敏感)
 * @throw std::invalid_argument 未知的机型名
 */
GimbalModel gimbalModelFromString(const std::string &name);

/**
 * @brief 单轴配置
 */
struct GimbalAxisConfig
{
    bool present = true;  // 该轴是否存在电机
    float min_deg = 0.0f; // 机械行程下限 (deg)
    float max_deg = 0.0f; // 机械行程上限 (deg)
};

/**
 * @brief 云台整体配置
 */
struct GimbalConfig
{
    GimbalModel model = GimbalModel::C200T;
    GimbalAxisConfig roll;
    GimbalAxisConfig pitch;
    GimbalAxisConfig yaw;

    // 控制增益 (沿用原硬编码的现场调参数值, 仅 C-40D/C-200T 使用)
    float pitch_roll_speed_scale = 12.0f; // 俯仰/滚转速度控制比例系数
    float yaw_speed_scale = 10.0f;        // 偏航速度控制比例系数
    float max_angle_speed = 150.0f;       // 最大角速度 (deg/s)

    // ---- C-20S 单轴俯仰逻辑 (其余机型不使用) ----
    // 常态: 锁定+角度控制, 保持期望俯仰姿态角 pitch_hold_deg (0=水平);
    // 云台 IMU 滚转角 |cam_angle[0]| > roll_hold_enter 时 (载机大坡度/倒飞,
    // 单轴无法维持水平且可能触发倾角保护): 切换锁定+真实角速度控制,
    // op_value=0 锁定当前俯仰角;
    // |滚转| < roll_hold_exit 后恢复角度控制 (可将 exit 设小于 enter 构成迟滞)
    float pitch_hold_deg = 0.0f;
    float roll_hold_enter_deg = 75.0f;
    float roll_hold_exit_deg = 75.0f;

    // V1.0.4 载机惯导数据融合。协议要求提供不准确的载机数据时必须置 valid=0，
    // 因此默认关闭，实机验证符号约定后再开启。
    bool send_uav_data = false;

    /**
     * @brief 按机型生成默认配置
     * @note 行程限位手册正文未给数值 (在图纸)，默认沿用三轴机的现场调参数值，
     *       可通过 ROS 参数按机型覆盖。飞行前务必按图纸实测确认。
     */
    static GimbalConfig defaultsFor(GimbalModel model);

    /**
     * @brief 从 ROS 参数读取配置 (两段式: 先读 gimbal.model，再按机型默认值声明其余参数)
     *
     * 参数: gimbal.model ("C200T"|"C40D"|"C20S")，
     *       gimbal.roll_min/roll_max, gimbal.pitch_min/pitch_max,
     *       gimbal.yaw_min/yaw_max (deg),
     *       gimbal.pitch_roll_speed_scale, gimbal.yaw_speed_scale,
     *       gimbal.max_angle_speed, gimbal.send_uav_data,
     *       gimbal.pitch_hold, gimbal.roll_hold_enter, gimbal.roll_hold_exit (仅 C-20S)
     */
    static GimbalConfig fromParameters(rclcpp::Node &node);
};

/**
 * @brief 云台状态快照 (来自 GBC 返回包)
 */
struct GimbalStatus
{
    uint8_t fw_ver = 0;    // 固件版本号
    uint8_t hw_err = 0;    // 硬件故障 (非 0 需返厂)
    uint8_t gbc_stat = 0;  // 云台状态 (0-未定义, 1-初始化, 2-停止, 3-保护, 4-手动控制, 5-指点平移)
    uint8_t cmd_stat = 0;  // 命令执行状态 (0-执行中, 1-成功, 2-失败)
    uint8_t cmd_value = 0; // 命令码回报
    bool tca_ready = false; // 温控就绪 (陀螺仪校准前提)
    bool inv_flag = false;  // 吊装(false)/立装(true)
    bool rx_ok = false;     // 是否收到过有效返回包
};

class GimbalControl
{
public:
    /**
     * @brief 构造函数 (按 C-200T 三轴默认配置，兼容旧调用)
     * @param port 串口端口号 (例如 "/dev/ttyUSB0" 或 "COM1")
     */
    explicit GimbalControl(const std::string &port);
    /**
     * @brief 构造函数
     * @param port 串口端口号
     * @param cfg 云台机型配置
     */
    GimbalControl(const std::string &port, const GimbalConfig &cfg);
    ~GimbalControl();

    void run();

    bool is_running() const;

    sensor_msgs::msg::Imu::UniquePtr getOrientationFLU();

    // -------------------- 状态 --------------------

    using StatusCallback = std::function<void(const GimbalStatus &)>;
    /**
     * @brief 注册状态回调 (在云台工作线程中触发，仅在状态变化时调用)
     */
    void setStatusCallback(StatusCallback cb);
    /**
     * @brief 获取最近一次状态快照
     */
    GimbalStatus getStatus();

    // -------------------- 命令 (V1.0.4) --------------------

    /**
     * @brief 发送一次性命令 (在工作线程中发出一包)
     * @param cmd 命令码: 1-陀螺仪校准, 2-启动云台, 3-停止云台
     * @return false 参数不合法，或陀螺仪校准时温控未就绪
     */
    bool sendCommand(uint8_t cmd);

    /**
     * @brief 请求逐轴回中 (协议注释5: go_zero 位变化触发回中，每次调用翻转一次)
     * @param axis_mask bit0-滚转, bit1-俯仰, bit2-偏航 (不存在的轴自动忽略)
     */
    void requestGoZero(uint8_t axis_mask);

    /**
     * @brief 指点平移 (命令5): 使目标点移向画面中心，一次性
     * @param horiz_deg 目标水平偏移角度 (右为正, deg)
     * @param vert_deg 目标垂直偏移角度 (下为正, deg)
     */
    void sendPointShift(float horiz_deg, float vert_deg);

    /**
     * @brief 云台内置目标跟踪 (命令6, V1.0.4 开放): 持续以目标偏离画面中心的
     *        角度驱动云台内置控制算法
     * @param enable 关闭后恢复手动控制 (命令4)
     * @param horiz_deg 目标水平偏移角度 (右为正, deg)
     * @param vert_deg 目标垂直偏移角度 (下为正, deg)
     */
    void setTargetTracking(bool enable, float horiz_deg, float vert_deg);

    /**
     * @brief 更新载机惯导数据 (uav 融合入口，仅 cfg.send_uav_data 为真时打包发送)
     * @param valid 数据有效标志 (无效/过期时云台包中 valid 置 0)
     * @param angle_cdeg 载机姿态角 (0.01deg) [滚转, 俯仰, 偏航]
     * @param accel_cms2 载机加速度 (0.01m/s2) [北, 东, 天]
     */
    void setUavData(bool valid, const int16_t angle_cdeg[3], const int16_t accel_cms2[3]);

private:
    std::string port_name;
    serial::Serial serial_port;
    bool is_open;
    std::atomic_bool running;
    GimbalConfig cfg;

    // 发送相关
    uint8_t cmd_trig_counter;
    // 当前控制量 (仅工作线程读写), gbc 槽位顺序 [0-滚转, 1-俯仰, 2-偏航]。
    // op_type: 0-角度(0.01deg) 2-真实角速度(0.1deg/s), 已按单位编码进 op_value
    uint8_t ctrl_op_type[3];
    int16_t ctrl_op_value[3];
    bool roll_hold_active = false; // C-20S: 大滚转角速度保持模式标志

    // 命令槽 (外部线程写, 工作线程消费; 维持串口单写者纪律)
    std::mutex cmd_mtx;
    int8_t pending_cmd = -1;           // 待发一次性命令 (命令码, -1 无)
    float pending_target[2] = {0, 0};  // 命令5 的目标偏移 (deg) [水平, 垂直]
    bool tracking_on = false;          // 命令6 目标跟踪使能
    float track_target[2] = {0, 0};    // 命令6 的目标偏移 (deg) [水平, 垂直]
    uint8_t go_zero_bits = 0;          // 各轴 go_zero 位 (bit0-滚转 bit1-俯仰 bit2-偏航)，变化触发回中
    StatusCallback status_cb;          // 状态回调

    // 载机惯导数据缓存
    bool uav_valid = false;
    int16_t uav_angle[3] = {0, 0, 0};
    int16_t uav_accel[3] = {0, 0, 0};
    std::chrono::steady_clock::time_point uav_stamp;

    // 接收状态机
    enum RxState { SYNC1, SYNC2, PAYLOAD };
    RxState rx_state = SYNC1;
    uint8_t rx_buffer[sizeof(Gbc2GcuPkt_t)];
    size_t rx_index = 0;

    Gbc2GcuPkt_t last_response;

    std::mutex mtx;
    float euler_angles_FLU[3]; // 0-滚转, 1-俯仰, 2-偏航
    GimbalStatus last_status;

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

    /**
     * @brief 工作线程的主发送路径: 按命令槽/跟踪状态/当前控制速度打包发送
     */
    void sendControlPacket();

    /**
     * @brief 尝试读取并解析一个云台返回包
     * @return true 如果成功解析一个包, false 如果超时或数据错误
     */
    bool readPacket();

    /**
     * @brief 由 last_response 更新状态快照，变化时触发回调 (工作线程)
     */
    void updateStatus();
};

} // namespace xfrobot
