#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int8.hpp>
#include "quadrotor_msgs/msg/control_command.hpp"
#include "quadrotor_msgs/msg/low_level_feedback.hpp"
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "custom_link/client.h"
#include "quadratic_thrust_model.h"
#include "ema_filter.h"
#include "gimbal.h"

namespace custom_link_bridge
{
    /// Betaflight 自定义高频链路 (USE_CUSTOM_LINK) 版机载桥接节点。
    ///
    /// 与 msp_bridge::MspVehicleNode (已移除) 对外接口完全一致：话题名、消息
    /// 类型、控制量语义、RC 触发逻辑不变，下游 (drone_interceptor /
    /// airsim_tracker) 无需改动。链路升级为固件侧专用二进制协议：
    ///   - 遥测 200/100/10 Hz 三条流，带 FC 时间戳，四时间戳软同步换算
    ///     到主机时基（取代 MSP 的事务中点估计）
    ///   - 控制直发角速度 + 油门 (0x20)，取代 MSP_SET_RAW_RC 摇杆量映射
    ///
    /// 外部控制要求 Betaflight 侧打开 OFFBOARD 模式 (BOXOFFBOARD 开关)：
    /// 开关在飞手遥控器上，激活后 FC 才受理主机解锁/接管。
    class CustomLinkVehicleNode : public rclcpp::Node
    {
    public:
        CustomLinkVehicleNode();
        ~CustomLinkVehicleNode() override;

    private:
        static constexpr double kBatteryFullVoltagePerCell = 4.2;
        static constexpr double kBatteryLowVoltagePerCell = 3.6;
        static constexpr double kBatteryCriticalVoltagePerCell = 3.3;
        static constexpr double kBatteryInvalidVoltagePerCell = 3.0;

        // ---- ROS 接口 ----
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_in_pub;
        rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub;
        rclcpp::Publisher<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_pub;
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub;
        rclcpp::Publisher<quadrotor_msgs::msg::LowLevelFeedback>::SharedPtr ap_feedback_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gimbal_imu_pub;
        rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr trigger_pub;

        rclcpp::Subscription<quadrotor_msgs::msg::ControlCommand>::SharedPtr control_command_sub;
        rclcpp::Subscription<quadrotor_msgs::msg::ControlCommand>::SharedPtr control_command_raw_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_interface_sub;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr tracker_state_sub;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr radar_state_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_state_sub;

        rclcpp::TimerBase::SharedPtr gimbal_timer;
        rclcpp::TimerBase::SharedPtr diag_timer;
        rclcpp::TimerBase::SharedPtr cmd_push_timer;

        /// 把最新控制量（含新鲜度判定）推给链路客户端；50 Hz。
        void pushControl();

        void ctrlCommandCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command);
        void ctrlCommandRawCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command);
        void armCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void rcTriggerUpdate(const uint16_t *channels, size_t count);

        // ---- 链路回调 (读取线程上下文) ----
        void handleFast(const custom_link::protocol::PayloadFast &p, int64_t hostUs);
        void handleMedium(const custom_link::protocol::PayloadMedium &p, int64_t hostUs);
        void handleSlow(const custom_link::protocol::PayloadSlow &p, int64_t hostUs);
        void publishImu(int64_t hostUs);
        void publishState(uint32_t mode_flags, bool armed);

        // ---- 参数 ----
        quadratic_thrust_model::MotorParams motor_params;
        double gravity = 9.80665;
        double mass = 2.0;
        int n_lipo_cells = 6;
        int channel_trigger = 6;
        bool voltage_compensation = false;
        bool baro_compensation = true;

        std::string imu_frame_id = "base_link";
        double orientation_variance = 0.0;
        double angular_velocity_variance = 0.0;
        double linear_acceleration_variance = 0.0;

        double control_timeout = 0.5;
        double airborne_altitude = 0.5;
        double rate_limit_dps = 0.0;      // 发送前角速度限幅, 0 = 不限制

        std::unique_ptr<custom_link::Client> client;

        // ---- 共享状态 ----
        std::mutex mtx_kp;
        double kp = 1.0;
        double battery_voltage = 0.0;

        std::mutex mtx_cmd;
        std::array<double, 3> cmd_bodyrates{{0.0, 0.0, 0.0}}; // FLU rad/s
        double cmd_thrust = 0.0;                             // 0..1
        rclcpp::Time cmd_stamp{0, 0, RCL_ROS_TIME};
        bool cmd_valid = false;
        bool set_unarm = false;

        // 姿态/IMU 快照（0x10 读取线程写，本线程发布）
        std::mutex mtx_att;
        double roll_flu = 0.0, pitch_flu = 0.0, yaw_enu = 0.0;
        std::array<double, 3> gyro_flu{{0.0, 0.0, 0.0}};
        std::array<double, 3> accel_flu{{0.0, 0.0, 0.0}};
        bool att_valid = false;

        std::atomic_bool armed{false};
        std::atomic_uint8_t hoverable{0}; // 0: 不可外控 1: 自稳 2: OFFBOARD
        std::atomic_uint8_t landed_state{mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED};
        std::atomic_bool in_tracking{false};
        std::atomic_bool radar_tracking{false};
        std::atomic_bool in_mission{false};
        std::atomic<double> altitude_m{0.0};

        std::atomic_bool in_hover{false};
        std::atomic_bool use_rate{false};
        std::atomic_bool rc_manual{true};
        std::atomic<double> current_heading{0.0};

        apm_bridge::EMAFilter<double, 1> ema{50};
        std::unique_ptr<xfrobot::GimbalControl> gimbal;

        // ---- 云台状态 (仅云台工作线程的回调中读写) ----
        bool gimbal_uav_fusion = false; // 载机惯导数据融合开关 (参数 gimbal.send_uav_data)
        uint8_t gimbal_last_stat = 0;
        bool gimbal_last_tca = false;
    };

} // namespace custom_link_bridge
