#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "quadrotor_msgs/msg/control_command.hpp"
#include "quadrotor_msgs/msg/low_level_feedback.hpp"
#include "quadrotor_msgs/srv/cli_command.hpp"
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "custom_link/client.h"
#include "quadratic_thrust_model.h"
#include "ema_filter.h"
#include "gimbal_interface.h"

namespace custom_link_bridge
{
    /// Betaflight 自定义高频链路 (USE_CUSTOM_LINK) 版机载桥接节点。
    ///
    /// 与 msp_bridge::MspVehicleNode (已移除) 对外接口完全一致：话题名、消息
    /// 类型、控制量语义、RC 触发逻辑不变，下游 (drone_interceptor /
    /// airsim_tracker) 无需改动。链路升级为固件侧专用二进制协议：
    ///   - 遥测 200/100/10 Hz 与 GPS 有效时的 100 Hz 里程计，带 FC 时间戳，四时间戳软同步换算
    ///     到主机时基（取代 MSP 的事务中点估计）
    ///   - 控制直发角速度 + 油门 (0x20)，取代 MSP_SET_RAW_RC 摇杆量映射
    ///
    /// 外部控制通过自定义协议请求 OFFBOARD，或由飞手 BOXOFFBOARD 开关授权；
    /// 飞手接管、GPS Rescue 与固件解锁安全检查仍由 FC 仲裁。
    class CustomLinkVehicleNode : public rclcpp::Node
    {
    public:
        CustomLinkVehicleNode();
        ~CustomLinkVehicleNode() override;

    private:
        static constexpr double kBatteryFullVoltagePerCell = 4.2;
        static constexpr double kBatteryNominalVoltagePerCell = 3.7;
        static constexpr double kBatteryLowVoltagePerCell = 3.6;
        static constexpr double kBatteryCriticalVoltagePerCell = 3.3;
        static constexpr double kBatteryInvalidVoltagePerCell = 3.0;

        std::vector<std::string> mode_strings = {
            "ANGLE",
            "HORIZON",
            "",
            "ALT_HOLD",
            "",
            "POS_HOLD",
            "HEADFREE",
            "",
            "PASSTHRU",
            "",
            "FAILSAFE",
            "RTH",
            "AUTO",
            "OFFBOARD"
        };

        // ---- ROS 接口 ----
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
        rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_in_pub;
        rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub;
        rclcpp::Publisher<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_pub;
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub;
        rclcpp::Publisher<quadrotor_msgs::msg::LowLevelFeedback>::SharedPtr ap_feedback_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gimbal_imu_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gimbal_mode_pub;
        rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr trigger_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_pub;
        // ESC 转速遥测 (0x16 PayloadEscTelemetry 携带，100 Hz)；data[i] 为电机
        // i 机械转速 rpm（固件按 motor_poles 换算后下发），无遥测的电机为 NaN
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_rpm_pub;
        // GPS / mAh / 失效保护 (0x12 PayloadSlow 携带，10 Hz)
        rclcpp::Publisher<mavros_msgs::msg::GPSRAW>::SharedPtr gpsraw_pub;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr failsafe_pub;

        rclcpp::Subscription<quadrotor_msgs::msg::ControlCommand>::SharedPtr control_command_sub;
        rclcpp::Subscription<quadrotor_msgs::msg::ControlCommand>::SharedPtr control_command_raw_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_interface_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr offboard_interface_sub;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr tracker_state_sub;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr radar_state_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_state_sub;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr target_los_sub;
        rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr gimbal_trigger_sub;
        rclcpp::TimerBase::SharedPtr gimbal_mode_timer;

        // ---- CLI 设置读写服务 (/fpv/cli) ----
        // 节点用单线程 executor：服务回调用 rclcpp 的延迟响应形式（只收
        // header+request，由专职工作线程阻塞等待链路响应 0x24/0x15 后调
        // send_response 回包），慢调用/超时不阻塞控制与遥测回调。
        rclcpp::Service<quadrotor_msgs::srv::CliCommand>::SharedPtr cli_service;
        struct CliJob
        {
            std::shared_ptr<rmw_request_id_t> header;
            std::shared_ptr<quadrotor_msgs::srv::CliCommand::Request> request;
        };
        std::thread cli_worker;
        std::mutex cli_job_mtx;
        std::condition_variable cli_job_cv;
        std::deque<CliJob> cli_jobs;
        std::atomic_bool cli_worker_running{false};
        bool rx_bind_pending = false;       // 持 cli_job_mtx；link up 时置位
        bool cli_bind_rx_on_connect = false;
        double cli_timeout = 3.0;
        void cliServiceCallback(const std::shared_ptr<rmw_request_id_t> header,
                                const std::shared_ptr<quadrotor_msgs::srv::CliCommand::Request> request);
        void cliWorkerLoop();
        void maybeStartRxBind();
        void cliRespond(const std::shared_ptr<rmw_request_id_t> header,
                        std::shared_ptr<quadrotor_msgs::srv::CliCommand::Response> response);
        apm_bridge::GimbalModeSelector gimbal_mode_selector;
        apm_bridge::GimbalMode gimbal_mode = apm_bridge::GimbalMode::AIM;
        bool gimbal_homing = false; // 启动回中 (机械零点) 进行中: /fpv/gimbal_mode 发 "HOMING"
        std::atomic_bool gimbal_aim_requested{false}, gimbal_abort_requested{false};
        int64_t last_los_stamp = 0;
        int64_t last_los_clock = 0;

        rclcpp::TimerBase::SharedPtr diag_timer;
        rclcpp::TimerBase::SharedPtr cmd_push_timer;

        /// 按 link.control_rate 把最新控制量（含新鲜度判定）推给链路客户端。
        void pushControl();
        void updateGimbalMode();

        void ctrlCommandCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command);
        void ctrlCommandRawCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command);
        void armCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void offboardCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void rcTriggerUpdate(const uint16_t *channels, size_t count);

        // ---- 链路回调 (读取线程上下文) ----
        void handleFast(const custom_link::protocol::PayloadFast &p, int64_t hostUs);
        void handleMedium(const custom_link::protocol::PayloadMedium &p, int64_t hostUs);
        void handleSlow(const custom_link::protocol::PayloadSlow &p, int64_t hostUs);
        void handleOdometry(const custom_link::protocol::PayloadOdometry &p, int64_t hostUs);
        void handleEscTelemetry(const custom_link::protocol::PayloadEscTelemetry &p, int64_t hostUs);
        void publishImu(int64_t hostUs);
        void publishState(uint32_t mode_flags, bool armed);

        // ---- 参数 ----
        quadratic_thrust_model::MotorParams motor_params;
        double gravity = 9.80665;
        double mass = 2.0;
        int n_lipo_cells = 6;
        double battery_capacity_mah = 13000.0; // 电池整包容量 (mAh)，0 = 未知
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
        bool airsim_enabled = false;
        bool mission_gimbal_on_ground = false;

        // ---- 共享状态 ----
        std::mutex mtx_kp;
        double kp = 1.0;
        double baro_temperature_c = 20.0;
        double battery_voltage = 0.0;

        std::mutex mtx_cmd;
        std::array<double, 3> cmd_bodyrates{{0.0, 0.0, 0.0}}; // FLU rad/s
        double cmd_thrust = 0.0;                             // 0..1
        rclcpp::Time cmd_stamp{0, 0, RCL_ROS_TIME};
        bool cmd_valid = false;
        bool cmd_angle = false;
        bool set_unarm = false;

        // 姿态/IMU 快照（0x10 读取线程写，本线程发布）
        std::mutex mtx_att;
        double roll_flu = 0.0, pitch_flu = 0.0, yaw_enu = 0.0;
        std::array<double, 3> gyro_flu{{0.0, 0.0, 0.0}};
        std::array<double, 3> accel_flu{{0.0, 0.0, 0.0}};
        bool att_valid = false;

        std::atomic_bool armed{false};
        std::atomic_uint8_t landed_state{mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED};
        std::atomic_bool in_tracking{false};
        std::atomic_bool radar_tracking{false};
        std::atomic_bool in_mission{false};
        std::atomic<double> altitude_m{0.0};
        std::atomic<double> vertical_velocity_mps{0.0};
        apm_bridge::EMAFilter<double, 1> sitl_altitude_ema{20};
        double sitl_ground_altitude = 0.0;
        bool sitl_ground_valid = false;
        bool armed_feedback_valid = false;
        double previous_altitude_m = 0.0;
        int64_t previous_altitude_us = 0;
        double landing_altitude = 0.2;
        double takeoff_confirm_time_s = 0.5;
        double landing_confirm_time_s = 1.0;
        uint8_t landed_candidate_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
        int64_t landed_candidate_since_us = 0;

        std::atomic_bool in_hover{false};
        std::atomic_bool use_rate{false};
        std::atomic_bool rc_manual{true};

        apm_bridge::EMAFilter<double, 1> ema{50};
        std::unique_ptr<apm_bridge::Gimbal> gimbal;
    };

} // namespace custom_link_bridge
