#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include "quadrotor_msgs/msg/control_command.hpp"
#include "quadrotor_msgs/msg/low_level_feedback.hpp"
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/waypoint_list.hpp>
#include <mavros_msgs/msg/status_text.hpp>
#include <mavros_msgs/msg/param_event.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>

#include "quadratic_thrust_model.h"
#include "ema_filter.h"
#include "gimbal.h"

#include <mavros_msgs/srv/command_bool.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <mavros_msgs/srv/command_home.hpp>

namespace apm_bridge
{
    class VehicleNode : public rclcpp::Node
    {
    public:
        VehicleNode();
        ~VehicleNode();

    private:
        static constexpr double kBatteryFullVoltagePerCell = 4.2;
        static constexpr double kBatteryLowVoltagePerCell = 3.6;
        static constexpr double kBatteryCriticalVoltagePerCell = 3.3;
        static constexpr double kBatteryInvalidVoltagePerCell = 3.0;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_br_static;

        rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr target_pub;
        rclcpp::Publisher<quadrotor_msgs::msg::LowLevelFeedback>::SharedPtr ap_feedback_pub;
        rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr fpv_pressure_pub;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr fpv_temperature_pub;

        rclcpp::Subscription<quadrotor_msgs::msg::ControlCommand>::SharedPtr control_command_sub;
        rclcpp::Subscription<quadrotor_msgs::msg::ControlCommand>::SharedPtr control_command_raw_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr atti_target_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_interface_sub;
        rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr atm_sub;
        rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub;
        rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
        rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_in_sub;
        rclcpp::Subscription<mavros_msgs::msg::ParamEvent>::SharedPtr param_event_sub;

        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr tracker_state_sub;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr radar_state_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_state_sub;
        rclcpp::Subscription<mavros_msgs::msg::WaypointList>::SharedPtr waypoint_list_sub;
        rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr set_global_pos_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gimbal_imu_pub;
        rclcpp::Publisher<mavros_msgs::msg::StatusText>::SharedPtr status_pub;
        rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr trigger_pub;

        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr command_arming;
        
        quadratic_thrust_model::MotorParams motor_params;

        double gravity;
        bool simulation;

        std::mutex mtx_kp;
        double kp = 1.0;
        double temp = 0.0;
        double baro = 101325.0;
        double battery_voltage = 0;
        double collective_force;
        double mass;

        int n_lipo_cells;
        int channel_trigger;

        uint8_t hoverable = 0; // 0: n/a 1: internal 2: external
        bool in_hover = false;

        std::atomic_bool armed = false;
        std::atomic_bool in_tracking = false;
        std::atomic_bool radar_tracking = false;
        std::atomic_bool in_mission = false;
        std::atomic_uint8_t landed_state;
        bool use_rate = false;
        bool rc_manual = true;
        
        bool voltage_compensation = false;
       
        EMAFilter<double, 5> ema;

        double current_heading = 0.0;

        std::unique_ptr<xfrobot::GimbalControl> gimbal;

        // ---- 云台状态 (仅云台工作线程的回调中读写) ----
        uint8_t gimbal_last_stat = 0;
        bool gimbal_last_tca = false;

        void setupMavlink();

        void armCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void ctrlCommandRawCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command);
        void ctrlCommandCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command);
        void attiTargetCallback(const mavros_msgs::msg::AttitudeTarget::SharedPtr att);
        void atmPressureCallback(const sensor_msgs::msg::FluidPressure::SharedPtr val);
        void tempCallback(const sensor_msgs::msg::Temperature::SharedPtr val);
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr val);
        void extStateCallback(const mavros_msgs::msg::ExtendedState::SharedPtr state);
        void stateCallback(const mavros_msgs::msg::State::SharedPtr state);
        void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr state);
        void rcInCallback(const mavros_msgs::msg::RCIn::SharedPtr rc);
        void paramEventCallback(const mavros_msgs::msg::ParamEvent::SharedPtr event);

        rclcpp::TimerBase::SharedPtr gimbal_timer;
        rclcpp::TimerBase::SharedPtr status_timer;
        int status_counter_ = 0;
        // rclcpp::TimerBase::SharedPtr sync_timer_;
        // void syncWorkerCallback();

        template<typename T>
        bool executeService(
            const typename rclcpp::Client<T>::SharedPtr & client,
            const typename T::Request::SharedPtr & request)
        {
            auto future = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
                return false;
            }
            return future.get()->success;
        }

    };
}
