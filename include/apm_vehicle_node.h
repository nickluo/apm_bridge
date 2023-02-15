#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <quadrotor_msgs/LowLevelFeedback.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/MessageInterval.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <string>

#include "quadratic_thrust_model.h"
#include "ema_filter.h"

// #define DEFAULT_TSC_FREQ      31249700

namespace apm_bridge
{
    class VehicleNode
    {
    public:
        VehicleNode(ros::NodeHandle &handle);
        ~VehicleNode();
        void Spin();
    private:
        // const double tsc_1_div_freq = 1000000.0 / DEFAULT_TSC_FREQ;

        static constexpr double kBatteryLowVoltagePerCell = 3.6;
        static constexpr double kBatteryCriticalVoltagePerCell = 3.4;
        static constexpr double kBatteryInvalidVoltagePerCell = 3.0;

        ros::NodeHandle nh;
        bool sync_running = true;
        std::thread sync_worker;

        tf2_ros::TransformBroadcaster tf_br;
        tf2_ros::StaticTransformBroadcaster tf_br_static;

        tf2::Quaternion q0;
        tf2::Quaternion q1;
        tf2::Vector3 v0;
        tf2::Vector3 v1;

        ros::Publisher target_pub;
        ros::Publisher ap_feedback_pub;

        ros::CallbackQueue queue_control;
        ros::CallbackQueue queue_fast;
        ros::CallbackQueue queue_slow;

        ros::Subscriber control_command_sub;
        ros::Subscriber control_command_raw_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber atti_target_sub;

        ros::Subscriber arm_interface_sub;
        ros::Subscriber atm_sub;
        ros::Subscriber temp_sub;
        ros::Subscriber ext_state_sub;
        ros::Subscriber state_sub;
        ros::Subscriber battery_sub;

        ros::ServiceClient command_arming;

        // ros::ServiceClient set_message_interval;

        // std::string frame_id_base_link;
        // std::string frame_id_axis;
        // std::string frame_id_lio;
        
        quadratic_thrust_model::MotorParams motor_params;

        double gravity;

        std::mutex mtx_kp;
        double kp = 1.0;
        double temp = 15.0;
        double baro = 101325.0;
        double battery_voltage = 0;
        double collective_force;
        double mass;

        int n_lipo_cells;

        uint8_t hoverable = 0; // 0: n/a 1: internal 2: external

        // std::vector<double> ex_r_i_a;
        // std::vector<double> ex_r_a_v;
        // std::vector<double> ex_t_i_a;
        // std::vector<double> ex_t_a_v;

        std::atomic_bool armed = false;
        std::atomic_uint8_t landed_state;
        bool use_rate = false;
        bool rc_manual = true;
        
        bool voltage_compensation = false;
       
        EMAFilter<5> ema;
        
        void setupMavlink();

        // void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void armCallback(const std_msgs::Bool::ConstPtr &msg);
        void ctrlCommandRawCallback(const quadrotor_msgs::ControlCommandConstPtr &command);
        void ctrlCommandCallback(const quadrotor_msgs::ControlCommandConstPtr &command);
        void attiTargetCallback(const mavros_msgs::AttitudeTargetConstPtr &att);
        void atmPressureCallback(const sensor_msgs::FluidPressureConstPtr &val);
        void tempCallback(const sensor_msgs::TemperatureConstPtr &val);
        void imuCallback(const sensor_msgs::ImuConstPtr &val);
        void extStateCallback(const mavros_msgs::ExtendedStateConstPtr &state);
        void stateCallback(const mavros_msgs::StateConstPtr &state);
        void batteryCallback(const sensor_msgs::BatteryStateConstPtr &state);

    };
}


