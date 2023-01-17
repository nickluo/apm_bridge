#include "apm_vehicle_node.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <cstdlib>

#include <mavlink/v2.0/ardupilotmega/ardupilotmega.hpp>

using namespace apm_bridge;

namespace mavlink_msg = mavlink::common::msg;

VehicleNode::VehicleNode(ros::NodeHandle &handle)
    : nh(handle), ema(10)
{
    nh.param("gravity_const", gravity, 9.80665);
    nh.param("mass", mass, 2.0);

    nh.param("lipo_cells", n_lipo_cells, 6);
    nh.param("voltage_compensation", voltage_compensation, false);

    nh.param("motor_parameters/a", motor_params.A, 0.0);
    nh.param("motor_parameters/b", motor_params.B, 0.0);
    nh.param("motor_parameters/c", motor_params.C, 0.0);
    nh.param("motor_parameters/n", motor_params.n_motors, 1);
    nh.param("motor_parameters/voltage_a", motor_params.voltage_map_a, 0.0);
    nh.param("motor_parameters/voltage_b", motor_params.voltage_map_b, 0.0);

    // nh.param("extrinsic_R_IinA", ex_r_i_a, std::vector<double>(9, 0));
    // nh.param("extrinsic_R_AinV", ex_r_a_v, std::vector<double>(9, 0));
    // nh.param("extrinsic_T_IinA", ex_t_i_a, std::vector<double>(3, 0));
    // nh.param("extrinsic_T_AinV", ex_t_a_v, std::vector<double>(3, 0));

    // nh.param("base_link_frame", frame_id_base_link, {});
    // nh.param("axis_frame", frame_id_axis, {});
    // nh.param("lio_frame", frame_id_lio, {});

    // tf2::Matrix3x3 mat0(ex_r_a_v[0], ex_r_a_v[1], ex_r_a_v[2],
    //                     ex_r_a_v[3], ex_r_a_v[4], ex_r_a_v[5],
    //                     ex_r_a_v[6], ex_r_a_v[7], ex_r_a_v[8]);
    // tf2::Matrix3x3 mat1(ex_r_i_a[0], ex_r_i_a[1], ex_r_i_a[2],
    //                     ex_r_i_a[3], ex_r_i_a[4], ex_r_i_a[5],
    //                     ex_r_i_a[6], ex_r_i_a[7], ex_r_i_a[8]);
    // mat0.getRotation(q0);
    // mat1.getRotation(q1);
    // v0.setValue(ex_t_a_v[0], ex_t_a_v[1], ex_t_a_v[2]);
    // v1.setValue(ex_t_i_a[0], ex_t_i_a[1], ex_t_i_a[2]);

    // geometry_msgs::TransformStamped tf_axis2base;
    // tf_axis2base.header.stamp = ros::Time::now();
    // tf_axis2base.header.frame_id = frame_id_base_link;
    // tf_axis2base.child_frame_id = frame_id_axis;
    // tf2::convert(q0, tf_axis2base.transform.rotation);
    // tf2::convert(v0, tf_axis2base.transform.translation);

    // tf_br_static.sendTransform(tf_axis2base);

    control_command_sub = nh.subscribe("control_command", 10, &VehicleNode::ctrlCommandCallback, this);
    control_command_raw_sub = nh.subscribe("control_command_raw", 10, &VehicleNode::ctrlCommandRawCallback, this);
    atm_sub = nh.subscribe("/mavros/imu/static_pressure", 10, &VehicleNode::atmPressureCallback, this);
    temp_sub = nh.subscribe("/mavros/imu/temperature_baro", 10, &VehicleNode::tempCallback, this);
    imu_sub = nh.subscribe("/mavros/imu/data", 10, &VehicleNode::imuCallback, this);
    ext_state_sub = nh.subscribe("/mavros/extended_state", 10, &VehicleNode::extStateCallback, this);
    state_sub = nh.subscribe("/mavros/state", 10, &VehicleNode::stateCallback, this);
    atti_target_sub = nh.subscribe("/mavros/setpoint_raw/target_attitude", 10, &VehicleNode::attiTargetCallback, this);
    battery_sub = nh.subscribe("/mavros/battery", 10, &VehicleNode::batteryCallback, this);

    target_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    ap_feedback_pub = nh.advertise<quadrotor_msgs::LowLevelFeedback>("low_level_feedback", 10);

    setupMavlink();

    // sync_worker = std::thread([this]()
    // {
    //     std::fstream ivc("/sys/devices/aon_echo/data_channel");
    //     if (ivc.peek()==0)
    //     {
    //         ivc<<"ENABLE\n";
    //         printf("============== ENABLE aon_echo/data_channel ==============\n");
    //     }
    //     std::string line;
    //     char *endptr = nullptr;
    //     int64_t last_tsc = 0;
    //     ros::Rate rate(100);
    //     while(sync_running)
    //     {
    //         ivc<<"REPORT\n";
    //         ivc.flush();
    //         rate.sleep();
    //         int i = 0;
    //         while (i++<5)
    //         {
    //             ivc.seekg(0, ivc.beg);
    //             std::getline(ivc, line);
    //             if (line.compare(0, 4, "RES ")==0)
    //             {
    //                 const char *str = line.c_str()+4;
    //                 int64_t res_tsc = std::strtol(str, &endptr, 10);
    //                 if (res_tsc == last_tsc)
    //                 {
    //                     rate.sleep();
    //                     continue;
    //                 }
    //                 last_tsc = res_tsc;
    //                 str = endptr;
    //                 float pos = std::strtof(str, &endptr);
    //                 str = endptr;
    //                 {
    //                     std::lock_guard<std::mutex> lk(mtx_kp);
    //                     temp = std::strtof(str, &endptr);
    //                     kp = (baro/101325.0) * (273.15/(273.15+temp));
    //                 }

    //                 int64_t current_tsc;
    //                 timespec current_time;
    //                 timespec_get(&current_time, TIME_UTC);
    //                 asm volatile("mrs %0, cntvct_el0":"=r"(current_tsc));
    //                 current_time.tv_nsec -= (int64_t)((current_tsc - res_tsc)*tsc_1_div_freq*1000);
    //                 if (current_time.tv_nsec<0l)
    //                 {
    //                     --current_time.tv_sec;
    //                     current_time.tv_nsec = 1000000000l + current_time.tv_nsec;
    //                 }

    //                 // geometry_msgs::TransformStamped tf_lio;
    //                 // tf_lio.header.stamp = ros::Time(current_time.tv_sec, current_time.tv_nsec);
    //                 // tf_lio.header.frame_id = frame_id_axis;
    //                 // tf_lio.child_frame_id = frame_id_lio;
    //                 // tf2::convert(v1, tf_lio.transform.translation);

    //                 // tf2::Quaternion qx;
    //                 // qx.setRPY(pos/180.0*M_PI, 0, 0);
    //                 // tf2::convert(qx*q1, tf_lio.transform.rotation);
    //                 // tf_br.sendTransform(tf_lio);
    //             }
    //         }
    //         // std::this_thread::sleep_for(std::chrono::microseconds(100));
    //     } });
}

VehicleNode::~VehicleNode()
{
    sync_running = false;
    if (sync_worker.joinable())
        sync_worker.join();
}

void VehicleNode::setupMavlink()
{
    auto set_message_interval = nh.serviceClient<mavros_msgs::MessageInterval>("/mavros/set_message_interval");
    set_message_interval.waitForExistence();
    mavros_msgs::MessageInterval mesg;
    mesg.request.message_id = mavlink_msg::ATTITUDE_TARGET::MSG_ID;
    mesg.request.message_rate = 10.0f;
    set_message_interval.call(mesg);
    mesg.request.message_id = mavlink_msg::EXTENDED_SYS_STATE::MSG_ID;
    mesg.request.message_rate = 5.0f;
    set_message_interval.call(mesg);
    mesg.request.message_id = mavlink_msg::SCALED_PRESSURE::MSG_ID;
    mesg.request.message_rate = 10.0f;
    set_message_interval.call(mesg);
    mesg.request.message_id = mavlink_msg::ATTITUDE_QUATERNION::MSG_ID;
    mesg.request.message_rate = 100.0f;
    set_message_interval.call(mesg);
    mesg.request.message_id = mavlink_msg::SCALED_IMU::MSG_ID;
    mesg.request.message_rate = 200.0f;
    set_message_interval.call(mesg);
    mesg.request.message_id = mavlink_msg::BATTERY_STATUS::MSG_ID;
    mesg.request.message_rate = 1.0f;
    set_message_interval.call(mesg);
}

void VehicleNode::extStateCallback(const mavros_msgs::ExtendedStateConstPtr &state)
{
    landed_state = state->landed_state;
}

void VehicleNode::stateCallback(const mavros_msgs::StateConstPtr &state)
{
    mavlink::minimal::MAV_STATE current_status{state->system_status};
    if (current_status == mavlink::minimal::MAV_STATE::ACTIVE)
    {
    }
    else if (current_status == mavlink::minimal::MAV_STATE::STANDBY)
    {
    }
    if (state->mode == mavros_msgs::State::MODE_APM_COPTER_LOITER || state->mode == mavros_msgs::State::MODE_APM_COPTER_POSHOLD)
    {
        hoverable = 1;
        rc_manual = true;
    }
    else if (state->mode == mavros_msgs::State::MODE_APM_COPTER_GUIDED || state->mode == mavros_msgs::State::MODE_APM_COPTER_GUIDED_NOGPS)
    {
        hoverable = 2;
        rc_manual = false;
    }
    else
    {
        hoverable = 0;
        rc_manual = true;
    }
}

void VehicleNode::attiTargetCallback(const mavros_msgs::AttitudeTargetConstPtr &att)
{
    if (hoverable == 1) // in Loiter or PosHold mode
    {
        std::lock_guard<std::mutex> lk(mtx_kp);
        if (voltage_compensation)
            collective_force = quadratic_thrust_model::thrustToForce(motor_params, att->thrust, kp, battery_voltage);
        else
            collective_force = quadratic_thrust_model::thrustToForce(motor_params, att->thrust, kp);
    }
}

void VehicleNode::imuCallback(const sensor_msgs::ImuConstPtr &val)
{
    static uint8_t last_state = mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;

    if (last_state != landed_state)
    {
        last_state = landed_state;
        ema.reset();
    }

    if (landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) // Update g value
    {
        auto acc = std::sqrt(val->linear_acceleration.x * val->linear_acceleration.x +
                             val->linear_acceleration.y * val->linear_acceleration.y +
                             val->linear_acceleration.z * val->linear_acceleration.z);
        gravity = ema.filter(acc, 2);
    }
    else if (landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR) // Update mass when hovering
    {
        auto r_roll = ema.filter(val->angular_velocity.x, 0);
        auto r_pitch = ema.filter(val->angular_velocity.y, 1);

        // acceleration without gravity
        tf2::Matrix3x3 mat(tf2::Quaternion(val->orientation.x, val->orientation.y, val->orientation.z, val->orientation.w));
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        tf2::Vector3 acc;
        acc.setY(val->linear_acceleration.y - std::sin(roll) * std::cos(pitch) * gravity);
        acc.setZ(val->linear_acceleration.z - std::cos(roll) * std::cos(pitch) * gravity);
        acc.setX(val->linear_acceleration.x + std::sin(pitch) * gravity);

        if (std::fabs(r_roll) < 0.1 && std::fabs(r_pitch) < 0.1 && ema.filter(acc.length2(), 4) < 0.01) // if hovering
        {
            mass = ema.filter(collective_force / gravity, 3);
        }
    }
}

void VehicleNode::ctrlCommandRawCallback(const quadrotor_msgs::ControlCommandConstPtr &command)
{
    if (command->armed == 0)
        return;
    auto target = mavros_msgs::AttitudeTargetPtr(new mavros_msgs::AttitudeTarget);
    switch (command->control_mode)
    {
    case quadrotor_msgs::ControlCommand::ATTITUDE:
        target->type_mask = mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        target->orientation = command->orientation;
        use_rate = false;
        break;
    case quadrotor_msgs::ControlCommand::BODY_RATES:
        target->type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        target->body_rate = command->bodyrates;
        use_rate = true;
        break;
    default:
        break;
    }

    target->thrust = (command->collective_thrust < 0.0) ? 0.0f : ((command->collective_thrust > 1.0) ? 1.0f : (float)command->collective_thrust);

    target_pub.publish(target);
}

void VehicleNode::ctrlCommandCallback(const quadrotor_msgs::ControlCommandConstPtr &command)
{
    if (command->armed == 0)
        return;
    auto target = mavros_msgs::AttitudeTargetPtr(new mavros_msgs::AttitudeTarget);
    switch (command->control_mode)
    {
    case quadrotor_msgs::ControlCommand::ATTITUDE:
        target->type_mask = mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        target->orientation = command->orientation;
        use_rate = false;
        break;
    case quadrotor_msgs::ControlCommand::BODY_RATES:
        target->type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        target->body_rate = command->bodyrates;
        use_rate = true;
        break;
    default:
        break;
    }

    double thrust = 0.0;
    collective_force = command->collective_thrust * mass;
    {
        std::lock_guard<std::mutex> lk(mtx_kp);
        if (voltage_compensation)
            thrust = quadratic_thrust_model::forceToThrust(motor_params, collective_force, kp, battery_voltage);
        else
            thrust = quadratic_thrust_model::forceToThrust(motor_params, collective_force, kp);
    }
    target->thrust = (thrust < 0.0) ? 0.0f : ((thrust > 1.0) ? 1.0f : (float)thrust);

    target_pub.publish(target);
}

void VehicleNode::atmPressureCallback(const sensor_msgs::FluidPressureConstPtr &val)
{
    std::lock_guard<std::mutex> lk(mtx_kp);
    baro = val->fluid_pressure;
    kp = (baro / 101325.0) * (273.15 / (273.15 + temp));
}

void VehicleNode::tempCallback(const sensor_msgs::TemperatureConstPtr &val)
{
    std::lock_guard<std::mutex> lk(mtx_kp);
    temp = val->temperature - 20.0;
}

void VehicleNode::batteryCallback(const sensor_msgs::BatteryStateConstPtr &state)
{
    quadrotor_msgs::LowLevelFeedbackPtr msg(new quadrotor_msgs::LowLevelFeedback);

    // Publish a low level feedback message
    msg->header.stamp = state->header.stamp;
    msg->battery_state = quadrotor_msgs::LowLevelFeedback::BAT_INVALID;
    if (state->cell_voltage.size() > 0)
    {
        battery_voltage = msg->battery_voltage = state->cell_voltage[0];
        if (msg->battery_voltage > n_lipo_cells * kBatteryLowVoltagePerCell)
        {
            msg->battery_state = quadrotor_msgs::LowLevelFeedback::BAT_GOOD;
        }
        else if (msg->battery_voltage >
                n_lipo_cells * kBatteryCriticalVoltagePerCell)
        {
            msg->battery_state = quadrotor_msgs::LowLevelFeedback::BAT_LOW;
        }
        else if (msg->battery_voltage >
                n_lipo_cells * kBatteryInvalidVoltagePerCell)
        {
            msg->battery_state = quadrotor_msgs::LowLevelFeedback::BAT_CRITICAL;
        }
    }
    
    if (rc_manual)
    {
        msg->control_mode = quadrotor_msgs::LowLevelFeedback::RC_MANUAL;
    }
    else
    {
        msg->control_mode = use_rate ? quadrotor_msgs::LowLevelFeedback::BODY_RATES : quadrotor_msgs::LowLevelFeedback::ATTITUDE;
        // msg.control_mode = msg.NONE;
    }

    ap_feedback_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apm_bridge");
    ros::NodeHandle nh("~");
    VehicleNode vh_node(nh);
    ros::spin();
    return 0;
}
