#include "apm_vehicle_node.h"
#include <mavlink/v2.0/ardupilotmega/ardupilotmega.hpp>

using namespace apm_bridge;
namespace mavlink_msg = mavlink::common::msg;

VehicleNode::VehicleNode()
    : Node("apm_vehicle_node"), ema(50)
{
    this->declare_parameter<double>("gravity_const", 9.80665);
    this->declare_parameter<double>("mass", 2.0);
    // this->declare_parameter<double>("control_frequency", 10.0);
    this->declare_parameter<bool>("simulation", false);
    this->declare_parameter<int>("lipo_cells", 6);
    this->declare_parameter<bool>("voltage_compensation", false);
    this->declare_parameter<double>("motor_parameters.a", 0.0);
    this->declare_parameter<double>("motor_parameters.b", 0.0);
    this->declare_parameter<double>("motor_parameters.c", 0.0);
    this->declare_parameter<int>("motor_parameters.n", 1);
    this->declare_parameter<double>("motor_parameters.voltage_a", 0.0);
    this->declare_parameter<double>("motor_parameters.voltage_b", 0.0);
    this->declare_parameter<double>("init_position.altitude", 100.0);
    this->declare_parameter<double>("init_position.latitude", 0.0);
    this->declare_parameter<double>("init_position.longitude", 0.0);

    this->get_parameter("gravity_const", gravity);
    this->get_parameter("mass", mass);
    // this->get_parameter("control_frequency", control_frequency);
    this->get_parameter("simulation", simulation);
    this->get_parameter("lipo_cells", n_lipo_cells);
    this->get_parameter("voltage_compensation", voltage_compensation);
    double a, b, c;
    this->get_parameter("motor_parameters.a", a);
    this->get_parameter("motor_parameters.b", b);
    this->get_parameter("motor_parameters.c", c);
    this->get_parameter("motor_parameters.n", motor_params.n_motors);
    this->get_parameter("motor_parameters.voltage_a", motor_params.voltage_map_a);
    this->get_parameter("motor_parameters.voltage_b", motor_params.voltage_map_b);

    quadratic_thrust_model::convert_from_abc(motor_params, a, b, c);

    printf("Motor parameters: A=%.6f, B=%.6f, C=%.6f, n=%d, voltage_a=%.6f, voltage_b=%.6f\n", 
        motor_params.A, motor_params.B, motor_params.C, motor_params.n_motors, motor_params.voltage_map_a, motor_params.voltage_map_b);

    // setupMavlink();

    tf_br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_br_static = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    control_command_sub = this->create_subscription<quadrotor_msgs::msg::ControlCommand>(
        "~/control_command", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), std::bind(&VehicleNode::ctrlCommandCallback, this, std::placeholders::_1));
    control_command_raw_sub = this->create_subscription<quadrotor_msgs::msg::ControlCommand>(
        "~/control_command_raw", rclcpp::QoS(rclcpp::KeepLast(5)).best_effort(), std::bind(&VehicleNode::ctrlCommandRawCallback, this, std::placeholders::_1));
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        std::bind(&VehicleNode::imuCallback, this, std::placeholders::_1));
    atti_target_sub = this->create_subscription<mavros_msgs::msg::AttitudeTarget>(
        "/mavros/setpoint_raw/target_attitude", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        std::bind(&VehicleNode::attiTargetCallback, this, std::placeholders::_1));
    arm_interface_sub = this->create_subscription<std_msgs::msg::Bool>(
        "~/arm",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(), 
        std::bind(&VehicleNode::armCallback, this, std::placeholders::_1));
    atm_sub = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        "/mavros/imu/static_pressure", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        std::bind(&VehicleNode::atmPressureCallback, this, std::placeholders::_1));
    temp_sub = this->create_subscription<sensor_msgs::msg::Temperature>(
        "/mavros/imu/temperature_baro", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        std::bind(&VehicleNode::tempCallback, this, std::placeholders::_1));
    ext_state_sub = this->create_subscription<mavros_msgs::msg::ExtendedState>(
        "/mavros/extended_state", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        std::bind(&VehicleNode::extStateCallback, this, std::placeholders::_1));
    state_sub = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        std::bind(&VehicleNode::stateCallback, this, std::placeholders::_1));
    battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/mavros/battery", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
        std::bind(&VehicleNode::batteryCallback, this, std::placeholders::_1));

    target_pub = this->create_publisher<mavros_msgs::msg::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    ap_feedback_pub = this->create_publisher<quadrotor_msgs::msg::LowLevelFeedback>("~/low_level_feedback", 10);

    command_arming = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    waypoint_list_sub = this->create_subscription<mavros_msgs::msg::WaypointList>("/mavros/mission/waypoints", 10, 
        [&](const mavros_msgs::msg::WaypointList::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received WaypointList with %zu waypoints", msg->waypoints.size());
            // sync_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&VehicleNode::syncWorkerCallback, this));
            auto t = std::thread(&VehicleNode::setupMavlink, this);
            t.detach();
            waypoint_list_sub.reset();
    });

    set_global_pos_pub = this->create_publisher<geographic_msgs::msg::GeoPointStamped>("/mavros/global_position/set_gp_origin", rclcpp::SensorDataQoS());
    // sync_timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&VehicleNode::syncWorkerCallback, this));
}

VehicleNode::~VehicleNode() 
{
    sync_timer_->cancel();
}

void VehicleNode::setupMavlink()
{
    RCLCPP_WARN(this->get_logger(), "VehicleNode::setupMavlink()");
    auto set_message_interval = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    if (!set_message_interval->wait_for_service(std::chrono::seconds(10)) || !set_message_interval->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Service /mavros/cmd/command not available");
        return;
    }

    auto send_request = [&](uint32_t msg_id, float msg_rate) {
        auto cmdrq = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        cmdrq->broadcast = false;
        cmdrq->command = uint16_t(mavlink::common::MAV_CMD::SET_MESSAGE_INTERVAL);
        cmdrq->confirmation = false;
        cmdrq->param1 = msg_id;
        cmdrq->param2 = 1000000.0f / msg_rate;

        auto future = set_message_interval->async_send_request(cmdrq,
            [&](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future) {
                // RCLCPP_ERROR(this->get_logger(), "set_message_interval callback called");
                if (future.valid()) {
                    RCLCPP_INFO(this->get_logger(), "Set message interval for msg_id %u to %.2f Hz", msg_id, msg_rate);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set message interval for msg_id %u", msg_id);
                }
            });
        future.wait();
    };

    // auto set_message_interval = this->create_client<mavros_msgs::srv::MessageInterval>("/mavros/set_message_interval");
    // if (!set_message_interval->wait_for_service(std::chrono::seconds(10)) || !set_message_interval->service_is_ready()) {
    //     RCLCPP_ERROR(this->get_logger(), "Service /mavros/set_message_interval not available");
    //     return;
    // }

    // RCLCPP_WARN(this->get_logger(), "Setting message intervals for various MAVLink messages...");
    
    // auto send_request = [&](uint32_t msg_id, float msg_rate) {
    //     auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
    //     request->message_id = msg_id;
    //     request->message_rate = msg_rate;
    //     // auto result = set_message_interval->async_send_request(request,
    //     //     [this, msg_id, msg_rate](rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedFuture future) {
    //     //         RCLCPP_ERROR(this->get_logger(), "set_message_interval callback called");
    //     //         if (future.valid()) {
    //     //             RCLCPP_INFO(this->get_logger(), "Set message interval for msg_id %u to %.2f Hz", msg_id, msg_rate);
    //     //         } else {
    //     //             RCLCPP_ERROR(this->get_logger(), "Failed to set message interval for msg_id %u", msg_id);
    //     //         }
    //     //     });
    //     // bool suc = executeService<mavros_msgs::srv::MessageInterval>(set_message_interval, request);
    //     // if (suc)
    //     // {
    //     //     RCLCPP_INFO(this->get_logger(), "Set message interval for msg_id %u to %.2f Hz", msg_id, msg_rate);
    //     // } else {
    //     //     RCLCPP_ERROR(this->get_logger(), "Failed to set message interval for msg_id %u", msg_id);
    //     // }
    //     // auto result = set_message_interval->async_send_request(request);
    //     // if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
    //     // {
    //     //     RCLCPP_INFO(this->get_logger(), "Set message interval for msg_id %u to %.2f Hz", msg_id, msg_rate);
    //     // } else {
    //     //     RCLCPP_ERROR(this->get_logger(), "Failed to set message interval for msg_id %u", msg_id);
    //     // }
    // };

    send_request(mavlink_msg::ATTITUDE_TARGET::MSG_ID, 100.0f);
    send_request(mavlink_msg::EXTENDED_SYS_STATE::MSG_ID, 5.0f);
    send_request(mavlink_msg::SCALED_PRESSURE::MSG_ID, 5.0f);
    send_request(mavlink_msg::BATTERY_STATUS::MSG_ID, 1.0f);
    send_request(mavlink_msg::RC_CHANNELS::MSG_ID, 50.0f);
    send_request(mavlink_msg::DISTANCE_SENSOR::MSG_ID, 10.0f);

    send_request(mavlink_msg::ATTITUDE_QUATERNION::MSG_ID, 200.0f);
    send_request(mavlink_msg::SCALED_IMU::MSG_ID, 200.0f);

    if (simulation)
    {
        send_request(mavlink_msg::LOCAL_POSITION_NED::MSG_ID, 100.0f);
        // send_request(mavlink_msg::LOCAL_POSITION_NED_COV::MSG_ID, (float)control_frequency);
    }
}

void VehicleNode::syncWorkerCallback()
{
    geographic_msgs::msg::GeoPointStamped pos;
    this->get_parameter("init_position.altitude", pos.position.altitude);
    this->get_parameter("init_position.latitude", pos.position.latitude);
    this->get_parameter("init_position.longitude", pos.position.longitude);

    if (set_global_pos_pub->get_subscription_count() > 0)
    {
        set_global_pos_pub->publish(pos);
    
        auto command_set_home = this->create_client<mavros_msgs::srv::CommandHome>("/mavros/cmd/set_home");
        if(command_set_home->wait_for_service()) //(std::chrono::seconds(1)))
        {
            auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
            request->current_gps = 0;
            request->yaw = 0;
            request->altitude = pos.position.altitude;
            request->latitude = pos.position.latitude;
            request->longitude = pos.position.longitude;
            command_set_home->async_send_request(request);
            RCLCPP_WARN(this->get_logger(), "VehicleNode::syncWorkerCallback()");
        }
        sync_timer_->cancel(); // Stop the timer after successful execution
    }
}

void VehicleNode::extStateCallback(const mavros_msgs::msg::ExtendedState::SharedPtr state)
{
    landed_state = state->landed_state;
}

void VehicleNode::stateCallback(const mavros_msgs::msg::State::SharedPtr state)
{
    armed.store(state->armed);
    if (state->mode == "LOITER" || state->mode == "POSHOLD")
    {
        hoverable = 1;
        rc_manual = true;
    }
    else if (state->mode == "GUIDED" || state->mode == "GUIDED_NOGPS")
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

void VehicleNode::attiTargetCallback(const mavros_msgs::msg::AttitudeTarget::SharedPtr att)
{
    //if (hoverable >=1 ) // when hoverable mode
    {
        std::lock_guard<std::mutex> lk(mtx_kp);
        if (voltage_compensation)
            collective_force = quadratic_thrust_model::thrustToForce(motor_params, att->thrust, kp, battery_voltage);
        else
            collective_force = quadratic_thrust_model::thrustToForce(motor_params, att->thrust, kp);
        // printf("collective_force: %.2f N, thrust: %.2f, kp: %.2f, voltage: %.2f V\n", collective_force, att->thrust, kp, battery_voltage);
    }
}

void VehicleNode::armCallback(const std_msgs::msg::Bool::SharedPtr msg) 
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = msg->data;
    // command_arming->async_send_request(request);
    executeService<mavros_msgs::srv::CommandBool>(command_arming, request);
}

void VehicleNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr val)
{
    static uint8_t last_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED;
    static double gravity_acc = gravity;

    auto current_state = landed_state.load();
    if (last_state != current_state)
    {
        last_state = current_state;
        ema.reset();
    }

    // printf("current_state: %u, armed: %d, hoverable: %u, in_hover: %d, use_rate: %d, collective_force: %.2f N, mass: %.2f kg\n", 
    //     current_state, armed.load(), hoverable, in_hover, use_rate, collective_force, mass);

    if (current_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND && !armed.load()) // Update g value
    {
        auto acc = std::sqrt(val->linear_acceleration.x * val->linear_acceleration.x +
                             val->linear_acceleration.y * val->linear_acceleration.y +
                             val->linear_acceleration.z * val->linear_acceleration.z);
        gravity_acc = ema.filter(acc, 2);
    }
    else if (current_state == mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR
        && (hoverable == 1 || (hoverable==2 && in_hover))) // Update mass when hovering
    {
        tf2::Quaternion q;
        tf2::fromMsg(val->orientation, q);
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        tf2::Vector3 acc;
        acc.setY(val->linear_acceleration.y - std::sin(roll) * std::cos(pitch) * gravity_acc);
        acc.setZ(val->linear_acceleration.z - std::cos(roll) * std::cos(pitch) * gravity_acc);
        acc.setX(val->linear_acceleration.x + std::sin(pitch) * gravity_acc);

        // printf("roll: %.2f, pitch: %.2f, yaw: %.2f, acc: (%.2f, %.2f, %.2f) m/s^2\n", 
        //     roll, pitch, yaw, val->linear_acceleration.x, val->linear_acceleration.y, val->linear_acceleration.z);
        // printf("Estimated mass: %.2f kg, g: %.2f m/s^2, acc^2: %.2f\n", 
        //         collective_force / gravity, gravity_acc, acc.length2());

        if (std::fabs(roll) < 0.05 && std::fabs(pitch) < 0.05 && ema.filter(acc.length2(), 4) < 0.01
            && collective_force > mass*gravity*0.5) // if hovering
        {
            auto mass_t = ema.filter(collective_force / gravity, 3);

            if (std::fabs(mass_t - mass) > 0.1)
            {
                std::lock_guard<std::mutex> lk(mtx_kp);
                mass = mass_t;
                RCLCPP_WARN(this->get_logger(), "Updated Mass = %f kg", mass);
            }
        }
    }
}

void VehicleNode::ctrlCommandRawCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command)
{
    if (command->armed == 0 || !armed.load() || hoverable != 2)
        return;
    auto target = std::make_unique<mavros_msgs::msg::AttitudeTarget>();
    switch (command->control_mode)
    {
    case quadrotor_msgs::msg::ControlCommand::ATTITUDE:
        target->type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                            mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                            mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
        target->orientation = command->orientation;
        use_rate = false;
        break;
    case quadrotor_msgs::msg::ControlCommand::BODY_RATES:
        target->type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
        target->body_rate = command->bodyrates;
        use_rate = true;
        break;
    default:
        break;
    }

    target->thrust = (command->collective_thrust < 0.0) ? 
        0.0f : ((command->collective_thrust > 1.0) ? 1.0f : (float)command->collective_thrust);

    target_pub->publish(std::move(target));
}

void VehicleNode::ctrlCommandCallback(const quadrotor_msgs::msg::ControlCommand::SharedPtr command)
{
    // const double delta_t = 1.0/control_frequency;
    static bool set_unarm = false;

    if (set_unarm && armed.load() && 
        landed_state.load() == mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND)
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = false;
        // command_arming->async_send_request(request);
        executeService<mavros_msgs::srv::CommandBool>(command_arming, request);
        set_unarm = false;
        return;
    }

    if (command->armed == 0 || !armed.load() || hoverable != 2)
        return;

    if (command->collective_thrust < 1e-6)  //Prepare unarming
    {
        set_unarm = true;
    }

    in_hover = command->is_hover_state != 0;
    auto target = std::make_unique<mavros_msgs::msg::AttitudeTarget>();
    switch (command->control_mode)
    {
    case quadrotor_msgs::msg::ControlCommand::ATTITUDE:
    {
        target->type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                            mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                            mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
        
        double delta_t = rclcpp::Time(command->expected_execution_time).seconds();
        tf2::Quaternion delta_q;
        delta_q.setRPY(command->bodyrates.x*delta_t, 
                     command->bodyrates.y*delta_t, 
                     command->bodyrates.z*delta_t);
        
        tf2::Quaternion q_cmd;
        tf2::fromMsg(command->orientation, q_cmd);
        tf2::Quaternion att = q_cmd * delta_q;
        target->orientation = tf2::toMsg(att);

        use_rate = false;
        break;
    }
    case quadrotor_msgs::msg::ControlCommand::BODY_RATES:
        target->type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;
        // target->type_mask = 0;
        // target->orientation = command->orientation;
        target->body_rate = command->bodyrates;
        use_rate = true;
        break;
    default:
        break;
    }

    double thrust = 0.0;
    {
        std::lock_guard<std::mutex> lk(mtx_kp);
        auto force = command->collective_thrust * mass;
        if (voltage_compensation)
            thrust = quadratic_thrust_model::forceToThrust(motor_params, force, kp, battery_voltage);
        else
            thrust = quadratic_thrust_model::forceToThrust(motor_params, force, kp);
    }
    target->thrust = (thrust < 0.0) ? 0.0f : ((thrust > 1.0) ? 1.0f : (float)thrust);

    // printf("target bodyrate: (%.2f, %.2f, %.2f) rad/s, force: %.2f, thrust: %.2f\n", 
    //     target->body_rate.x, target->body_rate.y, target->body_rate.z, command->collective_thrust * mass, thrust);

    target_pub->publish(std::move(target));
}

void VehicleNode::atmPressureCallback(const sensor_msgs::msg::FluidPressure::SharedPtr val)
{
    std::lock_guard<std::mutex> lk(mtx_kp);
    baro = val->fluid_pressure;
    // kp = (baro / 101325.0) * (273.15 / (273.15 + temp));
}

void VehicleNode::tempCallback(const sensor_msgs::msg::Temperature::SharedPtr val)
{
    std::lock_guard<std::mutex> lk(mtx_kp);
    temp = val->temperature - 20.0;
    // kp = (baro / 101325.0) * (273.15 / (273.15 + temp));
}

void VehicleNode::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr state)
{
    auto msg = std::make_unique<quadrotor_msgs::msg::LowLevelFeedback>();

    msg->header.stamp = state->header.stamp;
    msg->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_INVALID;
    if (state->cell_voltage.size() > 0)
    {
        {
            std::lock_guard<std::mutex> lk(mtx_kp);
            battery_voltage = msg->battery_voltage = state->cell_voltage[0];
        }
        if (msg->battery_voltage > n_lipo_cells * kBatteryLowVoltagePerCell)
        {
            msg->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_GOOD;
        }
        else if (msg->battery_voltage >
                n_lipo_cells * kBatteryCriticalVoltagePerCell)
        {
            msg->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_LOW;
        }
        else if (msg->battery_voltage >
                n_lipo_cells * kBatteryInvalidVoltagePerCell)
        {
            msg->battery_state = quadrotor_msgs::msg::LowLevelFeedback::BAT_CRITICAL;
        }
    }
    
    if (rc_manual)
    {
        msg->control_mode = quadrotor_msgs::msg::LowLevelFeedback::RC_MANUAL;
    }
    else
    {
        msg->control_mode = use_rate ? quadrotor_msgs::msg::LowLevelFeedback::BODY_RATES : quadrotor_msgs::msg::LowLevelFeedback::ATTITUDE;
    }

    ap_feedback_pub->publish(std::move(msg));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<apm_bridge::VehicleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
