#include "hx_uav_control/hx_uav_controller.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

HXUAVController::HXUAVController(ros::NodeHandle& nh) : nh_(nh)
{
    // Initialize state
    control_state_ = INITIALIZATION;
    current_controller_type_ = PID_CONTROL;
    system_armed_ = false;
    control_enabled_ = false;
    trajectory_tracking_ = false;
    trajectory_index_ = 0;
    
    // Load parameters
    loadParameters();
    
    // Initialize controllers
    initializeControllers();
    
    // Setup publishers
    control_state_pub_ = nh_.advertise<hx_msgs::UAVControlState>("/hx_uav/control_state", 10);
    controller_output_pub_ = nh_.advertise<hx_msgs::ControllerOutput>("/hx_uav/controller_output", 10);
    airsim_command_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/hx_uav/airsim_command", 10);
    
    // Setup subscribers
    uav_command_sub_ = nh_.subscribe<hx_msgs::UAVCommand>("/hx_uav/command", 10, 
                                                        &HXUAVController::uavCommandCallback, this);
    uav_state_sub_ = nh_.subscribe<hx_msgs::UAVState>("/hx_uav/state", 10,
                                                     &HXUAVController::uavStateCallback, this);
    airsim_state_sub_ = nh_.subscribe<hx_msgs::AirSimState>("/hx_uav/airsim_state", 10,
                                                          &HXUAVController::airSimStateCallback, this);
    
    // Setup timers
    control_timer_ = nh_.createTimer(ros::Duration(1.0/control_frequency_), 
                                   &HXUAVController::controlTimerCallback, this);
    state_timer_ = nh_.createTimer(ros::Duration(1.0/state_frequency_),
                                 &HXUAVController::stateTimerCallback, this);
    
    ROS_INFO("[HX UAV Controller] Initialized successfully");
}

HXUAVController::~HXUAVController()
{
    ROS_INFO("[HX UAV Controller] Shutting down");
}

void HXUAVController::loadParameters()
{
    // Control frequencies
    nh_.param<double>("control_frequency", control_frequency_, 50.0);
    nh_.param<double>("state_frequency", state_frequency_, 20.0);
    
    // Safety parameters
    nh_.param<double>("safety/max_velocity", max_velocity_, 5.0);
    nh_.param<double>("safety/max_acceleration", max_acceleration_, 3.0);
    nh_.param<double>("safety/flight_boundary", flight_boundary_, 50.0);
    nh_.param<double>("safety/min_height", min_height_, 0.5);
    nh_.param<double>("safety/max_height", max_height_, 20.0);
    
    ROS_INFO("[HX UAV Controller] Parameters loaded");
}

void HXUAVController::initializeControllers()
{
    pid_controller_ = std::make_unique<pos_controller_PID>();
    ude_controller_ = std::make_unique<pos_controller_UDE>();
    adrc_controller_ = std::make_unique<pos_controller_ADRC>();
    
    pid_controller_->init(nh_);
    ude_controller_->init(nh_);
    adrc_controller_->init(nh_);
    
    ROS_INFO("[HX UAV Controller] All controllers initialized");
}

void HXUAVController::run()
{
    ros::spin();
}

void HXUAVController::uavCommandCallback(const hx_msgs::UAVCommand::ConstPtr& msg)
{
    if (!isCommandValid(*msg))
    {
        ROS_WARN("[HX UAV Controller] Invalid command received");
        return;
    }
    
    current_command_ = *msg;
    
    // Process command based on type
    switch (current_command_.Agent_CMD)
    {
        case hx_msgs::UAVCommand::Init_Pos_Hover:
            if (control_state_ == COMMAND_CONTROL)
            {
                control_state_ = HOLD;
                setDesiredState(current_uav_state_.position, {0, 0, 0}, current_uav_state_.attitude[2]);
            }
            break;
            
        case hx_msgs::UAVCommand::Move_ENU:
            if (control_state_ == COMMAND_CONTROL || control_state_ == HOLD)
            {
                setDesiredState(current_command_.position, current_command_.linear_vel, 0.0);
                trajectory_tracking_ = false;
            }
            break;
            
        case hx_msgs::UAVCommand::Takeoff:
            if (control_state_ == INITIALIZATION)
            {
                control_state_ = TAKEOFF;
                geometry_msgs::Point takeoff_pos;
                takeoff_pos.x = current_uav_state_.position[0];
                takeoff_pos.y = current_uav_state_.position[1]; 
                takeoff_pos.z = 3.0;  // Default takeoff height
                setDesiredState(takeoff_pos, {0, 0, 0}, 0.0);
            }
            break;
            
        case hx_msgs::UAVCommand::Land:
            control_state_ = LAND;
            geometry_msgs::Point land_pos;
            land_pos.x = current_uav_state_.position[0];
            land_pos.y = current_uav_state_.position[1];
            land_pos.z = 0.0;
            setDesiredState(land_pos, {0, 0, -0.5}, 0.0);
            break;
            
        case hx_msgs::UAVCommand::Trajectory_Tracking:
            if (current_command_.trajectory_flag == hx_msgs::UAVCommand::Circle_trajectory)
            {
                generateCircleTrajectory({0, 0, -3}, 3.0, 2.0, 30.0);
                trajectory_tracking_ = true;
                trajectory_index_ = 0;
            }
            else if (current_command_.trajectory_flag == hx_msgs::UAVCommand::Eight_trajectory)
            {
                generateEightTrajectory({0, 0, -3}, 3.0, 2.0, 60.0);
                trajectory_tracking_ = true;
                trajectory_index_ = 0;
            }
            break;
    }
    
    // Switch controller if requested
    if (current_command_.Control_mode != 0)
    {
        switchController(static_cast<CONTROLLER_TYPE>(current_command_.Control_mode));
    }
}

void HXUAVController::uavStateCallback(const hx_msgs::UAVState::ConstPtr& msg)
{
    current_uav_state_ = *msg;
    system_armed_ = current_uav_state_.armed;
    
    // Update controller states
    if (pid_controller_) pid_controller_->set_current_state(current_uav_state_);
    if (ude_controller_) ude_controller_->set_current_state(current_uav_state_);
    if (adrc_controller_) adrc_controller_->set_current_state(current_uav_state_);
}

void HXUAVController::airSimStateCallback(const hx_msgs::AirSimState::ConstPtr& msg)
{
    airsim_state_ = *msg;
}

void HXUAVController::controlTimerCallback(const ros::TimerEvent& e)
{
    if (!system_armed_)
    {
        return;
    }
    
    // Safety check
    if (!safetyCheck())
    {
        emergencyStop();
        return;
    }
    
    // Update control state machine
    updateControlState();
    
    // Execute control
    executeControlCommand();
    
    // Publish states
    publishControlState();
}

void HXUAVController::stateTimerCallback(const ros::TimerEvent& e)
{
    // Convert AirSim state to UAV state if needed
    convertAirSimToUAVState();
}

void HXUAVController::updateControlState()
{
    switch (control_state_)
    {
        case INITIALIZATION:
            handleInitialization();
            break;
        case TAKEOFF:
            handleTakeoff();
            break;
        case COMMAND_CONTROL:
            handleCommandControl();
            break;
        case HOLD:
            handleHold();
            break;
        case LAND:
            handleLand();
            break;
        case EMERGENCY:
            handleEmergency();
            break;
    }
}

void HXUAVController::executeControlCommand()
{
    if (!control_enabled_)
        return;
        
    // Update trajectory if tracking
    if (trajectory_tracking_)
    {
        updateTrajectoryTracking();
    }
    
    // Set desired state to controllers
    pid_controller_->set_desired_state(desired_state_);
    ude_controller_->set_desired_state(desired_state_);
    adrc_controller_->set_desired_state(desired_state_);
    
    // Compute control output
    Eigen::Vector4d control_output = computeControlOutput();
    
    // Publish controller output
    publishControllerOutput(control_output, current_controller_type_);
    
    // Send command to AirSim interface
    geometry_msgs::TwistStamped airsim_cmd;
    airsim_cmd.header.stamp = ros::Time::now();
    airsim_cmd.twist.angular.x = control_output(0);  // roll
    airsim_cmd.twist.angular.y = control_output(1);  // pitch
    airsim_cmd.twist.angular.z = control_output(2);  // yaw_rate
    airsim_cmd.twist.linear.z = control_output(3);   // throttle
    
    airsim_command_pub_.publish(airsim_cmd);
}

Eigen::Vector4d HXUAVController::computeControlOutput()
{
    Eigen::Vector4d output;
    
    switch (current_controller_type_)
    {
        case PID_CONTROL:
            output = pid_controller_->update(control_frequency_);
            break;
        case UDE_CONTROL:
            output = ude_controller_->update(control_frequency_);
            break;
        case ADRC_CONTROL:
            output = adrc_controller_->update(control_frequency_);
            break;
        default:
            output.setZero();
            ROS_ERROR("[HX UAV Controller] Unknown controller type");
            break;
    }
    
    return output;
}

void HXUAVController::handleInitialization()
{
    if (system_armed_ && current_uav_state_.connected)
    {
        control_enabled_ = true;
        ROS_INFO("[HX UAV Controller] System initialized and ready");
    }
}

void HXUAVController::handleTakeoff()
{
    // Check if takeoff is complete
    double height = current_uav_state_.position[2];
    if (height >= 2.5)  // Near desired takeoff height
    {
        control_state_ = COMMAND_CONTROL;
        ROS_INFO("[HX UAV Controller] Takeoff complete, entering command control");
    }
}

void HXUAVController::handleCommandControl()
{
    // Normal operation state - commands are processed in callback
}

void HXUAVController::handleHold()
{
    // Maintain current position
}

void HXUAVController::handleLand()
{
    double height = current_uav_state_.position[2];
    if (height <= 0.2)
    {
        control_state_ = DISARM;
        control_enabled_ = false;
        ROS_INFO("[HX UAV Controller] Landing complete");
    }
}

void HXUAVController::handleEmergency()
{
    // Emergency hover
    geometry_msgs::Point emergency_pos;
    emergency_pos.x = current_uav_state_.position[0];
    emergency_pos.y = current_uav_state_.position[1];
    emergency_pos.z = current_uav_state_.position[2];
    setDesiredState(emergency_pos, {0, 0, 0}, current_uav_state_.attitude[2]);
}

void HXUAVController::switchController(CONTROLLER_TYPE type)
{
    if (type != current_controller_type_)
    {
        current_controller_type_ = type;
        string controller_name;
        switch (type)
        {
            case PID_CONTROL: controller_name = "PID"; break;
            case UDE_CONTROL: controller_name = "UDE"; break;
            case ADRC_CONTROL: controller_name = "ADRC"; break;
        }
        ROS_INFO("[HX UAV Controller] Switched to %s controller", controller_name.c_str());
    }
}

void HXUAVController::setDesiredState(const geometry_msgs::Point& pos, const geometry_msgs::Vector3& vel, double yaw)
{
    desired_state_.pos << pos.x, pos.y, pos.z;
    desired_state_.vel << vel.x, vel.y, vel.z;
    desired_state_.acc.setZero();
    desired_state_.yaw = yaw;
}

bool HXUAVController::safetyCheck()
{
    // Check position bounds
    if (abs(current_uav_state_.position[0]) > flight_boundary_ ||
        abs(current_uav_state_.position[1]) > flight_boundary_)
    {
        ROS_ERROR("[HX UAV Controller] Position out of bounds");
        return false;
    }
    
    // Check height bounds
    if (current_uav_state_.position[2] < min_height_ ||
        current_uav_state_.position[2] > max_height_)
    {
        ROS_ERROR("[HX UAV Controller] Height out of bounds: %.2f", current_uav_state_.position[2]);
        return false;
    }
    
    // Check velocity
    double velocity_norm = sqrt(pow(current_uav_state_.velocity[0], 2) + 
                              pow(current_uav_state_.velocity[1], 2) + 
                              pow(current_uav_state_.velocity[2], 2));
    if (velocity_norm > max_velocity_)
    {
        ROS_ERROR("[HX UAV Controller] Velocity too high: %.2f", velocity_norm);
        return false;
    }
    
    return true;
}

void HXUAVController::emergencyStop()
{
    control_state_ = EMERGENCY;
    ROS_ERROR("[HX UAV Controller] Emergency stop activated");
}

void HXUAVController::publishControlState()
{
    control_state_msg_.header.stamp = ros::Time::now();
    control_state_msg_.control_state = control_state_;
    control_state_msg_.control_mode = current_controller_type_;
    control_state_msg_.control_enable = control_enabled_;
    control_state_msg_.trajectory_tracking = trajectory_tracking_;
    control_state_msg_.current_command = current_command_;
    
    control_state_pub_.publish(control_state_msg_);
}

void HXUAVController::publishControllerOutput(const Eigen::Vector4d& control_output, CONTROLLER_TYPE type)
{
    hx_msgs::ControllerOutput msg;
    msg.header.stamp = ros::Time::now();
    msg.controller_type = type;
    msg.roll_command = control_output(0);
    msg.pitch_command = control_output(1);
    msg.yaw_rate_command = control_output(2);
    msg.throttle_command = control_output(3);
    
    controller_output_pub_.publish(msg);
}

bool HXUAVController::isCommandValid(const hx_msgs::UAVCommand& cmd)
{
    return true;  // Basic validation - can be expanded
}

void HXUAVController::convertAirSimToUAVState()
{
    // Convert AirSim state to standard UAV state if needed
    // This function can be used to bridge different state representations
}

void HXUAVController::generateCircleTrajectory(const geometry_msgs::Point& center, double radius, double speed, double total_time)
{
    current_trajectory_.clear();
    int num_points = static_cast<int>(total_time * control_frequency_);
    
    for (int i = 0; i < num_points; i++)
    {
        double t = static_cast<double>(i) / control_frequency_;
        double angle = 2 * M_PI * speed * t / (2 * M_PI * radius);
        
        TrajectoryPoint point;
        point.position.x = center.x + radius * cos(angle);
        point.position.y = center.y + radius * sin(angle);
        point.position.z = center.z;
        
        point.velocity.x = -radius * sin(angle) * speed / radius;
        point.velocity.y = radius * cos(angle) * speed / radius;
        point.velocity.z = 0;
        
        point.time_from_start = t;
        current_trajectory_.push_back(point);
    }
    
    ROS_INFO("[HX UAV Controller] Circle trajectory generated with %zu points", current_trajectory_.size());
}

void HXUAVController::generateEightTrajectory(const geometry_msgs::Point& center, double radius, double speed, double total_time)
{
    current_trajectory_.clear();
    int num_points = static_cast<int>(total_time * control_frequency_);
    
    for (int i = 0; i < num_points; i++)
    {
        double t = static_cast<double>(i) / control_frequency_;
        double angle = 2 * M_PI * speed * t / (4 * radius);  // Figure-8 parameter
        
        TrajectoryPoint point;
        point.position.x = center.x + radius * sin(angle);
        point.position.y = center.y + radius * sin(2 * angle) / 2;
        point.position.z = center.z;
        
        point.velocity.x = radius * cos(angle) * speed / (4 * radius);
        point.velocity.y = radius * cos(2 * angle) * speed / (4 * radius);
        point.velocity.z = 0;
        
        point.time_from_start = t;
        current_trajectory_.push_back(point);
    }
    
    ROS_INFO("[HX UAV Controller] Eight trajectory generated with %zu points", current_trajectory_.size());
}

void HXUAVController::updateTrajectoryTracking()
{
    if (trajectory_index_ < current_trajectory_.size())
    {
        const auto& point = current_trajectory_[trajectory_index_];
        setDesiredState(point.position, point.velocity, 0.0);
        trajectory_index_++;
    }
    else
    {
        trajectory_tracking_ = false;
        ROS_INFO("[HX UAV Controller] Trajectory tracking completed");
    }
}