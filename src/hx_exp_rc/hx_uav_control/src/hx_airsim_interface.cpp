#include "hx_uav_control/hx_airsim_interface.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Note: This is a simplified implementation without actual AirSim API
// In a real implementation, you would include and use AirSim headers:
// #include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

HXAirSimInterface::HXAirSimInterface(ros::NodeHandle& nh) : nh_(nh), connected_(false), running_(false)
{
    loadParameters();
    initializePublishers();
    initializeSubscribers();
    
    ROS_INFO("[HX AirSim Interface] Interface initialized");
}

HXAirSimInterface::~HXAirSimInterface()
{
    shutdown();
}

void HXAirSimInterface::loadParameters()
{
    nh_.param<std::string>("airsim/ip", airsim_ip_, "localhost");
    nh_.param<int>("airsim/port", airsim_port_, 41451);
    nh_.param<std::string>("airsim/vehicle_name", vehicle_name_, "Drone1");
    nh_.param<double>("state_update_frequency", state_update_frequency_, 50.0);
    nh_.param<bool>("use_ned_frame", use_ned_frame_, true);
    
    ROS_INFO("[HX AirSim Interface] Parameters loaded - IP: %s, Port: %d, Vehicle: %s", 
             airsim_ip_.c_str(), airsim_port_, vehicle_name_.c_str());
}

void HXAirSimInterface::initializePublishers()
{
    uav_state_pub_ = nh_.advertise<hx_msgs::UAVState>("/hx_uav/state", 10);
    airsim_state_pub_ = nh_.advertise<hx_msgs::AirSimState>("/hx_uav/airsim_state", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/hx_uav/imu", 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/hx_uav/pose", 10);
}

void HXAirSimInterface::initializeSubscribers()
{
    airsim_command_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/hx_uav/airsim_command", 10,
                                                                   &HXAirSimInterface::airSimCommandCallback, this);
}

bool HXAirSimInterface::initialize()
{
    ROS_INFO("[HX AirSim Interface] Initializing connection to AirSim...");
    
    if (!connectToAirSim())
    {
        ROS_ERROR("[HX AirSim Interface] Failed to connect to AirSim");
        return false;
    }
    
    if (!enableApiControl())
    {
        ROS_ERROR("[HX AirSim Interface] Failed to enable API control");
        return false;
    }
    
    if (!armVehicle())
    {
        ROS_ERROR("[HX AirSim Interface] Failed to arm vehicle");
        return false;
    }
    
    // Setup state update timer
    state_update_timer_ = nh_.createTimer(ros::Duration(1.0/state_update_frequency_),
                                        &HXAirSimInterface::stateUpdateTimerCallback, this);
    
    running_ = true;
    connected_ = true;
    
    ROS_INFO("[HX AirSim Interface] Successfully connected and initialized");
    return true;
}

void HXAirSimInterface::run()
{
    if (!initialize())
    {
        return;
    }
    
    ros::spin();
}

void HXAirSimInterface::shutdown()
{
    running_ = false;
    
    if (state_update_thread_.joinable())
    {
        state_update_thread_.join();
    }
    
    disconnectFromAirSim();
    
    ROS_INFO("[HX AirSim Interface] Shutdown complete");
}

bool HXAirSimInterface::connectToAirSim()
{
    try
    {
        // In a real implementation, create AirSim client here:
        // airsim_client_ = std::make_unique<msr::airlib::MultirotorRpcLibClient>(airsim_ip_, airsim_port_);
        // airsim_client_->confirmConnection();
        
        // For demonstration, simulate successful connection
        ROS_INFO("[HX AirSim Interface] Connected to AirSim at %s:%d", airsim_ip_.c_str(), airsim_port_);
        return true;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[HX AirSim Interface] Connection failed: %s", e.what());
        return false;
    }
}

void HXAirSimInterface::disconnectFromAirSim()
{
    connected_ = false;
    // airsim_client_.reset();
    ROS_INFO("[HX AirSim Interface] Disconnected from AirSim");
}

bool HXAirSimInterface::enableApiControl()
{
    try
    {
        // In a real implementation:
        // airsim_client_->enableApiControl(true, vehicle_name_);
        
        ROS_INFO("[HX AirSim Interface] API control enabled for vehicle: %s", vehicle_name_.c_str());
        return true;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[HX AirSim Interface] Failed to enable API control: %s", e.what());
        return false;
    }
}

bool HXAirSimInterface::armVehicle()
{
    try
    {
        // In a real implementation:
        // airsim_client_->armDisarm(true, vehicle_name_);
        
        ROS_INFO("[HX AirSim Interface] Vehicle armed: %s", vehicle_name_.c_str());
        return true;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[HX AirSim Interface] Failed to arm vehicle: %s", e.what());
        return false;
    }
}

void HXAirSimInterface::airSimCommandCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    if (!connected_ || !running_)
    {
        return;
    }
    
    // Extract control commands
    double roll = msg->twist.angular.x;
    double pitch = msg->twist.angular.y;
    double yaw_rate = msg->twist.angular.z;
    double throttle = msg->twist.linear.z;
    
    // Send control command to AirSim
    sendControlCommand(roll, pitch, yaw_rate, throttle);
}

void HXAirSimInterface::stateUpdateTimerCallback(const ros::TimerEvent& e)
{
    if (!connected_ || !running_)
    {
        return;
    }
    
    updateVehicleState();
    publishUAVState();
    publishAirSimState();
    publishIMU();
    publishPose();
}

void HXAirSimInterface::updateVehicleState()
{
    try
    {
        // In a real implementation, get state from AirSim:
        // auto pose = airsim_client_->simGetVehiclePose(vehicle_name_);
        // auto kinematics = airsim_client_->getMultirotorState(vehicle_name_).kinematics_estimated;
        
        // For demonstration, create dummy state
        current_uav_state_.header.stamp = ros::Time::now();
        current_uav_state_.position[0] = 0.0;  // x
        current_uav_state_.position[1] = 0.0;  // y  
        current_uav_state_.position[2] = 3.0;  // z (height)
        
        current_uav_state_.velocity[0] = 0.0;
        current_uav_state_.velocity[1] = 0.0;
        current_uav_state_.velocity[2] = 0.0;
        
        current_uav_state_.attitude[0] = 0.0;  // roll
        current_uav_state_.attitude[1] = 0.0;  // pitch
        current_uav_state_.attitude[2] = 0.0;  // yaw
        
        // Quaternion from euler angles
        tf2::Quaternion q;
        q.setRPY(current_uav_state_.attitude[0], 
                current_uav_state_.attitude[1], 
                current_uav_state_.attitude[2]);
        current_uav_state_.attitude_q.w = q.w();
        current_uav_state_.attitude_q.x = q.x();
        current_uav_state_.attitude_q.y = q.y();
        current_uav_state_.attitude_q.z = q.z();
        
        current_uav_state_.connected = connected_;
        current_uav_state_.armed = true;
        current_uav_state_.odom_valid = true;
        current_uav_state_.mode = "GUIDED";
        
        // Update AirSim specific state
        current_airsim_state_.header.stamp = ros::Time::now();
        current_airsim_state_.api_control_enabled = true;
        current_airsim_state_.vehicle_ready = true;
        current_airsim_state_.collision_detected = false;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[HX AirSim Interface] Failed to update vehicle state: %s", e.what());
        connected_ = false;
    }
}

void HXAirSimInterface::publishUAVState()
{
    uav_state_pub_.publish(current_uav_state_);
}

void HXAirSimInterface::publishAirSimState()
{
    airsim_state_pub_.publish(current_airsim_state_);
}

void HXAirSimInterface::publishIMU()
{
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "base_link";
    
    // Copy orientation from UAV state
    imu_msg.orientation = current_uav_state_.attitude_q;
    
    // Angular velocity (would come from AirSim)
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    
    // Linear acceleration (would come from AirSim)
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 9.81;
    
    imu_pub_.publish(imu_msg);
}

void HXAirSimInterface::publishPose()
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world";
    
    pose_msg.pose.position.x = current_uav_state_.position[0];
    pose_msg.pose.position.y = current_uav_state_.position[1];
    pose_msg.pose.position.z = current_uav_state_.position[2];
    
    pose_msg.pose.orientation = current_uav_state_.attitude_q;
    
    pose_pub_.publish(pose_msg);
}

void HXAirSimInterface::sendControlCommand(double roll, double pitch, double yaw_rate, double throttle)
{
    try
    {
        // In a real implementation:
        // airsim_client_->moveByRollPitchYawZAsync(roll, pitch, yaw_rate, 
        //                                         current_uav_state_.position[2], 
        //                                         0.02, vehicle_name_);
        
        // For now, just log the command
        ROS_DEBUG("[HX AirSim Interface] Control command - Roll: %.3f, Pitch: %.3f, Yaw Rate: %.3f, Throttle: %.3f",
                 roll, pitch, yaw_rate, throttle);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[HX AirSim Interface] Failed to send control command: %s", e.what());
    }
}

void HXAirSimInterface::sendVelocityCommand(double vx, double vy, double vz, double yaw_rate)
{
    try
    {
        // In a real implementation:
        // airsim_client_->moveByVelocityAsync(vx, vy, vz, 0.02, 
        //                                    msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
        //                                    msr::airlib::YawMode(true, yaw_rate), vehicle_name_);
        
        ROS_DEBUG("[HX AirSim Interface] Velocity command - Vx: %.3f, Vy: %.3f, Vz: %.3f, Yaw Rate: %.3f",
                 vx, vy, vz, yaw_rate);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[HX AirSim Interface] Failed to send velocity command: %s", e.what());
    }
}

void HXAirSimInterface::sendPositionCommand(double x, double y, double z, double yaw)
{
    try
    {
        // In a real implementation:
        // airsim_client_->moveToPositionAsync(x, y, z, 2.0, 
        //                                    msr::airlib::YawMode(false, yaw), vehicle_name_);
        
        ROS_DEBUG("[HX AirSim Interface] Position command - X: %.3f, Y: %.3f, Z: %.3f, Yaw: %.3f",
                 x, y, z, yaw);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[HX AirSim Interface] Failed to send position command: %s", e.what());
    }
}