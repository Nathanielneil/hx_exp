#ifndef HX_AIRSIM_INTERFACE_H
#define HX_AIRSIM_INTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <hx_msgs/UAVState.h>
#include <hx_msgs/AirSimState.h>

#include <string>
#include <thread>
#include <atomic>

// Forward declarations for AirSim API
namespace msr { namespace airlib {
    class MultirotorRpcLibClient;
}}

class HXAirSimInterface
{
public:
    HXAirSimInterface(ros::NodeHandle& nh);
    ~HXAirSimInterface();
    
    bool initialize();
    void run();
    void shutdown();

private:
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Publishers
    ros::Publisher uav_state_pub_;
    ros::Publisher airsim_state_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher pose_pub_;
    
    // Subscribers  
    ros::Subscriber airsim_command_sub_;
    
    // Timers
    ros::Timer state_update_timer_;
    
    // AirSim client
    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> airsim_client_;
    
    // Connection parameters
    std::string airsim_ip_;
    int airsim_port_;
    std::string vehicle_name_;
    
    // State update parameters
    double state_update_frequency_;
    
    // System state
    std::atomic<bool> connected_;
    std::atomic<bool> running_;
    hx_msgs::UAVState current_uav_state_;
    hx_msgs::AirSimState current_airsim_state_;
    
    // Coordinate transformation parameters
    bool use_ned_frame_;
    
    // Threading
    std::thread state_update_thread_;
    
    // Callback functions
    void airSimCommandCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void stateUpdateTimerCallback(const ros::TimerEvent& e);
    
    // AirSim interface functions
    bool connectToAirSim();
    void disconnectFromAirSim();
    bool enableApiControl();
    bool armVehicle();
    
    // State update functions
    void updateVehicleState();
    void publishUAVState();
    void publishAirSimState();
    void publishIMU();
    void publishPose();
    
    // Control command functions
    void sendControlCommand(double roll, double pitch, double yaw_rate, double throttle);
    void sendVelocityCommand(double vx, double vy, double vz, double yaw_rate);
    void sendPositionCommand(double x, double y, double z, double yaw);
    
    // Coordinate transformation functions
    void airsimToROS(const msr::airlib::Vector3r& airsim_vec, geometry_msgs::Vector3& ros_vec);
    void airsimToROS(const msr::airlib::Pose& airsim_pose, geometry_msgs::Pose& ros_pose);
    void rosToAirSim(const geometry_msgs::Vector3& ros_vec, msr::airlib::Vector3r& airsim_vec);
    
    // Utility functions
    void loadParameters();
    void initializePublishers();
    void initializeSubscribers();
    bool waitForConnection(double timeout_sec = 10.0);
    void logConnectionStatus();
};

#endif