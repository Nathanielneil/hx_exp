#ifndef HX_UAV_CONTROLLER_H
#define HX_UAV_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <hx_msgs/UAVState.h>
#include <hx_msgs/UAVCommand.h>
#include <hx_msgs/UAVControlState.h>
#include <hx_msgs/ControllerOutput.h>
#include <hx_msgs/AirSimState.h>

#include <Eigen/Eigen>
#include <string>
#include <memory>

#include "Position_Controller/pos_controller_PID.h"
#include "Position_Controller/pos_controller_UDE.h"
#include "Position_Controller/pos_controller_ADRC.h"
#include "Position_Controller/controller_utils.h"

using namespace std;

enum CONTROL_STATE
{
    INITIALIZATION = 0,
    TAKEOFF = 1,
    COMMAND_CONTROL = 2,
    HOLD = 3,
    LAND = 4,
    DISARM = 5,
    EMERGENCY = 6
};

enum CONTROLLER_TYPE
{
    PID_CONTROL = 1,
    UDE_CONTROL = 2,
    ADRC_CONTROL = 3
};

class HXUAVController
{
public:
    HXUAVController(ros::NodeHandle& nh);
    ~HXUAVController();
    
    void run();

private:
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Publishers
    ros::Publisher control_state_pub_;
    ros::Publisher controller_output_pub_;
    ros::Publisher airsim_command_pub_;
    
    // Subscribers
    ros::Subscriber uav_command_sub_;
    ros::Subscriber uav_state_sub_;
    ros::Subscriber airsim_state_sub_;
    
    // Timers
    ros::Timer control_timer_;
    ros::Timer state_timer_;
    
    // Control state
    CONTROL_STATE control_state_;
    CONTROLLER_TYPE current_controller_type_;
    
    // UAV state and command
    hx_msgs::UAVState current_uav_state_;
    hx_msgs::UAVCommand current_command_;
    hx_msgs::UAVControlState control_state_msg_;
    hx_msgs::AirSimState airsim_state_;
    
    // Controller instances
    std::unique_ptr<pos_controller_PID> pid_controller_;
    std::unique_ptr<pos_controller_UDE> ude_controller_;
    std::unique_ptr<pos_controller_ADRC> adrc_controller_;
    
    // Desired state
    Desired_State desired_state_;
    
    // Controller parameters
    double control_frequency_;
    double state_frequency_;
    bool system_armed_;
    bool control_enabled_;
    
    // Safety parameters
    double max_velocity_;
    double max_acceleration_;
    double flight_boundary_;
    double min_height_;
    double max_height_;
    
    // Trajectory tracking
    vector<TrajectoryPoint> current_trajectory_;
    int trajectory_index_;
    bool trajectory_tracking_;
    
    // Callback functions
    void uavCommandCallback(const hx_msgs::UAVCommand::ConstPtr& msg);
    void uavStateCallback(const hx_msgs::UAVState::ConstPtr& msg);
    void airSimStateCallback(const hx_msgs::AirSimState::ConstPtr& msg);
    
    // Timer callbacks
    void controlTimerCallback(const ros::TimerEvent& e);
    void stateTimerCallback(const ros::TimerEvent& e);
    
    // Control functions
    void updateControlState();
    void executeControlCommand();
    void publishControlState();
    void publishControllerOutput(const Eigen::Vector4d& control_output, CONTROLLER_TYPE type);
    
    // State machine functions
    void handleInitialization();
    void handleTakeoff();
    void handleCommandControl();
    void handleHold();
    void handleLand();
    void handleEmergency();
    
    // Controller functions
    void switchController(CONTROLLER_TYPE type);
    Eigen::Vector4d computeControlOutput();
    void setDesiredState(const geometry_msgs::Point& pos, const geometry_msgs::Vector3& vel, double yaw);
    
    // Trajectory functions
    void generateCircleTrajectory(const geometry_msgs::Point& center, double radius, double speed, double total_time);
    void generateEightTrajectory(const geometry_msgs::Point& center, double radius, double speed, double total_time);
    void updateTrajectoryTracking();
    
    // Safety functions
    bool safetyCheck();
    void emergencyStop();
    
    // Utility functions
    void loadParameters();
    void initializeControllers();
    bool isCommandValid(const hx_msgs::UAVCommand& cmd);
    
    // State conversion functions
    void convertAirSimToUAVState();
};

#endif