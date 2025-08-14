#include <ros/ros.h>
#include <hx_msgs/UAVCommand.h>
#include <hx_msgs/UAVState.h>
#include <hx_msgs/UAVControlState.h>
#include <geometry_msgs/Point.h>

class BasicPositionControlDemo
{
public:
    BasicPositionControlDemo(ros::NodeHandle& nh) : nh_(nh), demo_step_(0), step_start_time_(0)
    {
        // Publishers
        command_pub_ = nh_.advertise<hx_msgs::UAVCommand>("/Drone1/hx_uav/command", 10);
        
        // Subscribers
        state_sub_ = nh_.subscribe<hx_msgs::UAVState>("/Drone1/hx_uav/state", 10,
                                                     &BasicPositionControlDemo::stateCallback, this);
        control_state_sub_ = nh_.subscribe<hx_msgs::UAVControlState>("/Drone1/hx_uav/control_state", 10,
                                                                   &BasicPositionControlDemo::controlStateCallback, this);
        
        // Timer for demo execution
        demo_timer_ = nh_.createTimer(ros::Duration(0.1), &BasicPositionControlDemo::demoTimerCallback, this);
        
        ROS_INFO("[Basic Position Control Demo] Initialized - Waiting for UAV connection...");
    }
    
private:
    ros::NodeHandle nh_;
    ros::Publisher command_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber control_state_sub_;
    ros::Timer demo_timer_;
    
    hx_msgs::UAVState current_state_;
    hx_msgs::UAVControlState current_control_state_;
    
    int demo_step_;
    ros::Time step_start_time_;
    bool state_received_;
    bool control_state_received_;
    
    void stateCallback(const hx_msgs::UAVState::ConstPtr& msg)
    {
        current_state_ = *msg;
        state_received_ = true;
    }
    
    void controlStateCallback(const hx_msgs::UAVControlState::ConstPtr& msg)
    {
        current_control_state_ = *msg;
        control_state_received_ = true;
    }
    
    void demoTimerCallback(const ros::TimerEvent& e)
    {
        if (!state_received_ || !control_state_received_)
        {
            return;
        }
        
        ros::Time current_time = ros::Time::now();
        
        switch (demo_step_)
        {
            case 0: // Takeoff
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 1: Takeoff");
                    sendTakeoffCommand();
                    step_start_time_ = current_time;
                }
                else if ((current_time - step_start_time_).toSec() > 5.0 && 
                         current_control_state_.control_state == hx_msgs::UAVControlState::COMMAND_CONTROL)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 1: // Move to position 1
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 2: Move to position [2, 2, 3]");
                    sendMoveCommand(2.0, 2.0, 3.0);
                    step_start_time_ = current_time;
                }
                else if ((current_time - step_start_time_).toSec() > 8.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 2: // Move to position 2
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 3: Move to position [-2, 2, 3]");
                    sendMoveCommand(-2.0, 2.0, 3.0);
                    step_start_time_ = current_time;
                }
                else if ((current_time - step_start_time_).toSec() > 8.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 3: // Move to position 3
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 4: Move to position [-2, -2, 3]");
                    sendMoveCommand(-2.0, -2.0, 3.0);
                    step_start_time_ = current_time;
                }
                else if ((current_time - step_start_time_).toSec() > 8.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 4: // Move to position 4
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 5: Move to position [2, -2, 3]");
                    sendMoveCommand(2.0, -2.0, 3.0);
                    step_start_time_ = current_time;
                }
                else if ((current_time - step_start_time_).toSec() > 8.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 5: // Return to center
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 6: Return to center [0, 0, 3]");
                    sendMoveCommand(0.0, 0.0, 3.0);
                    step_start_time_ = current_time;
                }
                else if ((current_time - step_start_time_).toSec() > 8.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 6: // Land
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 7: Landing");
                    sendLandCommand();
                    step_start_time_ = current_time;
                }
                else if ((current_time - step_start_time_).toSec() > 10.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 7: // Demo complete
                ROS_INFO("[Demo] Basic position control demo completed successfully!");
                ros::shutdown();
                break;
                
            default:
                break;
        }
        
        // Print current status
        if ((current_time.toSec() - (int)current_time.toSec()) < 0.1)  // Print once per second
        {
            ROS_INFO("[Demo] Current position: [%.2f, %.2f, %.2f], State: %d", 
                     current_state_.position[0], current_state_.position[1], current_state_.position[2],
                     current_control_state_.control_state);
        }
    }
    
    void sendTakeoffCommand()
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Takeoff;
        cmd.Control_mode = hx_msgs::UAVCommand::PID_CONTROL;
        cmd.Command_ID = "demo_takeoff";
        
        command_pub_.publish(cmd);
    }
    
    void sendMoveCommand(double x, double y, double z)
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Move_ENU;
        cmd.Control_mode = hx_msgs::UAVCommand::PID_CONTROL;
        
        cmd.position.x = x;
        cmd.position.y = y;
        cmd.position.z = z;
        
        cmd.linear_vel.x = 0.0;
        cmd.linear_vel.y = 0.0;
        cmd.linear_vel.z = 0.0;
        
        cmd.Command_ID = "demo_move";
        
        command_pub_.publish(cmd);
    }
    
    void sendLandCommand()
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Land;
        cmd.Command_ID = "demo_land";
        
        command_pub_.publish(cmd);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_position_control_demo");
    ros::NodeHandle nh;
    
    ROS_INFO("[Basic Position Control Demo] Starting demo...");
    
    BasicPositionControlDemo demo(nh);
    
    ros::spin();
    
    return 0;
}