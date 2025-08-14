#include <ros/ros.h>
#include <hx_msgs/UAVCommand.h>
#include <hx_msgs/UAVState.h>
#include <hx_msgs/UAVControlState.h>
#include <geometry_msgs/Point.h>
#include <cmath>

class CircleTrajectoryDemo
{
public:
    CircleTrajectoryDemo(ros::NodeHandle& nh) : nh_(nh), demo_step_(0), step_start_time_(0)
    {
        // Publishers
        command_pub_ = nh_.advertise<hx_msgs::UAVCommand>("/Drone1/hx_uav/command", 10);
        
        // Subscribers
        state_sub_ = nh_.subscribe<hx_msgs::UAVState>("/Drone1/hx_uav/state", 10,
                                                     &CircleTrajectoryDemo::stateCallback, this);
        control_state_sub_ = nh_.subscribe<hx_msgs::UAVControlState>("/Drone1/hx_uav/control_state", 10,
                                                                   &CircleTrajectoryDemo::controlStateCallback, this);
        
        // Timer for demo execution
        demo_timer_ = nh_.createTimer(ros::Duration(0.1), &CircleTrajectoryDemo::demoTimerCallback, this);
        
        // Trajectory parameters
        circle_center_x_ = 0.0;
        circle_center_y_ = 0.0;
        circle_height_ = 3.0;
        circle_radius_ = 3.0;
        circle_speed_ = 1.0; // m/s
        
        ROS_INFO("[Circle Trajectory Demo] Initialized - Circle parameters: center=[%.1f,%.1f], radius=%.1f, height=%.1f", 
                 circle_center_x_, circle_center_y_, circle_radius_, circle_height_);
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
    
    // Trajectory parameters
    double circle_center_x_, circle_center_y_;
    double circle_height_;
    double circle_radius_;
    double circle_speed_;
    
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
                    ROS_INFO("[Demo] Step 1: Takeoff to height %.1fm", circle_height_);
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
                
            case 1: // Move to circle start position
                if (step_start_time_.isZero())
                {
                    double start_x = circle_center_x_ + circle_radius_;
                    double start_y = circle_center_y_;
                    ROS_INFO("[Demo] Step 2: Move to circle start position [%.1f, %.1f, %.1f]", 
                             start_x, start_y, circle_height_);
                    sendMoveCommand(start_x, start_y, circle_height_);
                    step_start_time_ = current_time;
                }
                else if ((current_time - step_start_time_).toSec() > 8.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 2: // Execute circle trajectory with PID controller
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 3: Executing circle trajectory with PID controller");
                    step_start_time_ = current_time;
                }
                executeCircleTrajectory(current_time, hx_msgs::UAVCommand::PID_CONTROL);
                
                if ((current_time - step_start_time_).toSec() > 20.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 3: // Switch to UDE controller and execute circle
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 4: Switching to UDE controller");
                    step_start_time_ = current_time;
                }
                executeCircleTrajectory(current_time, hx_msgs::UAVCommand::UDE_CONTROL);
                
                if ((current_time - step_start_time_).toSec() > 20.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 4: // Switch to ADRC controller and execute circle
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 5: Switching to ADRC controller");
                    step_start_time_ = current_time;
                }
                executeCircleTrajectory(current_time, hx_msgs::UAVCommand::ADRC_CONTROL);
                
                if ((current_time - step_start_time_).toSec() > 20.0)
                {
                    demo_step_++;
                    step_start_time_ = ros::Time(0);
                }
                break;
                
            case 5: // Return to center and land
                if (step_start_time_.isZero())
                {
                    ROS_INFO("[Demo] Step 6: Returning to center and landing");
                    sendMoveCommand(circle_center_x_, circle_center_y_, circle_height_);
                    step_start_time_ = current_time;
                }
                else if ((current_time - step_start_time_).toSec() > 5.0)
                {
                    sendLandCommand();
                    if ((current_time - step_start_time_).toSec() > 15.0)
                    {
                        demo_step_++;
                    }
                }
                break;
                
            case 6: // Demo complete
                ROS_INFO("[Demo] Circle trajectory demo completed successfully!");
                ROS_INFO("[Demo] Controllers tested: PID, UDE, ADRC");
                ros::shutdown();
                break;
                
            default:
                break;
        }
    }
    
    void executeCircleTrajectory(const ros::Time& current_time, int controller_type)
    {
        static ros::Time trajectory_start_time;
        
        if (trajectory_start_time.isZero())
        {
            trajectory_start_time = current_time;
        }
        
        double elapsed_time = (current_time - trajectory_start_time).toSec();
        
        // Calculate circle position
        double angular_velocity = circle_speed_ / circle_radius_; // rad/s
        double angle = angular_velocity * elapsed_time;
        
        double target_x = circle_center_x_ + circle_radius_ * cos(angle);
        double target_y = circle_center_y_ + circle_radius_ * sin(angle);
        double target_z = circle_height_;
        
        // Calculate circle velocity
        double vel_x = -circle_speed_ * sin(angle);
        double vel_y = circle_speed_ * cos(angle);
        double vel_z = 0.0;
        
        // Send command
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = current_time;
        cmd.Agent_CMD = hx_msgs::UAVCommand::Move_ENU;
        cmd.Control_mode = controller_type;
        
        cmd.position.x = target_x;
        cmd.position.y = target_y;
        cmd.position.z = target_z;
        
        cmd.linear_vel.x = vel_x;
        cmd.linear_vel.y = vel_y;
        cmd.linear_vel.z = vel_z;
        
        cmd.Command_ID = "circle_trajectory";
        
        command_pub_.publish(cmd);
        
        // Reset trajectory start time when switching controllers
        if ((current_time.toSec() - (int)current_time.toSec()) < 0.1)  // Every second
        {
            std::string controller_name;
            switch(controller_type)
            {
                case hx_msgs::UAVCommand::PID_CONTROL: controller_name = "PID"; break;
                case hx_msgs::UAVCommand::UDE_CONTROL: controller_name = "UDE"; break;
                case hx_msgs::UAVCommand::ADRC_CONTROL: controller_name = "ADRC"; break;
            }
            
            ROS_INFO("[Demo] %s Controller - Target: [%.2f, %.2f, %.2f], Current: [%.2f, %.2f, %.2f]", 
                     controller_name.c_str(),
                     target_x, target_y, target_z,
                     current_state_.position[0], current_state_.position[1], current_state_.position[2]);
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
    ros::init(argc, argv, "circle_trajectory_demo");
    ros::NodeHandle nh;
    
    ROS_INFO("[Circle Trajectory Demo] Starting circle trajectory demo...");
    
    CircleTrajectoryDemo demo(nh);
    
    ros::spin();
    
    return 0;
}