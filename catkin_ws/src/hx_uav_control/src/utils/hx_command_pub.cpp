#include <ros/ros.h>
#include <hx_msgs/UAVCommand.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <string>

class HXCommandPublisher
{
public:
    HXCommandPublisher(ros::NodeHandle& nh) : nh_(nh)
    {
        command_pub_ = nh_.advertise<hx_msgs::UAVCommand>("/hx_uav/command", 10);
        
        // Wait for publisher to establish connections
        ros::Duration(1.0).sleep();
        
        ROS_INFO("[HX Command Publisher] Ready to send commands");
    }
    
    void publishTakeoffCommand()
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Takeoff;
        cmd.Control_mode = hx_msgs::UAVCommand::PID_CONTROL;
        cmd.Command_ID = "takeoff_command";
        
        command_pub_.publish(cmd);
        ROS_INFO("[HX Command Publisher] Takeoff command sent");
    }
    
    void publishLandCommand()
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Land;
        cmd.Command_ID = "land_command";
        
        command_pub_.publish(cmd);
        ROS_INFO("[HX Command Publisher] Land command sent");
    }
    
    void publishMoveCommand(double x, double y, double z, double yaw = 0.0)
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
        
        cmd.Command_ID = "move_command";
        
        command_pub_.publish(cmd);
        ROS_INFO("[HX Command Publisher] Move command sent: [%.2f, %.2f, %.2f]", x, y, z);
    }
    
    void publishHoldCommand()
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Hold;
        cmd.Command_ID = "hold_command";
        
        command_pub_.publish(cmd);
        ROS_INFO("[HX Command Publisher] Hold command sent");
    }
    
    void publishCircleTrajectoryCommand()
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Trajectory_Tracking;
        cmd.trajectory_flag = hx_msgs::UAVCommand::Circle_trajectory;
        cmd.Control_mode = hx_msgs::UAVCommand::PID_CONTROL;
        cmd.Command_ID = "circle_trajectory";
        
        command_pub_.publish(cmd);
        ROS_INFO("[HX Command Publisher] Circle trajectory command sent");
    }
    
    void publishEightTrajectoryCommand()
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Trajectory_Tracking;
        cmd.trajectory_flag = hx_msgs::UAVCommand::Eight_trajectory;
        cmd.Control_mode = hx_msgs::UAVCommand::UDE_CONTROL;
        cmd.Command_ID = "eight_trajectory";
        
        command_pub_.publish(cmd);
        ROS_INFO("[HX Command Publisher] Figure-eight trajectory command sent");
    }
    
    void publishControllerSwitchCommand(int controller_type)
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Hold;  // Keep current position
        cmd.Control_mode = controller_type;
        cmd.Command_ID = "controller_switch";
        
        command_pub_.publish(cmd);
        
        std::string controller_name;
        switch(controller_type)
        {
            case hx_msgs::UAVCommand::PID_CONTROL: controller_name = "PID"; break;
            case hx_msgs::UAVCommand::UDE_CONTROL: controller_name = "UDE"; break;
            case hx_msgs::UAVCommand::ADRC_CONTROL: controller_name = "ADRC"; break;
            default: controller_name = "Unknown"; break;
        }
        
        ROS_INFO("[HX Command Publisher] Controller switch command sent: %s", controller_name.c_str());
    }
    
    void showMenu()
    {
        std::cout << "\n=== HX UAV Command Publisher ===" << std::endl;
        std::cout << "Available commands:" << std::endl;
        std::cout << "  1 - Takeoff" << std::endl;
        std::cout << "  2 - Land" << std::endl;
        std::cout << "  3 - Move to position" << std::endl;
        std::cout << "  4 - Hold position" << std::endl;
        std::cout << "  5 - Circle trajectory" << std::endl;
        std::cout << "  6 - Figure-8 trajectory" << std::endl;
        std::cout << "  7 - Switch to PID controller" << std::endl;
        std::cout << "  8 - Switch to UDE controller" << std::endl;
        std::cout << "  9 - Switch to ADRC controller" << std::endl;
        std::cout << "  0 - Exit" << std::endl;
        std::cout << "Enter command: ";
    }
    
    void runInteractive()
    {
        int choice;
        double x, y, z;
        
        while (ros::ok())
        {
            showMenu();
            
            if (!(std::cin >> choice))
            {
                std::cin.clear();
                std::cin.ignore(10000, '\n');
                std::cout << "Invalid input. Please enter a number." << std::endl;
                continue;
            }
            
            switch (choice)
            {
                case 1:
                    publishTakeoffCommand();
                    break;
                    
                case 2:
                    publishLandCommand();
                    break;
                    
                case 3:
                    std::cout << "Enter position (x y z): ";
                    if (std::cin >> x >> y >> z)
                    {
                        publishMoveCommand(x, y, z);
                    }
                    else
                    {
                        std::cout << "Invalid position input" << std::endl;
                        std::cin.clear();
                        std::cin.ignore(10000, '\n');
                    }
                    break;
                    
                case 4:
                    publishHoldCommand();
                    break;
                    
                case 5:
                    publishCircleTrajectoryCommand();
                    break;
                    
                case 6:
                    publishEightTrajectoryCommand();
                    break;
                    
                case 7:
                    publishControllerSwitchCommand(hx_msgs::UAVCommand::PID_CONTROL);
                    break;
                    
                case 8:
                    publishControllerSwitchCommand(hx_msgs::UAVCommand::UDE_CONTROL);
                    break;
                    
                case 9:
                    publishControllerSwitchCommand(hx_msgs::UAVCommand::ADRC_CONTROL);
                    break;
                    
                case 0:
                    std::cout << "Exiting..." << std::endl;
                    return;
                    
                default:
                    std::cout << "Invalid choice. Please try again." << std::endl;
                    break;
            }
            
            ros::Duration(0.1).sleep();  // Small delay
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher command_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hx_command_pub");
    ros::NodeHandle nh;
    
    HXCommandPublisher cmd_pub(nh);
    
    // Check if command line arguments were provided
    if (argc > 1)
    {
        std::string command = argv[1];
        
        if (command == "takeoff")
        {
            cmd_pub.publishTakeoffCommand();
        }
        else if (command == "land")
        {
            cmd_pub.publishLandCommand();
        }
        else if (command == "hold")
        {
            cmd_pub.publishHoldCommand();
        }
        else if (command == "move" && argc >= 5)
        {
            double x = std::atof(argv[2]);
            double y = std::atof(argv[3]);
            double z = std::atof(argv[4]);
            cmd_pub.publishMoveCommand(x, y, z);
        }
        else if (command == "circle")
        {
            cmd_pub.publishCircleTrajectoryCommand();
        }
        else if (command == "eight")
        {
            cmd_pub.publishEightTrajectoryCommand();
        }
        else
        {
            std::cout << "Usage: " << argv[0] << " [takeoff|land|hold|move x y z|circle|eight]" << std::endl;
            return 1;
        }
    }
    else
    {
        // Run interactive mode
        cmd_pub.runInteractive();
    }
    
    return 0;
}