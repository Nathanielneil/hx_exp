#include <ros/ros.h>
#include "hx_uav_control/hx_uav_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hx_uav_control_node");
    ros::NodeHandle nh("~");
    
    ROS_INFO("[HX UAV Control Node] Starting HX UAV Control System...");
    
    try 
    {
        HXUAVController controller(nh);
        
        ROS_INFO("[HX UAV Control Node] Controller initialized successfully");
        ROS_INFO("[HX UAV Control Node] System ready for operation");
        
        controller.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[HX UAV Control Node] Exception: %s", e.what());
        return -1;
    }
    
    ROS_INFO("[HX UAV Control Node] Shutting down");
    return 0;
}