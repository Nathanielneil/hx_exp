#include <ros/ros.h>
#include "hx_uav_control/hx_airsim_interface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hx_airsim_interface_node");
    ros::NodeHandle nh("~");
    
    ROS_INFO("[HX AirSim Interface Node] Starting AirSim interface...");
    
    try 
    {
        HXAirSimInterface interface(nh);
        
        ROS_INFO("[HX AirSim Interface Node] Interface initialized successfully");
        ROS_INFO("[HX AirSim Interface Node] Connecting to AirSim...");
        
        interface.run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[HX AirSim Interface Node] Exception: %s", e.what());
        return -1;
    }
    
    ROS_INFO("[HX AirSim Interface Node] Shutting down");
    return 0;
}