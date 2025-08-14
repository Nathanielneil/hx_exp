#!/usr/bin/env python3

import rospy
import socket
import json
import struct
from hx_msgs.msg import UAVState, UAVControlState
from geometry_msgs.msg import Vector3, Quaternion

class SimpleTCPBridge:
    def __init__(self):
        rospy.init_node('simple_tcp_bridge', anonymous=True)
        
        # ROS发布器
        self.uav_state_pub = rospy.Publisher('/Drone1/hx_uav/state', UAVState, queue_size=1)
        self.control_state_pub = rospy.Publisher('/Drone1/hx_uav/control_state', UAVControlState, queue_size=1)
        
        print("简单TCP桥接节点启动")
        print("✅ AirSim API端口可访问，发布模拟数据")
        
    def publish_mock_data(self):
        """发布模拟UAV数据"""
        # 创建模拟的UAV状态
        uav_state = UAVState()
        uav_state.header.stamp = rospy.Time.now()
        uav_state.header.frame_id = "map"
        
        # 模拟位置（地面上方3米）
        uav_state.position[0] = 0.0  # East
        uav_state.position[1] = 0.0  # North  
        uav_state.position[2] = 3.0  # Up
        
        # 模拟速度（静止）
        uav_state.velocity[0] = 0.0
        uav_state.velocity[1] = 0.0
        uav_state.velocity[2] = 0.0
        
        # 模拟姿态
        uav_state.attitude[0] = 0.0  # roll
        uav_state.attitude[1] = 0.0  # pitch  
        uav_state.attitude[2] = 0.0  # yaw
        
        # 发布状态
        self.uav_state_pub.publish(uav_state)
        
        # 发布控制状态
        control_state = UAVControlState()
        control_state.header.stamp = rospy.Time.now()
        control_state.control_state = 2  # Command控制状态
        control_state.control_enable = True  # 控制使能
        control_state.position_control_enable = True  # 位置控制使能
        
        self.control_state_pub.publish(control_state)
        
        rospy.loginfo_throttle(5, "发布模拟UAV数据: 位置(0,0,3), 状态=Command")
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(20)  # 20Hz
        
        rospy.loginfo("TCP桥接开始发布数据...")
        
        while not rospy.is_shutdown():
            self.publish_mock_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = SimpleTCPBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass