#!/usr/bin/env python3

import rospy
import sys
import os

# 尝试多个可能的airsim路径
airsim_paths = [
    '/home/ubuntu/anaconda3/envs/jz/lib/python3.7/site-packages',
    '/home/ubuntu/anaconda3/envs/jz/lib/python3.8/site-packages',
    '/home/ubuntu/anaconda3/envs/jz/lib/python3.9/site-packages',
    '/home/ubuntu/.local/lib/python3.8/site-packages',
    '/usr/local/lib/python3.8/dist-packages'
]

airsim_found = False
for path in airsim_paths:
    if os.path.exists(path):
        sys.path.insert(0, path)
        try:
            import airsim
            print(f"AirSim模块加载成功 (from {path})")
            airsim_found = True
            break
        except ImportError:
            continue

if not airsim_found:
    print("❌ 无法找到airsim模块")
    print("尝试安装: pip install airsim")
    
    # 尝试简单的网络连接测试
    import socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex(('127.0.0.1', 41451))
        sock.close()
        if result == 0:
            print("✅ AirSim API端口(41451)可访问，但Python模块缺失")
        else:
            print("❌ AirSim API端口(41451)不可访问")
    except Exception as e:
        print(f"网络测试失败: {e}")
    
    sys.exit(1)

from hx_msgs.msg import UAVState, UAVControlState
from geometry_msgs.msg import Vector3, Quaternion
import time

class SimpleAirSimBridge:
    def __init__(self):
        rospy.init_node('simple_airsim_bridge', anonymous=True)
        
        # AirSim客户端
        self.client = None
        self.connected = False
        
        # ROS发布器
        self.uav_state_pub = rospy.Publisher('/Drone1/hx_uav/state', UAVState, queue_size=1)
        self.control_state_pub = rospy.Publisher('/Drone1/hx_uav/control_state', UAVControlState, queue_size=1)
        
        print("简单AirSim桥接节点启动")
        self.connect_to_airsim()
        
    def connect_to_airsim(self):
        """连接到AirSim"""
        try:
            print("正在连接AirSim...")
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            print("✅ AirSim连接成功!")
            self.connected = True
        except Exception as e:
            print(f"❌ AirSim连接失败: {e}")
            print("请确保AirSim正在运行")
            self.connected = False
            
    def publish_uav_state(self):
        """发布UAV状态"""
        if not self.connected:
            return
            
        try:
            # 获取无人机状态
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            vel = state.kinematics_estimated.linear_velocity
            orientation = state.kinematics_estimated.orientation
            
            # 创建ROS消息
            uav_state = UAVState()
            uav_state.header.stamp = rospy.Time.now()
            uav_state.header.frame_id = "map"
            
            # 位置 (AirSim使用NED坐标系，转换为ENU)
            uav_state.position[0] = pos.y_val  # East
            uav_state.position[1] = pos.x_val  # North  
            uav_state.position[2] = -pos.z_val # Up
            
            # 速度
            uav_state.velocity[0] = vel.y_val
            uav_state.velocity[1] = vel.x_val
            uav_state.velocity[2] = -vel.z_val
            
            # 姿态（四元数转欧拉角的简化版本）
            uav_state.attitude[0] = 0  # roll
            uav_state.attitude[1] = 0  # pitch  
            uav_state.attitude[2] = 0  # yaw
            
            # 发布状态
            self.uav_state_pub.publish(uav_state)
            
            # 发布控制状态
            control_state = UAVControlState()
            control_state.header.stamp = rospy.Time.now()
            control_state.control_state = 2  # Command控制状态
            control_state.landed = (state.landed_state == airsim.LandedState.Landed)
            
            self.control_state_pub.publish(control_state)
            
        except Exception as e:
            print(f"发布状态失败: {e}")
            self.connected = False
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown():
            if not self.connected:
                self.connect_to_airsim()
                time.sleep(1)
                continue
                
            self.publish_uav_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = SimpleAirSimBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass