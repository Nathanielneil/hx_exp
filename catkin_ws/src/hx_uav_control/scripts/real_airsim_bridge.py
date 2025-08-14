#!/usr/bin/env python3

import rospy
import sys
import subprocess
import time

# 安装必要的依赖
def install_requirements():
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "msgpack-rpc-python"])
        subprocess.check_call([sys.executable, "-m", "pip", "install", "airsim"])
        print("依赖安装成功")
        return True
    except Exception as e:
        print(f"依赖安装失败: {e}")
        return False

# 尝试导入airsim，如果失败就安装
try:
    import airsim
    print("AirSim模块已存在")
except ImportError:
    print("正在安装AirSim依赖...")
    if not install_requirements():
        print("无法安装依赖，使用备用方案")
        sys.exit(1)
    import airsim

from hx_msgs.msg import UAVState, UAVControlState, UAVCommand
from geometry_msgs.msg import Vector3, Quaternion
import math

class RealAirSimBridge:
    def __init__(self):
        rospy.init_node('real_airsim_bridge', anonymous=True)
        
        # AirSim客户端
        self.client = None
        self.connected = False
        
        # 当前状态
        self.current_position = [0, 0, 0]
        self.target_position = [0, 0, -3]  # AirSim使用NED坐标系
        self.is_moving = False
        
        # ROS发布器
        self.uav_state_pub = rospy.Publisher('/Drone1/hx_uav/state', UAVState, queue_size=1)
        self.control_state_pub = rospy.Publisher('/Drone1/hx_uav/control_state', UAVControlState, queue_size=1)
        
        # ROS订阅器
        self.command_sub = rospy.Subscriber('/Drone1/hx_uav/command', UAVCommand, self.command_callback)
        
        print("真实AirSim桥接节点启动")
        self.connect_to_airsim()
        
    def connect_to_airsim(self):
        """连接到AirSim"""
        try:
            print("正在连接AirSim...")
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            print("AirSim连接成功，无人机已解锁!")
            self.connected = True
            
            # 获取初始位置
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            self.current_position = [pos.x_val, pos.y_val, pos.z_val]
            print(f"当前位置: {self.current_position}")
            
        except Exception as e:
            print(f"AirSim连接失败: {e}")
            self.connected = False
    
    def command_callback(self, msg):
        """处理ROS控制命令"""
        if not self.connected:
            return
            
        try:
            print(f"收到命令: {msg.Agent_CMD}")
            
            if msg.Agent_CMD == UAVCommand.Takeoff:
                print("执行起飞命令")
                self.client.takeoffAsync().join()
                self.target_position = [0, 0, -3]
                
            elif msg.Agent_CMD == UAVCommand.Land:
                print("执行降落命令")
                self.client.landAsync().join()
                
            elif msg.Agent_CMD == UAVCommand.Move_ENU:
                # ENU坐标转换为NED坐标
                enu_x, enu_y, enu_z = msg.position.x, msg.position.y, msg.position.z
                ned_x, ned_y, ned_z = enu_y, enu_x, -enu_z  # ENU->NED转换
                
                print(f"移动到位置: ENU({enu_x:.1f}, {enu_y:.1f}, {enu_z:.1f}) -> NED({ned_x:.1f}, {ned_y:.1f}, {ned_z:.1f})")
                
                # 异步移动命令
                self.client.moveToPositionAsync(ned_x, ned_y, ned_z, 2.0)  # 2m/s速度
                self.target_position = [ned_x, ned_y, ned_z]
                self.is_moving = True
                
            elif msg.Agent_CMD == UAVCommand.Hold:
                print("悬停命令")
                # 获取当前位置并保持
                state = self.client.getMultirotorState()
                pos = state.kinematics_estimated.position
                self.client.moveToPositionAsync(pos.x_val, pos.y_val, pos.z_val, 1.0)
                
        except Exception as e:
            print(f"命令执行失败: {e}")
    
    def publish_uav_state(self):
        """发布真实的UAV状态"""
        if not self.connected:
            return
            
        try:
            # 获取无人机状态
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            vel = state.kinematics_estimated.linear_velocity
            
            # 更新当前位置
            self.current_position = [pos.x_val, pos.y_val, pos.z_val]
            
            # 创建ROS消息
            uav_state = UAVState()
            uav_state.header.stamp = rospy.Time.now()
            uav_state.header.frame_id = "map"
            
            # 位置 (NED -> ENU 转换)
            uav_state.position[0] = pos.y_val   # East
            uav_state.position[1] = pos.x_val   # North  
            uav_state.position[2] = -pos.z_val  # Up
            
            # 速度
            uav_state.velocity[0] = vel.y_val
            uav_state.velocity[1] = vel.x_val
            uav_state.velocity[2] = -vel.z_val
            
            # 姿态
            uav_state.attitude[0] = 0  # roll
            uav_state.attitude[1] = 0  # pitch  
            uav_state.attitude[2] = 0  # yaw
            
            # 发布状态
            self.uav_state_pub.publish(uav_state)
            
            # 发布控制状态
            control_state = UAVControlState()
            control_state.header.stamp = rospy.Time.now()
            control_state.control_state = 2  # Command控制状态
            control_state.control_enable = True
            control_state.position_control_enable = True
            
            self.control_state_pub.publish(control_state)
            
            # 打印状态信息
            rospy.loginfo_throttle(2, f"位置: ENU({pos.y_val:.1f}, {pos.x_val:.1f}, {-pos.z_val:.1f})")
            
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
        bridge = RealAirSimBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("程序被用户中断")