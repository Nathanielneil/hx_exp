#!/usr/bin/env python3

import rospy
import requests
import json
from hx_msgs.msg import UAVState, UAVControlState, UAVCommand
from geometry_msgs.msg import Vector3, Quaternion
import time
import math

class HTTPAirSimBridge:
    def __init__(self):
        rospy.init_node('http_airsim_bridge', anonymous=True)
        
        # AirSim API配置
        self.base_url = "http://127.0.0.1:41451"
        self.connected = False
        
        # 当前状态
        self.current_position = [0, 0, 0]
        self.sequence_id = 0
        
        # ROS发布器
        self.uav_state_pub = rospy.Publisher('/Drone1/hx_uav/state', UAVState, queue_size=1)
        self.control_state_pub = rospy.Publisher('/Drone1/hx_uav/control_state', UAVControlState, queue_size=1)
        
        # ROS订阅器
        self.command_sub = rospy.Subscriber('/Drone1/hx_uav/command', UAVCommand, self.command_callback)
        
        print("🌐 HTTP AirSim桥接节点启动")
        self.test_connection()
        
    def test_connection(self):
        """测试AirSim连接"""
        try:
            # 尝试连接AirSim API
            response = requests.get(f"{self.base_url}/ping", timeout=2)
            if response.status_code == 200:
                print("✅ AirSim HTTP API连接成功!")
                self.connected = True
                self.enable_api_control()
            else:
                print(f"AirSim API响应错误: {response.status_code}")
        except requests.exceptions.RequestException as e:
            print(f"❌ 无法连接AirSim HTTP API: {e}")
            print("将发布模拟数据以保持ROS程序运行")
            self.connected = False
    
    def enable_api_control(self):
        """启用API控制"""
        try:
            # 启用API控制
            data = {"vehicle_name": "Drone1"}
            response = requests.post(f"{self.base_url}/enableApiControl", 
                                   json=data, timeout=2)
            
            # 解锁无人机
            response = requests.post(f"{self.base_url}/armDisarm", 
                                   json={"arm": True, "vehicle_name": "Drone1"}, timeout=2)
            print("🔓 无人机已解锁，API控制已启用")
            
        except Exception as e:
            print(f"启用API控制失败: {e}")
    
    def command_callback(self, msg):
        """处理ROS控制命令"""
        if not self.connected:
            print(f"收到命令但未连接: {msg.command_type}")
            return
            
        try:
            print(f"📡 收到HTTP命令: {msg.command_type}")
            
            if msg.command_type == UAVCommand.Takeoff:
                self.execute_takeoff()
                
            elif msg.command_type == UAVCommand.Land:
                self.execute_land()
                
            elif msg.command_type == UAVCommand.Move_ENU:
                # ENU坐标转换为NED坐标
                enu_x, enu_y, enu_z = msg.position[0], msg.position[1], msg.position[2]
                ned_x, ned_y, ned_z = enu_y, enu_x, -enu_z  # ENU->NED转换
                
                self.execute_move_to_position(ned_x, ned_y, ned_z)
                
            elif msg.command_type == UAVCommand.Hold:
                self.execute_hover()
                
        except Exception as e:
            print(f"命令执行失败: {e}")
    
    def execute_takeoff(self):
        """执行起飞"""
        try:
            print("🛫 执行HTTP起飞命令")
            data = {"vehicle_name": "Drone1", "timeout_sec": 10}
            response = requests.post(f"{self.base_url}/takeoff", json=data, timeout=10)
            if response.status_code == 200:
                print("✅ 起飞命令发送成功")
            else:
                print(f"起飞命令失败: {response.status_code}")
        except Exception as e:
            print(f"起飞HTTP请求失败: {e}")
    
    def execute_land(self):
        """执行降落"""
        try:
            print("🛬 执行HTTP降落命令")
            data = {"vehicle_name": "Drone1", "timeout_sec": 10}
            response = requests.post(f"{self.base_url}/land", json=data, timeout=10)
            if response.status_code == 200:
                print("✅ 降落命令发送成功")
        except Exception as e:
            print(f"降落HTTP请求失败: {e}")
    
    def execute_move_to_position(self, x, y, z):
        """移动到指定位置"""
        try:
            print(f"🎯 执行HTTP移动命令: NED({x:.1f}, {y:.1f}, {z:.1f})")
            data = {
                "x": float(x),
                "y": float(y), 
                "z": float(z),
                "velocity": 2.0,
                "vehicle_name": "Drone1",
                "timeout_sec": 15
            }
            response = requests.post(f"{self.base_url}/moveToPosition", json=data, timeout=15)
            if response.status_code == 200:
                print("✅ 移动命令发送成功")
            else:
                print(f"移动命令失败: {response.status_code}")
        except Exception as e:
            print(f"移动HTTP请求失败: {e}")
    
    def execute_hover(self):
        """悬停"""
        try:
            print("⏸️ 执行HTTP悬停命令")
            data = {"vehicle_name": "Drone1", "timeout_sec": 1}
            response = requests.post(f"{self.base_url}/hover", json=data, timeout=2)
            if response.status_code == 200:
                print("✅ 悬停命令发送成功")
        except Exception as e:
            print(f"悬停HTTP请求失败: {e}")
    
    def get_position(self):
        """获取当前位置"""
        try:
            response = requests.get(f"{self.base_url}/getMultirotorState?vehicle_name=Drone1", timeout=1)
            if response.status_code == 200:
                data = response.json()
                pos = data.get('kinematics_estimated', {}).get('position', {})
                return [pos.get('x_val', 0), pos.get('y_val', 0), pos.get('z_val', 0)]
        except Exception as e:
            pass
        return self.current_position
    
    def publish_uav_state(self):
        """发布UAV状态"""
        # 尝试获取真实位置
        if self.connected:
            real_pos = self.get_position()
            if real_pos != [0, 0, 0]:
                self.current_position = real_pos
        
        # 创建ROS消息
        uav_state = UAVState()
        uav_state.header.stamp = rospy.Time.now()
        uav_state.header.frame_id = "map"
        
        # 位置 (NED -> ENU 转换)
        ned_x, ned_y, ned_z = self.current_position
        uav_state.position[0] = ned_y    # East
        uav_state.position[1] = ned_x    # North  
        uav_state.position[2] = -ned_z   # Up
        
        # 速度（简化为0）
        uav_state.velocity[0] = 0
        uav_state.velocity[1] = 0
        uav_state.velocity[2] = 0
        
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
        
        if self.connected:
            rospy.loginfo_throttle(3, f"位置: ENU({ned_y:.1f}, {ned_x:.1f}, {-ned_z:.1f})")
        else:
            rospy.loginfo_throttle(5, "发布模拟数据，等待AirSim连接...")
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(10)  # 10Hz (降低频率减少HTTP请求)
        
        while not rospy.is_shutdown():
            if not self.connected:
                self.test_connection()
                
            self.publish_uav_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = HTTPAirSimBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("程序被用户中断")