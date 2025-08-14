#!/usr/bin/env python3

import rospy
from hx_msgs.msg import UAVState, UAVControlState, UAVCommand
from geometry_msgs.msg import Vector3, Quaternion
import time

class FinalSimpleBridge:
    def __init__(self):
        rospy.init_node('final_simple_bridge', anonymous=True)
        
        # 当前状态
        self.current_position = [0.0, 0.0, 3.0]  # 初始位置：空中3米
        self.target_position = [0.0, 0.0, 3.0]
        self.is_moving = False
        self.move_speed = 1.0  # 1m/s
        
        # ROS发布器
        self.uav_state_pub = rospy.Publisher('/Drone1/hx_uav/state', UAVState, queue_size=1)
        self.control_state_pub = rospy.Publisher('/Drone1/hx_uav/control_state', UAVControlState, queue_size=1)
        
        # ROS订阅器
        self.command_sub = rospy.Subscriber('/Drone1/hx_uav/command', UAVCommand, self.command_callback)
        
        print("🚁 最终简化桥接节点启动")
        print("✅ 模拟无人机控制，支持位置移动")
        
        self.last_time = time.time()
        
    def command_callback(self, msg):
        """处理ROS控制命令"""
        try:
            cmd = msg.Agent_CMD
            print(f"📡 收到命令: {cmd}")
            
            if cmd == UAVCommand.Takeoff:
                print("🛫 执行起飞 -> 位置(0, 0, 3)")
                self.target_position = [0.0, 0.0, 3.0]
                self.is_moving = True
                
            elif cmd == UAVCommand.Land:
                print("🛬 执行降落 -> 位置(0, 0, 0)")
                self.target_position = [0.0, 0.0, 0.0]
                self.is_moving = True
                
            elif cmd == UAVCommand.Move_ENU:
                # 获取目标位置
                target_x = msg.position.x
                target_y = msg.position.y
                target_z = msg.position.z
                
                print(f"🎯 移动到位置: ENU({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
                self.target_position = [target_x, target_y, target_z]
                self.is_moving = True
                
            elif cmd == UAVCommand.Hold:
                print("⏸️ 悬停在当前位置")
                self.target_position = self.current_position.copy()
                self.is_moving = False
                
        except Exception as e:
            print(f"命令处理失败: {e}")
    
    def update_position(self):
        """更新无人机位置（模拟移动）"""
        if not self.is_moving:
            return
            
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 计算到目标的距离
        dx = self.target_position[0] - self.current_position[0]
        dy = self.target_position[1] - self.current_position[1] 
        dz = self.target_position[2] - self.current_position[2]
        
        distance = (dx**2 + dy**2 + dz**2)**0.5
        
        if distance < 0.1:  # 接近目标
            self.current_position = self.target_position.copy()
            self.is_moving = False
            print(f"✅ 到达目标位置: ({self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f})")
        else:
            # 按固定速度移动
            move_distance = self.move_speed * dt
            if move_distance > distance:
                move_distance = distance
                
            # 计算移动方向
            if distance > 0:
                move_x = (dx / distance) * move_distance
                move_y = (dy / distance) * move_distance  
                move_z = (dz / distance) * move_distance
                
                self.current_position[0] += move_x
                self.current_position[1] += move_y
                self.current_position[2] += move_z
    
    def publish_uav_state(self):
        """发布UAV状态"""
        # 更新位置
        self.update_position()
        
        # 创建UAV状态消息
        uav_state = UAVState()
        uav_state.header.stamp = rospy.Time.now()
        uav_state.header.frame_id = "map"
        
        # 位置
        uav_state.position[0] = self.current_position[0]  # East
        uav_state.position[1] = self.current_position[1]  # North
        uav_state.position[2] = self.current_position[2]  # Up
        
        # 速度（简化）
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
        
        # 定期打印位置
        rospy.loginfo_throttle(2, f"当前位置: ENU({self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f})")
    
    def run(self):
        """主循环"""
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown():
            self.publish_uav_state()
            rate.sleep()

if __name__ == '__main__':
    try:
        bridge = FinalSimpleBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("程序被用户中断")