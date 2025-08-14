#!/usr/bin/env python3

import rospy
import time
from hx_msgs.msg import UAVCommand, UAVState, UAVControlState
from geometry_msgs.msg import Point
import csv
import os

class SimpleGroundEffectTest:
    def __init__(self):
        rospy.init_node('simple_ground_effect_test', anonymous=True)
        
        # 测试参数
        self.takeoff_height = 4.0
        self.landing_speed = 0.4  # 慢速降落以观察地面效应
        self.hover_duration = 5.0
        
        # 控制器列表 (PID=1, UDE=2, ADRC=3)
        self.controllers = [1, 2, 3]
        self.controller_names = {1: 'PID', 2: 'UDE', 3: 'ADRC'}
        
        # ROS发布器和订阅器
        self.cmd_pub = rospy.Publisher('/Drone1/hx_uav/command', UAVCommand, queue_size=1)
        self.state_sub = rospy.Subscriber('/Drone1/hx_uav/state', UAVState, self.state_callback)
        
        # 状态变量
        self.current_position = [0, 0, 0]
        self.experiment_data = []
        self.current_controller = None
        self.test_phase = "idle"
        
        # 数据保存路径
        self.data_dir = os.path.join(os.path.dirname(__file__), '../ground_effect_data')
        os.makedirs(self.data_dir, exist_ok=True)
        
        print("简化地面效应测试启动")
        print(f"测试参数: 起飞高度={self.takeoff_height}m, 降落速度={self.landing_speed}m/s")
        
    def state_callback(self, msg):
        """接收无人机状态"""
        self.current_position = [msg.position[0], msg.position[1], msg.position[2]]
        
        # 记录实验数据
        if self.test_phase == "landing":
            data_point = {
                'timestamp': time.time(),
                'controller': self.controller_names.get(self.current_controller, 'Unknown'),
                'height': msg.position[2],
                'x': msg.position[0], 
                'y': msg.position[1],
                'target_height': max(0.2, self.takeoff_height - (time.time() - self.landing_start_time) * self.landing_speed)
            }
            self.experiment_data.append(data_point)
    
    def send_command(self, cmd_type, position=None, controller_mode=1):
        """发送控制命令"""
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.Agent_CMD = cmd_type
        cmd.Control_mode = controller_mode
        
        if position:
            cmd.position = Point()
            cmd.position.x = position[0]
            cmd.position.y = position[1] 
            cmd.position.z = position[2]
            
        self.cmd_pub.publish(cmd)
        
    def wait_for_position(self, target_pos, tolerance=0.3, timeout=30):
        """等待到达指定位置"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not rospy.is_shutdown():
                dist = ((self.current_position[0] - target_pos[0])**2 + 
                       (self.current_position[1] - target_pos[1])**2 + 
                       (self.current_position[2] - target_pos[2])**2)**0.5
                if dist < tolerance:
                    return True
                rospy.sleep(0.1)
        return False
    
    def test_single_controller(self, controller_id):
        """测试单个控制器"""
        controller_name = self.controller_names[controller_id]
        print(f"\n开始测试 {controller_name} 控制器")
        
        self.current_controller = controller_id
        self.experiment_data = []
        
        # 1. 起飞
        print("步骤1: 起飞")
        self.test_phase = "takeoff"
        self.send_command(UAVCommand.Takeoff, controller_mode=controller_id)
        rospy.sleep(3)  # 等待起飞
        
        # 2. 移动到起飞高度
        print(f"步骤2: 移动到起飞高度 {self.takeoff_height}m")
        self.send_command(UAVCommand.Move_ENU, [0, 0, self.takeoff_height], controller_id)
        if not self.wait_for_position([0, 0, self.takeoff_height]):
            print(f"警告: 未能到达目标高度")
            
        # 3. 悬停
        print(f"步骤3: 悬停 {self.hover_duration}s")
        self.test_phase = "hover"
        self.send_command(UAVCommand.Hold, controller_mode=controller_id)
        rospy.sleep(self.hover_duration)
        
        # 4. 缓慢降落 (模拟地面效应测试)
        print("步骤4: 缓慢降落 (地面效应测试)")
        self.test_phase = "landing"
        self.landing_start_time = time.time()
        
        # 分阶段降落以模拟地面效应
        landing_heights = [3.0, 2.0, 1.5, 1.0, 0.5, 0.2]  # 地面效应通常在1.5米以下开始明显
        
        for target_height in landing_heights:
            if target_height >= self.takeoff_height:
                continue
                
            print(f"  降落到 {target_height}m")
            self.send_command(UAVCommand.Move_ENU, [0, 0, target_height], controller_id)
            
            # 在低高度时降低速度以观察地面效应
            if target_height <= 1.5:
                wait_time = 3.0  # 在地面效应区域等待更长时间
                print(f"    进入地面效应区域，等待观察...")
            else:
                wait_time = 2.0
                
            rospy.sleep(wait_time)
        
        # 5. 最终降落
        print("步骤5: 最终降落")
        self.send_command(UAVCommand.Land, controller_mode=controller_id)
        rospy.sleep(5)
        
        # 6. 保存数据
        self.save_test_data(controller_name)
        print(f"{controller_name} 控制器测试完成")
        
        # 短暂休息
        rospy.sleep(2)
        
    def save_test_data(self, controller_name):
        """保存测试数据"""
        if not self.experiment_data:
            print("警告: 无数据可保存")
            return
            
        filename = f"{controller_name}_ground_effect_test.csv"
        filepath = os.path.join(self.data_dir, filename)
        
        with open(filepath, 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'controller', 'height', 'x', 'y', 'target_height']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for data in self.experiment_data:
                writer.writerow(data)
        
        print(f"数据已保存到: {filepath}")
        print(f"记录了 {len(self.experiment_data)} 个数据点")
    
    def run_experiment(self):
        """运行完整实验"""
        print("开始地面效应对比实验")
        print("=" * 50)
        
        # 等待连接
        rospy.sleep(2)
        
        for controller_id in self.controllers:
            try:
                self.test_single_controller(controller_id)
            except Exception as e:
                print(f"控制器 {self.controller_names[controller_id]} 测试失败: {e}")
                continue
        
        print("\n实验完成!")
        print(f"数据保存位置: {self.data_dir}")
        print("可以查看CSV文件分析不同控制器在降落过程中的表现")
        
        # 生成简单的分析报告
        self.generate_simple_report()
    
    def generate_simple_report(self):
        """生成简单的分析报告"""
        report_file = os.path.join(self.data_dir, "ground_effect_report.txt")
        
        with open(report_file, 'w') as f:
            f.write("地面效应实验报告\n")
            f.write("=" * 30 + "\n\n")
            f.write(f"实验参数:\n")
            f.write(f"- 起飞高度: {self.takeoff_height}m\n")
            f.write(f"- 降落速度: {self.landing_speed}m/s\n") 
            f.write(f"- 悬停时间: {self.hover_duration}s\n")
            f.write(f"- 测试控制器: {[self.controller_names[c] for c in self.controllers]}\n\n")
            
            f.write("实验说明:\n")
            f.write("- 地面效应通常在高度1.5米以下开始显现\n")
            f.write("- 表现为升力增加，可能导致无人机难以降落\n")
            f.write("- 不同控制器应对地面效应的能力不同\n\n")
            
            f.write("数据文件:\n")
            for controller_id in self.controllers:
                controller_name = self.controller_names[controller_id]
                f.write(f"- {controller_name}_ground_effect_test.csv\n")
        
        print(f"实验报告已保存到: {report_file}")

if __name__ == '__main__':
    try:
        test = SimpleGroundEffectTest()
        test.run_experiment()
    except rospy.ROSInterruptException:
        print("实验被中断")
    except KeyboardInterrupt:
        print("实验被用户中断")