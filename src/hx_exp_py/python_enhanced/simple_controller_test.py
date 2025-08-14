#!/usr/bin/env python3
"""
简化的控制器测试 - 避免复杂的多线程问题
Simple Controller Test - Avoid complex multi-threading issues
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'controllers'))

import airsim
import numpy as np
import time
from controllers.pid_controller import PIDController
from controllers.ude_controller import UDEController
from controllers.adrc_controller import ADRCController

class SimpleControllerTest:
    """简化的控制器测试"""
    
    def __init__(self):
        self.client = None
        self.controllers = {}
        
    def initialize(self):
        """初始化AirSim连接"""
        try:
            print("正在连接到AirSim...")
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            
            print("启用API控制...")
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            
            # 初始化控制器
            self.initialize_controllers()
            
            return True
            
        except Exception as e:
            print(f"初始化失败: {e}")
            return False
    
    def initialize_controllers(self):
        """初始化控制器"""
        # 通用参数
        common_params = {
            'quad_mass': 1.5,
            'hov_percent': 0.5,  # 降低悬停油门
            'tilt_angle_max': 10.0,  # 降低最大倾斜角
            'pxy_int_max': 0.3,
            'pz_int_max': 0.3,
        }
        
        # PID控制器
        pid_controller = PIDController()
        pid_params = {
            **common_params,
            'Kp_xy': 1.0,  # 降低增益
            'Kp_z': 1.5,
            'Kv_xy': 1.0,  # 降低增益
            'Kv_z': 1.5,
            'Kvi_xy': 0.1,
            'Kvi_z': 0.1
        }
        pid_controller.init_from_dict(pid_params)
        self.controllers['PID'] = pid_controller
        
        # UDE控制器
        ude_controller = UDEController()
        ude_params = {
            **common_params,
            'Kp_xy': 0.3,  # 降低增益
            'Kp_z': 0.5,
            'Kd_xy': 1.0,  # 降低增益
            'Kd_z': 1.5,
            'T_ude': 1.5
        }
        ude_controller.init_from_dict(ude_params)
        self.controllers['UDE'] = ude_controller
        
        # ADRC控制器
        adrc_controller = ADRCController()
        adrc_params = {
            **common_params,
            'beta_max': 0.3,  # 降低参数
            'C1': 0.3,
            'C2': 0.3,
            'sigmaD': 0.3,
            'amesogain_l': 0.3,
            'method_choose': 1
        }
        adrc_controller.init_from_dict(adrc_params)
        self.controllers['ADRC'] = adrc_controller
        
        print("控制器初始化完成")
    
    def get_state(self):
        """获取当前状态，转换为NED坐标系"""
        try:
            # 获取位置和姿态
            pose = self.client.simGetVehiclePose()
            # AirSim使用NED坐标系，Z轴负值表示高度
            position = np.array([pose.position.x_val, pose.position.y_val, pose.position.z_val])
            
            # 获取速度
            velocity_body = self.client.getMultirotorState().kinematics_estimated.linear_velocity
            velocity = np.array([velocity_body.x_val, velocity_body.y_val, velocity_body.z_val])
            
            # 获取姿态（四元数）
            orientation = pose.orientation
            quat = np.array([orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val])
            
            return position, velocity, quat
            
        except Exception as e:
            print(f"状态获取失败: {e}")
            return np.zeros(3), np.zeros(3), np.array([1,0,0,0])
    
    def send_control_command(self, roll, pitch, yaw, throttle):
        """发送控制命令"""
        try:
            # 限制控制量
            roll = np.clip(roll, -0.2, 0.2)      # ±11度
            pitch = np.clip(pitch, -0.2, 0.2)    # ±11度
            throttle = np.clip(throttle, 0.3, 0.7)  # 限制油门范围
            
            # 使用正确的控制接口 - 直接设置姿态角和油门
            self.client.moveByRollPitchYawThrottleAsync(
                roll=roll, pitch=pitch, yaw=yaw, throttle=throttle, duration=0.1
            )
            
        except Exception as e:
            print(f"控制命令发送失败: {e}")
    
    def test_controller(self, controller_name, duration=20):
        """测试单个控制器"""
        print(f"\n测试 {controller_name} 控制器...")
        
        controller = self.controllers[controller_name]
        
        # 起飞到安全高度
        print("  起飞中...")
        self.client.takeoffAsync().join()
        time.sleep(5)
        
        # 移动到起始位置 (使用我们的控制器)
        print("  使用控制器悬停到起始位置...")
        # 先短暂悬停，让控制器接管
        for i in range(20):  # 2秒的控制器初始化
            pos, vel, quat = self.get_state()
            controller.set_current_state(pos, vel, quat)
            controller.set_desired_state(pos, np.zeros(3), np.zeros(3), 0.0)  # 就地悬停
            u_att = controller.update(10.0)  # 10Hz
            self.send_control_command(u_att[0], u_att[1], u_att[2], u_att[3])
            time.sleep(0.1)
        
        print("  控制器已接管，开始测试...")
        
        # 获取当前位置作为起点
        current_pos, _, _ = self.get_state()
        
        # 设置目标位置 (相对当前位置的小幅移动，保持合理高度)
        target_pos = current_pos + np.array([1.5, 1.0, 0.0])  # 水平移动
        # 确保高度在合理范围内(AirSim中-3到-1米为合理飞行高度)
        if target_pos[2] > -1.0:
            target_pos[2] = -2.0  # 设置到2米高度
        target_vel = np.zeros(3)
        target_acc = np.zeros(3)
        target_yaw = 0.0
        
        print(f"  起始位置: [{current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f}]")
        print(f"  目标位置: [{target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f}]")
        
        # 记录数据
        positions = []
        errors = []
        timestamps = []
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                current_time = time.time() - start_time
                
                # 获取当前状态
                pos, vel, quat = self.get_state()
                
                # 设置控制器状态
                controller.set_current_state(pos, vel, quat)
                controller.set_desired_state(target_pos, target_vel, target_acc, target_yaw)
                
                # 计算控制输出
                u_att = controller.update(10.0)  # 10Hz控制频率
                
                # 打印控制输出（调试用）
                if int(current_time * 10) % 20 == 0:  # 每2秒打印一次
                    print(f"      控制输出: roll={u_att[0]*180/3.14:.1f}°, pitch={u_att[1]*180/3.14:.1f}°, throttle={u_att[3]:.3f}")
                
                # 发送控制命令
                self.send_control_command(u_att[0], u_att[1], u_att[2], u_att[3])
                
                # 记录数据
                error = np.linalg.norm(target_pos - pos)
                positions.append(pos.copy())
                errors.append(error)
                timestamps.append(current_time)
                
                # 打印状态
                if int(current_time) % 2 == 0 and current_time - int(current_time) < 0.1:
                    print(f"    {current_time:4.1f}s: 位置=[{pos[0]:5.2f}, {pos[1]:5.2f}, {pos[2]:5.2f}], 误差={error:.3f}m")
                
                # 安全检查
                if abs(pos[2]) > 10 or error > 10:  # 如果飞得太高或误差太大
                    print("    安全限制触发，停止测试")
                    break
                
                time.sleep(0.05)  # 20Hz
                
        except KeyboardInterrupt:
            print("    用户中断测试")
        except Exception as e:
            print(f"    测试过程出错: {e}")
        
        # 返回结果
        if errors:
            result = {
                'controller': controller_name,
                'final_error': errors[-1],
                'max_error': max(errors),
                'avg_error': np.mean(errors),
                'positions': positions,
                'errors': errors,
                'timestamps': timestamps
            }
            
            print(f"  测试完成:")
            print(f"    最终误差: {result['final_error']:.3f} m")
            print(f"    最大误差: {result['max_error']:.3f} m")
            print(f"    平均误差: {result['avg_error']:.3f} m")
            
            return result
        else:
            return None
    
    def safe_land(self):
        """安全降落"""
        try:
            print("安全降落中...")
            self.client.landAsync().join()
            time.sleep(2)
        except Exception as e:
            print(f"降落失败: {e}")
    
    def cleanup(self):
        """清理资源"""
        try:
            self.safe_land()
            if self.client:
                self.client.enableApiControl(False)
        except Exception as e:
            print(f"清理资源失败: {e}")

def main():
    """主函数"""
    print("HX 简化控制器测试")
    print("保持原有Prometheus控制算法核心不变")
    print("\n注意: 请确保AirSim正在运行且无人机在空旷区域")
    
    tester = SimpleControllerTest()
    
    if not tester.initialize():
        return
    
    results = {}
    
    try:
        # 测试每个控制器
        for controller_name in ['PID', 'UDE', 'ADRC']:
            result = tester.test_controller(controller_name, duration=15)
            if result:
                results[controller_name] = result
            
            # 控制器间的休息时间
            print("等待系统稳定...")
            tester.safe_land()
            time.sleep(5)
        
        # 打印对比结果
        print("\n" + "="*60)
        print("📊 控制器性能对比结果")
        print("="*60)
        print(f"{'控制器':<10} {'最终误差':<12} {'最大误差':<12} {'平均误差':<12}")
        print("-" * 60)
        
        for controller_name, result in results.items():
            print(f"{controller_name:<10} {result['final_error']:<12.3f} {result['max_error']:<12.3f} {result['avg_error']:<12.3f}")
        
        # 找出最佳控制器
        if results:
            best_controller = min(results.items(), key=lambda x: x[1]['avg_error'])
            print(f"\n🏆 平均误差最小的控制器: {best_controller[0]}")
            print(f"   平均误差: {best_controller[1]['avg_error']:.3f} m")
        
        print("\n✅ 测试完成！")
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"\n💥 测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.cleanup()

if __name__ == "__main__":
    main()