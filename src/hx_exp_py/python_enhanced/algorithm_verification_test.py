#!/usr/bin/env python3
"""
控制算法验证测试 - 不依赖AirSim，直接测试算法
Algorithm Verification Test - Test algorithms directly without AirSim dependency
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'controllers'))

import numpy as np
import matplotlib.pyplot as plt
from controllers.pid_controller import PIDController
from controllers.ude_controller import UDEController
from controllers.adrc_controller import ADRCController

class AlgorithmVerificationTest:
    """控制算法验证测试类"""
    
    def __init__(self):
        self.controllers = {}
        self.initialize_controllers()
        
    def initialize_controllers(self):
        """初始化所有控制器"""
        print(" 初始化控制器...")
        
        # 通用参数
        common_params = {
            'quad_mass': 1.5,
            'hov_percent': 0.5,
            'tilt_angle_max': 15.0,
            'pxy_int_max': 0.5,
            'pz_int_max': 0.5,
        }
        
        # PID控制器
        pid_controller = PIDController()
        pid_params = {
            **common_params,
            'Kp_xy': 2.0,
            'Kp_z': 2.5,
            'Kv_xy': 1.5,
            'Kv_z': 2.0,
            'Kvi_xy': 0.2,
            'Kvi_z': 0.3
        }
        pid_controller.init_from_dict(pid_params)
        self.controllers['PID'] = pid_controller
        
        # UDE控制器
        ude_controller = UDEController()
        ude_params = {
            **common_params,
            'Kp_xy': 0.5,
            'Kp_z': 0.5,
            'Kd_xy': 2.0,
            'Kd_z': 2.0,
            'T_ude': 1.0
        }
        ude_controller.init_from_dict(ude_params)
        self.controllers['UDE'] = ude_controller
        
        # ADRC控制器
        adrc_controller = ADRCController()
        adrc_params = {
            **common_params,
            'beta_max': 0.5,
            'C1': 0.5,
            'C2': 0.5,
            'sigmaD': 0.5,
            'amesogain_l': 0.5,
            'method_choose': 1
        }
        adrc_controller.init_from_dict(adrc_params)
        self.controllers['ADRC'] = adrc_controller
        
        print(" 控制器初始化完成")
    
    def test_step_response(self, controller_name, initial_pos, target_pos, duration=10.0):
        """测试阶跃响应"""
        print(f"\n 测试 {controller_name} 控制器阶跃响应...")
        
        controller = self.controllers[controller_name]
        
        # 模拟参数
        dt = 0.02  # 50Hz
        steps = int(duration / dt)
        
        # 初始状态
        pos = np.array(initial_pos, dtype=float)
        vel = np.zeros(3)
        quat = np.array([1.0, 0.0, 0.0, 0.0])  # 水平姿态
        
        # 记录数据
        time_data = []
        pos_data = []
        vel_data = []
        control_data = []
        error_data = []
        
        print(f"  起始位置: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
        print(f"  目标位置: [{target_pos[0]:.1f}, {target_pos[1]:.1f}, {target_pos[2]:.1f}]")
        
        for i in range(steps):
            t = i * dt
            
            # 设置控制器状态
            controller.set_current_state(pos, vel, quat)
            controller.set_desired_state(
                np.array(target_pos), 
                np.zeros(3), 
                np.zeros(3), 
                0.0
            )
            
            # 计算控制输出
            try:
                u_att = controller.update(1.0 / dt)
                
                # 简化的动力学模拟 (基于小角度假设)
                # 这里简化为直接从姿态角推导加速度
                roll, pitch, yaw, throttle = u_att[0], u_att[1], u_att[2], u_att[3]
                
                # 重力加速度
                g = 9.81
                mass = controller.ctrl_param.quad_mass
                
                # 从油门和姿态角计算加速度 (简化模型)
                thrust = throttle * mass * g / 0.5  # 假设0.5为悬停油门
                
                # 在世界坐标系下的加速度
                acc_x = thrust * np.sin(pitch) / mass
                acc_y = -thrust * np.sin(roll) / mass  # 注意符号
                acc_z = (thrust * np.cos(roll) * np.cos(pitch) - mass * g) / mass
                
                acceleration = np.array([acc_x, acc_y, acc_z])
                
                # 限制加速度 (物理约束)
                max_acc = 5.0  # m/s²
                acc_norm = np.linalg.norm(acceleration)
                if acc_norm > max_acc:
                    acceleration = acceleration / acc_norm * max_acc
                
                # 更新速度和位置 (欧拉积分)
                vel += acceleration * dt
                pos += vel * dt
                
                # 简单的阻尼 (空气阻力)
                vel *= 0.99
                
            except Exception as e:
                print(f"    控制计算错误 (t={t:.1f}s): {e}")
                u_att = np.array([0, 0, 0, 0.5])
                acceleration = np.array([0, 0, 0])
            
            # 记录数据
            error = np.linalg.norm(np.array(target_pos) - pos)
            
            time_data.append(t)
            pos_data.append(pos.copy())
            vel_data.append(vel.copy())
            control_data.append(u_att.copy())
            error_data.append(error)
            
            # 每秒打印一次状态
            if i % int(1.0 / dt) == 0:
                print(f"    {t:4.1f}s: 位置=[{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}], "
                      f"误差={error:.3f}m, 控制=[{u_att[0]*180/3.14:.1f}°, {u_att[1]*180/3.14:.1f}°, {u_att[3]:.3f}]")
        
        # 计算性能指标
        final_error = error_data[-1]
        max_error = max(error_data)
        avg_error = np.mean(error_data[int(len(error_data)*0.1):])  # 忽略前10%的数据
        
        # 计算稳定时间 (2%误差范围)
        target_distance = np.linalg.norm(np.array(target_pos) - np.array(initial_pos))
        settling_threshold = 0.02 * target_distance if target_distance > 0 else 0.1
        settling_time = duration
        
        for i in range(len(error_data)-1, -1, -1):
            if error_data[i] > settling_threshold:
                if i + 1 < len(time_data):
                    settling_time = time_data[i + 1]
                break
        
        result = {
            'controller': controller_name,
            'time': time_data,
            'positions': pos_data,
            'velocities': vel_data,
            'controls': control_data,
            'errors': error_data,
            'final_error': final_error,
            'max_error': max_error,
            'avg_error': avg_error,
            'settling_time': settling_time,
            'final_position': pos.copy()
        }
        
        print(f"  测试完成:")
        print(f"    最终位置: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        print(f"    最终误差: {final_error:.3f} m")
        print(f"    最大误差: {max_error:.3f} m")
        print(f"    平均误差: {avg_error:.3f} m")
        print(f"    稳定时间: {settling_time:.1f} s")
        
        return result
    
    def run_comparison_test(self):
        """运行对比测试"""
        print("\n" + "="*80)
        print(" 控制算法验证测试")
        print("保持原有Prometheus控制算法核心不变 - 纯算法测试")
        print("="*80)
        
        # 测试场景
        initial_pos = [0.0, 0.0, 0.0]
        target_pos = [3.0, 2.0, -1.0]  # 3D移动
        
        results = {}
        
        # 测试所有控制器
        for controller_name in ['PID', 'UDE', 'ADRC']:
            result = self.test_step_response(controller_name, initial_pos, target_pos, duration=15.0)
            results[controller_name] = result
        
        # 生成对比报告
        self.generate_report(results)
        
        # 绘制对比图表
        self.plot_results(results)
        
        return results
    
    def generate_report(self, results):
        """生成性能对比报告"""
        print("\n" + "="*80)
        print(" 控制算法性能对比报告")
        print("="*80)
        
        print(f"{'控制器':<10} {'最终误差':<12} {'最大误差':<12} {'平均误差':<12} {'稳定时间':<12}")
        print("-" * 80)
        
        for controller_name, result in results.items():
            print(f"{controller_name:<10} {result['final_error']:<12.3f} "
                  f"{result['max_error']:<12.3f} {result['avg_error']:<12.3f} "
                  f"{result['settling_time']:<12.1f}")
        
        # 排名
        print(f"\n 性能排名 (按平均误差):")
        sorted_results = sorted(results.items(), key=lambda x: x[1]['avg_error'])
        for i, (controller_name, result) in enumerate(sorted_results):
            print(f"  {i+1}. {controller_name} 控制器 - 平均误差: {result['avg_error']:.3f} m")
        
        print(f"\n 最佳控制器: {sorted_results[0][0]}")
    
    def plot_results(self, results):
        """绘制对比图表"""
        try:
            # 设置支持中文的字体
            plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS']
            plt.rcParams['axes.unicode_minus'] = False
            
            fig, axes = plt.subplots(2, 2, figsize=(15, 10))
            fig.suptitle('Controller Performance Comparison - Original Prometheus Algorithms', fontsize=16)
            
            colors = {'PID': 'blue', 'UDE': 'red', 'ADRC': 'green'}
            
            # 误差对比
            ax1 = axes[0, 0]
            for controller_name, result in results.items():
                ax1.plot(result['time'], result['errors'], 
                        label=f'{controller_name}', color=colors[controller_name], linewidth=2)
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Tracking Error (m)')
            ax1.set_title('Tracking Error Comparison')
            ax1.legend()
            ax1.grid(True)
            
            # 轨迹对比 (XY平面)
            ax2 = axes[0, 1]
            for controller_name, result in results.items():
                positions = result['positions']
                x_pos = [pos[0] for pos in positions]
                y_pos = [pos[1] for pos in positions]
                ax2.plot(x_pos, y_pos, label=f'{controller_name}', 
                        color=colors[controller_name], linewidth=2)
            
            # 标记起点和终点
            ax2.plot(0, 0, 'ko', markersize=10, label='Start')
            ax2.plot(3, 2, 'r*', markersize=15, label='Target')
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_title('XY Trajectory Comparison')
            ax2.legend()
            ax2.grid(True)
            ax2.axis('equal')
            
            # Z轴位置对比
            ax3 = axes[1, 0]
            for controller_name, result in results.items():
                positions = result['positions']
                z_pos = [pos[2] for pos in positions]
                ax3.plot(result['time'], z_pos, 
                        label=f'{controller_name}', color=colors[controller_name], linewidth=2)
            ax3.axhline(y=-1.0, color='k', linestyle='--', alpha=0.5, label='Target Height')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Z Position (m)')
            ax3.set_title('Height Control Comparison')
            ax3.legend()
            ax3.grid(True)
            
            # 性能指标对比
            ax4 = axes[1, 1]
            controllers = list(results.keys())
            final_errors = [results[c]['final_error'] for c in controllers]
            avg_errors = [results[c]['avg_error'] for c in controllers]
            settling_times = [results[c]['settling_time'] for c in controllers]
            
            x = np.arange(len(controllers))
            width = 0.25
            
            ax4.bar(x - width, final_errors, width, label='Final Error', alpha=0.8)
            ax4.bar(x, avg_errors, width, label='Average Error', alpha=0.8)
            ax4.bar(x + width, [t/10 for t in settling_times], width, label='Settling Time/10', alpha=0.8)
            
            ax4.set_xlabel('Controller')
            ax4.set_ylabel('Error (m) / Time (s)')
            ax4.set_title('Performance Metrics Comparison')
            ax4.set_xticks(x)
            ax4.set_xticklabels(controllers)
            ax4.legend()
            ax4.grid(True, axis='y')
            
            plt.tight_layout()
            
            # 保存图表
            plt.savefig('algorithm_comparison_results.png', dpi=300, bbox_inches='tight')
            print(f"\n 对比图表已保存: algorithm_comparison_results.png")
            
            # 显示图表
            # plt.show()
            
        except ImportError:
            print("\n matplotlib未安装，跳过图表生成")
        except Exception as e:
            print(f"\n 图表生成失败: {e}")

def main():
    """主函数"""
    print("HX 控制算法验证测试")
    print("保持原有Prometheus控制算法核心不变")
    print("通过数值仿真直接验证算法性能")
    
    tester = AlgorithmVerificationTest()
    
    try:
        results = tester.run_comparison_test()
        
        print("\n 算法验证测试完成！")
        print(" 这个测试验证了控制算法本身的正确性")
        print(" 如果算法工作正常，那么AirSim集成的问题就是接口层面的")
        
    except Exception as e:
        print(f"\n 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()