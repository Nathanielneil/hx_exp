#!/usr/bin/env python3
"""
控制器对比工具 - 保持原有控制算法不变
Controller Comparison Tool - Keeping original control algorithms unchanged

对比PID、UDE、ADRC三种控制器的性能
Compare performance of PID, UDE, ADRC controllers
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import threading

from hx_airsim_controller import HXAirSimController, ControllerType, ControlMode, TrajectoryPoint

class ControllerComparison:
    """控制器性能对比工具"""
    
    def __init__(self):
        self.controller = HXAirSimController()
        
        # 对比配置
        self.controllers_to_test = [
            (ControllerType.PID, "PID"),
            (ControllerType.UDE, "UDE"), 
            (ControllerType.ADRC, "ADRC")
        ]
        
        # 测试场景
        self.test_scenarios = [
            {
                'name': '位置阶跃响应',
                'type': 'step_response',
                'target': [5.0, 3.0, -3.0, 0.0],  # x, y, z, yaw
                'duration': 20.0
            },
            {
                'name': '圆形轨迹跟踪',
                'type': 'circle_trajectory',
                'center': [0.0, 0.0, -3.0],
                'radius': 4.0,
                'duration': 30.0,
                'num_points': 60
            },
            {
                'name': '8字轨迹跟踪',
                'type': 'figure8_trajectory',
                'center': [0.0, 0.0, -3.0],
                'size': 3.0,
                'duration': 40.0,
                'num_points': 80
            },
            {
                'name': '扰动抑制测试',
                'type': 'disturbance_rejection',
                'hover_point': [0.0, 0.0, -3.0, 0.0],
                'disturbance_time': 10.0,
                'recovery_time': 15.0
            }
        ]
        
        # 数据记录
        self.results = defaultdict(lambda: defaultdict(list))
        self.time_stamps = defaultdict(list)
        
    def initialize(self):
        """初始化控制器"""
        if not self.controller.initialize():
            print("❌ 控制器初始化失败")
            return False
        
        print("✅ 控制器初始化成功")
        return True
    
    def run_comparison(self):
        """运行完整的控制器对比测试"""
        if not self.initialize():
            return False
        
        print("\n" + "="*80)
        print("HX UAV 控制器性能对比测试")
        print("保持原有Prometheus控制算法核心不变")
        print("测试控制器: PID, UDE, ADRC")
        print("="*80)
        
        self.controller.start()
        
        try:
            # 对每个控制器进行测试
            for controller_type, controller_name in self.controllers_to_test:
                print(f"\n测试 {controller_name} 控制器...")
                self._test_controller(controller_type, controller_name)
                
                # 控制器间休息时间
                print("等待系统稳定...")
                time.sleep(3)
            
            # 生成对比报告
            self._generate_comparison_report()
            
            # 绘制对比图表
            self._plot_comparison_results()
            
            return True
            
        except Exception as e:
            print(f"❌ 对比测试出错: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            self.controller.stop()
    
    def _test_controller(self, controller_type, controller_name):
        """测试单个控制器"""
        # 切换到指定控制器
        self.controller.switch_controller(controller_type)
        print(f"  已切换到 {controller_name} 控制器")
        
        # 回到起始位置
        print("  回到起始位置...")
        self.controller.set_target_position(0.0, 0.0, -1.0, 0.0)
        time.sleep(5)
        
        # 对每个测试场景进行测试
        for scenario in self.test_scenarios:
            print(f"  执行测试: {scenario['name']}")
            
            if scenario['type'] == 'step_response':
                self._test_step_response(controller_name, scenario)
            elif scenario['type'] == 'circle_trajectory':
                self._test_circle_trajectory(controller_name, scenario)
            elif scenario['type'] == 'figure8_trajectory':
                self._test_figure8_trajectory(controller_name, scenario)
            elif scenario['type'] == 'disturbance_rejection':
                self._test_disturbance_rejection(controller_name, scenario)
            
            # 场景间休息
            time.sleep(2)
    
    def _test_step_response(self, controller_name, scenario):
        """阶跃响应测试"""
        target = scenario['target']
        duration = scenario['duration']
        
        # 记录数据
        start_time = time.time()
        positions = []
        errors = []
        timestamps = []
        
        # 发送阶跃指令
        self.controller.set_target_position(target[0], target[1], target[2], target[3])
        
        # 数据采集
        while time.time() - start_time < duration:
            current_time = time.time() - start_time
            pos = self.controller.get_position()
            error = np.linalg.norm(np.array(target[:3]) - np.array(pos))
            
            positions.append(pos.copy())
            errors.append(error)
            timestamps.append(current_time)
            
            time.sleep(0.1)  # 10Hz采样
        
        # 存储结果
        scenario_name = scenario['name']
        self.results[controller_name][scenario_name] = {
            'positions': positions,
            'errors': errors,
            'timestamps': timestamps,
            'target': target,
            'final_error': errors[-1] if errors else float('inf'),
            'max_error': max(errors) if errors else float('inf'),
            'avg_error': np.mean(errors) if errors else float('inf'),
            'settling_time': self._calculate_settling_time(timestamps, errors, target[:3])
        }
    
    def _test_circle_trajectory(self, controller_name, scenario):
        """圆形轨迹跟踪测试"""
        center = scenario['center']
        radius = scenario['radius']
        duration = scenario['duration']
        num_points = scenario['num_points']
        
        # 生成圆形轨迹
        trajectory = self._generate_circle_trajectory(center, radius, duration, num_points)
        
        # 执行轨迹跟踪测试
        self._execute_trajectory_test(controller_name, scenario['name'], trajectory, duration)
    
    def _test_figure8_trajectory(self, controller_name, scenario):
        """8字轨迹跟踪测试"""
        center = scenario['center']
        size = scenario['size']
        duration = scenario['duration']
        num_points = scenario['num_points']
        
        # 生成8字轨迹
        trajectory = self._generate_figure8_trajectory(center, size, duration, num_points)
        
        # 执行轨迹跟踪测试
        self._execute_trajectory_test(controller_name, scenario['name'], trajectory, duration)
    
    def _test_disturbance_rejection(self, controller_name, scenario):
        """扰动抑制测试"""
        hover_point = scenario['hover_point']
        disturbance_time = scenario['disturbance_time']
        recovery_time = scenario['recovery_time']
        total_duration = disturbance_time + recovery_time
        
        # 先悬停
        self.controller.set_target_position(hover_point[0], hover_point[1], hover_point[2], hover_point[3])
        time.sleep(3)
        
        # 记录数据
        start_time = time.time()
        positions = []
        errors = []
        timestamps = []
        
        disturbance_applied = False
        
        while time.time() - start_time < total_duration:
            current_time = time.time() - start_time
            pos = self.controller.get_position()
            error = np.linalg.norm(np.array(hover_point[:3]) - np.array(pos))
            
            # 在指定时间施加扰动
            if current_time >= disturbance_time and not disturbance_applied:
                # 模拟扰动 - 发送一个偏离目标的位置指令
                disturbance_pos = [
                    hover_point[0] + 3.0,
                    hover_point[1] + 2.0,
                    hover_point[2],
                    hover_point[3]
                ]
                self.controller.set_target_position(
                    disturbance_pos[0], disturbance_pos[1], 
                    disturbance_pos[2], disturbance_pos[3]
                )
                print(f"    在 {disturbance_time:.1f}s 施加扰动")
                time.sleep(1)  # 短暂扰动
                
                # 恢复原目标
                self.controller.set_target_position(
                    hover_point[0], hover_point[1], hover_point[2], hover_point[3]
                )
                disturbance_applied = True
            
            positions.append(pos.copy())
            errors.append(error)
            timestamps.append(current_time)
            
            time.sleep(0.1)
        
        # 存储结果
        scenario_name = scenario['name']
        self.results[controller_name][scenario_name] = {
            'positions': positions,
            'errors': errors,
            'timestamps': timestamps,
            'disturbance_time': disturbance_time,
            'recovery_time': recovery_time,
            'final_error': errors[-1] if errors else float('inf'),
            'max_error': max(errors) if errors else float('inf'),
            'avg_error': np.mean(errors) if errors else float('inf'),
            'recovery_performance': self._calculate_recovery_performance(
                timestamps, errors, disturbance_time
            )
        }
    
    def _execute_trajectory_test(self, controller_name, scenario_name, trajectory, duration):
        """执行轨迹跟踪测试"""
        # 设置轨迹
        self.controller.set_trajectory(trajectory)
        
        # 记录数据
        start_time = time.time()
        positions = []
        errors = []
        timestamps = []
        trajectory_errors = []
        
        while time.time() - start_time < duration + 2:  # 额外2秒缓冲
            current_time = time.time() - start_time
            pos = self.controller.get_position()
            tracking_error = self.controller.get_tracking_error()
            
            # 计算与当前轨迹点的误差
            current_trajectory_point = self._get_trajectory_point_at_time(trajectory, current_time)
            if current_trajectory_point is not None:
                traj_error = np.linalg.norm(current_trajectory_point.position - np.array(pos))
                trajectory_errors.append(traj_error)
            else:
                trajectory_errors.append(0.0)
            
            positions.append(pos.copy())
            errors.append(tracking_error)
            timestamps.append(current_time)
            
            time.sleep(0.1)
        
        # 存储结果
        self.results[controller_name][scenario_name] = {
            'positions': positions,
            'errors': errors,
            'timestamps': timestamps,
            'trajectory_errors': trajectory_errors,
            'trajectory': trajectory,
            'final_error': errors[-1] if errors else float('inf'),
            'max_error': max(errors) if errors else float('inf'),
            'avg_error': np.mean(errors) if errors else float('inf'),
            'max_trajectory_error': max(trajectory_errors) if trajectory_errors else float('inf'),
            'avg_trajectory_error': np.mean(trajectory_errors) if trajectory_errors else float('inf')
        }
    
    def _generate_circle_trajectory(self, center, radius, duration, num_points):
        """生成圆形轨迹"""
        trajectory = []
        
        for i in range(num_points):
            t = i / (num_points - 1) * duration
            angle = 2 * np.pi * t / duration
            
            # 位置
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]
            
            # 速度
            angular_velocity = 2 * np.pi / duration
            vx = -radius * angular_velocity * np.sin(angle)
            vy = radius * angular_velocity * np.cos(angle)
            vz = 0.0
            
            # 加速度
            ax = -radius * angular_velocity**2 * np.cos(angle)
            ay = -radius * angular_velocity**2 * np.sin(angle)
            az = 0.0
            
            # Yaw角度
            yaw = angle + np.pi / 2
            
            point = TrajectoryPoint(
                position=np.array([x, y, z]),
                velocity=np.array([vx, vy, vz]),
                acceleration=np.array([ax, ay, az]),
                yaw=yaw,
                time_from_start=t
            )
            
            trajectory.append(point)
        
        return trajectory
    
    def _generate_figure8_trajectory(self, center, size, duration, num_points):
        """生成8字轨迹"""
        trajectory = []
        
        for i in range(num_points):
            t = i / (num_points - 1) * duration
            angle = 4 * np.pi * t / duration  # 8字需要两个周期
            
            # 8字轨迹参数方程
            x = center[0] + size * np.sin(angle)
            y = center[1] + size * np.sin(angle) * np.cos(angle)
            z = center[2]
            
            # 速度
            angular_velocity = 4 * np.pi / duration
            vx = size * angular_velocity * np.cos(angle)
            vy = size * angular_velocity * (np.cos(angle)**2 - np.sin(angle)**2)
            vz = 0.0
            
            # 加速度（简化）
            ax = -size * angular_velocity**2 * np.sin(angle)
            ay = -size * angular_velocity**2 * 2 * np.sin(angle) * np.cos(angle)
            az = 0.0
            
            # Yaw角度
            yaw = np.arctan2(vy, vx) if abs(vx) > 1e-6 or abs(vy) > 1e-6 else 0.0
            
            point = TrajectoryPoint(
                position=np.array([x, y, z]),
                velocity=np.array([vx, vy, vz]),
                acceleration=np.array([ax, ay, az]),
                yaw=yaw,
                time_from_start=t
            )
            
            trajectory.append(point)
        
        return trajectory
    
    def _get_trajectory_point_at_time(self, trajectory, time):
        """获取指定时间的轨迹点"""
        if not trajectory:
            return None
        
        # 寻找最接近的时间点
        min_diff = float('inf')
        closest_point = None
        
        for point in trajectory:
            diff = abs(point.time_from_start - time)
            if diff < min_diff:
                min_diff = diff
                closest_point = point
        
        return closest_point
    
    def _calculate_settling_time(self, timestamps, errors, target):
        """计算稳定时间（2%误差范围内）"""
        if not errors or not timestamps:
            return float('inf')
        
        # 目标距离（用于计算2%误差）
        target_distance = np.linalg.norm(target)
        if target_distance == 0:
            target_distance = 1.0  # 避免除零
        
        settling_threshold = 0.02 * target_distance  # 2%误差
        
        # 从后往前查找，找到最后一次超过阈值的时间
        for i in range(len(errors) - 1, -1, -1):
            if errors[i] > settling_threshold:
                if i + 1 < len(timestamps):
                    return timestamps[i + 1]
                else:
                    return timestamps[-1]
        
        return timestamps[0] if timestamps else 0.0
    
    def _calculate_recovery_performance(self, timestamps, errors, disturbance_time):
        """计算扰动恢复性能"""
        if not errors or not timestamps:
            return float('inf')
        
        # 找到扰动后的数据
        recovery_errors = []
        for i, t in enumerate(timestamps):
            if t > disturbance_time:
                recovery_errors.append(errors[i])
        
        if not recovery_errors:
            return float('inf')
        
        # 返回扰动后的平均误差
        return np.mean(recovery_errors)
    
    def _generate_comparison_report(self):
        """生成对比报告"""
        print("\n" + "="*80)
        print("📊 控制器性能对比报告")
        print("="*80)
        
        # 为每个测试场景生成报告
        for scenario in self.test_scenarios:
            scenario_name = scenario['name']
            print(f"\n{scenario_name} 测试结果:")
            print("-" * 60)
            
            # 表头
            print(f"{'控制器':<10} {'最终误差':<12} {'最大误差':<12} {'平均误差':<12} {'备注':<20}")
            print("-" * 60)
            
            # 各控制器数据
            for controller_type, controller_name in self.controllers_to_test:
                if scenario_name in self.results[controller_name]:
                    result = self.results[controller_name][scenario_name]
                    
                    final_error = result['final_error']
                    max_error = result['max_error']
                    avg_error = result['avg_error']
                    
                    extra_info = ""
                    if 'settling_time' in result:
                        extra_info = f"稳定时间:{result['settling_time']:.1f}s"
                    elif 'avg_trajectory_error' in result:
                        extra_info = f"轨迹误差:{result['avg_trajectory_error']:.3f}m"
                    elif 'recovery_performance' in result:
                        extra_info = f"恢复性能:{result['recovery_performance']:.3f}m"
                    
                    print(f"{controller_name:<10} {final_error:<12.3f} {max_error:<12.3f} {avg_error:<12.3f} {extra_info:<20}")
        
        # 总体性能排名
        print(f"\n🏆 总体性能评估:")
        print("-" * 60)
        self._calculate_overall_ranking()
    
    def _calculate_overall_ranking(self):
        """计算总体性能排名"""
        controller_scores = defaultdict(float)
        
        # 为每个控制器在每个场景中的表现打分
        for scenario in self.test_scenarios:
            scenario_name = scenario['name']
            
            # 收集所有控制器在此场景的平均误差
            avg_errors = {}
            for controller_type, controller_name in self.controllers_to_test:
                if scenario_name in self.results[controller_name]:
                    avg_errors[controller_name] = self.results[controller_name][scenario_name]['avg_error']
            
            if avg_errors:
                # 按误差从小到大排序，给分数
                sorted_controllers = sorted(avg_errors.items(), key=lambda x: x[1])
                for rank, (controller_name, error) in enumerate(sorted_controllers):
                    score = len(sorted_controllers) - rank  # 最好的得最高分
                    controller_scores[controller_name] += score
        
        # 按总分排序
        final_ranking = sorted(controller_scores.items(), key=lambda x: x[1], reverse=True)
        
        print("总体排名 (基于所有测试场景的综合表现):")
        for rank, (controller_name, score) in enumerate(final_ranking):
            print(f"  {rank + 1}. {controller_name} 控制器 (总分: {score})")
        
        if final_ranking:
            best_controller = final_ranking[0][0]
            print(f"\n🥇 综合性能最佳: {best_controller} 控制器")
            print("   (在大多数测试场景中表现最优)")
    
    def _plot_comparison_results(self):
        """绘制对比图表"""
        print("\n📈 生成对比图表...")
        
        try:
            import matplotlib.pyplot as plt
            
            # 为每个测试场景创建图表
            for scenario in self.test_scenarios:
                scenario_name = scenario['name']
                
                # 创建子图
                fig, axes = plt.subplots(2, 2, figsize=(15, 10))
                fig.suptitle(f'{scenario_name} - 控制器性能对比', fontsize=16)
                
                # 误差对比
                ax1 = axes[0, 0]
                for controller_type, controller_name in self.controllers_to_test:
                    if scenario_name in self.results[controller_name]:
                        result = self.results[controller_name][scenario_name]
                        ax1.plot(result['timestamps'], result['errors'], 
                                label=f'{controller_name}', linewidth=2)
                
                ax1.set_xlabel('时间 (s)')
                ax1.set_ylabel('跟踪误差 (m)')
                ax1.set_title('跟踪误差对比')
                ax1.legend()
                ax1.grid(True)
                
                # 轨迹对比（如果是轨迹跟踪测试）
                ax2 = axes[0, 1]
                if 'trajectory' in self.results.get(self.controllers_to_test[0][1], {}).get(scenario_name, {}):
                    # 绘制期望轨迹
                    trajectory = self.results[self.controllers_to_test[0][1]][scenario_name]['trajectory']
                    traj_x = [point.position[0] for point in trajectory]
                    traj_y = [point.position[1] for point in trajectory]
                    ax2.plot(traj_x, traj_y, 'k--', linewidth=3, label='期望轨迹')
                    
                    # 绘制实际轨迹
                    for controller_type, controller_name in self.controllers_to_test:
                        if scenario_name in self.results[controller_name]:
                            positions = self.results[controller_name][scenario_name]['positions']
                            pos_x = [pos[0] for pos in positions]
                            pos_y = [pos[1] for pos in positions]
                            ax2.plot(pos_x, pos_y, label=f'{controller_name}', linewidth=2)
                    
                    ax2.set_xlabel('X (m)')
                    ax2.set_ylabel('Y (m)')
                    ax2.set_title('轨迹跟踪对比')
                    ax2.legend()
                    ax2.grid(True)
                    ax2.axis('equal')
                else:
                    ax2.text(0.5, 0.5, '非轨迹跟踪测试', ha='center', va='center', transform=ax2.transAxes)
                    ax2.set_title('轨迹对比')
                
                # 性能指标对比
                ax3 = axes[1, 0]
                controllers = []
                final_errors = []
                max_errors = []
                avg_errors = []
                
                for controller_type, controller_name in self.controllers_to_test:
                    if scenario_name in self.results[controller_name]:
                        result = self.results[controller_name][scenario_name]
                        controllers.append(controller_name)
                        final_errors.append(result['final_error'])
                        max_errors.append(result['max_error'])
                        avg_errors.append(result['avg_error'])
                
                x = np.arange(len(controllers))
                width = 0.25
                
                ax3.bar(x - width, final_errors, width, label='最终误差')
                ax3.bar(x, max_errors, width, label='最大误差')
                ax3.bar(x + width, avg_errors, width, label='平均误差')
                
                ax3.set_xlabel('控制器')
                ax3.set_ylabel('误差 (m)')
                ax3.set_title('性能指标对比')
                ax3.set_xticks(x)
                ax3.set_xticklabels(controllers)
                ax3.legend()
                ax3.grid(True, axis='y')
                
                # 统计信息
                ax4 = axes[1, 1]
                ax4.axis('off')
                
                stats_text = "统计信息:\n\n"
                for controller_type, controller_name in self.controllers_to_test:
                    if scenario_name in self.results[controller_name]:
                        result = self.results[controller_name][scenario_name]
                        stats_text += f"{controller_name}:\n"
                        stats_text += f"  最终误差: {result['final_error']:.3f} m\n"
                        stats_text += f"  最大误差: {result['max_error']:.3f} m\n"
                        stats_text += f"  平均误差: {result['avg_error']:.3f} m\n"
                        if 'settling_time' in result:
                            stats_text += f"  稳定时间: {result['settling_time']:.1f} s\n"
                        stats_text += "\n"
                
                ax4.text(0.1, 0.9, stats_text, transform=ax4.transAxes, fontsize=10, verticalalignment='top')
                
                plt.tight_layout()
                
                # 保存图表
                filename = f"controller_comparison_{scenario_name.replace(' ', '_')}.png"
                plt.savefig(filename, dpi=300, bbox_inches='tight')
                print(f"  已保存图表: {filename}")
                
                # 显示图表（可选）
                # plt.show()
                
                plt.close()
            
        except ImportError:
            print("matplotlib未安装，跳过图表生成")
        except Exception as e:
            print(f"图表生成出错: {e}")

def main():
    """主函数"""
    print("HX UAV 控制器性能对比工具")
    print("保持原有Prometheus控制算法核心不变")
    print("\n注意: 请确保AirSim正在运行")
    
    comparison_tool = ControllerComparison()
    
    try:
        success = comparison_tool.run_comparison()
        if success:
            print("\n🎉 控制器对比测试完成!")
            print("📊 详细报告已生成")
            print("📈 对比图表已保存")
        else:
            print("\n❌ 对比测试未完全成功")
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
    except Exception as e:
        print(f"\n💥 测试异常终止: {e}")

if __name__ == "__main__":
    main()