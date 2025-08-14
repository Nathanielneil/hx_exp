#!/usr/bin/env python3

import csv
import os
import sys
import numpy as np

# 设置matplotlib使用非交互式后端
import matplotlib
matplotlib.use('Agg')  # 使用非GUI后端
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# 尝试导入seaborn，如果失败就跳过
try:
    import seaborn as sns
    sns.set_style("whitegrid")
except ImportError:
    print("警告: 未安装seaborn，使用默认样式")

class GroundEffectAnalyzer:
    def __init__(self, data_dir=None):
        if data_dir is None:
            script_dir = os.path.dirname(__file__)
            self.data_dir = os.path.join(script_dir, '../ground_effect_data')
        else:
            self.data_dir = data_dir
            
        self.controllers = ['PID', 'UDE', 'ADRC']
        self.data = {}
        self.ground_effect_zone = 1.5  # 地面效应区域阈值（米）
        
        # IEEE论文标准字体设置
        plt.rcParams['font.family'] = 'serif'
        plt.rcParams['font.serif'] = ['Times New Roman', 'Times', 'serif']
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.titlesize'] = 12
        plt.rcParams['axes.labelsize'] = 11
        plt.rcParams['xtick.labelsize'] = 10
        plt.rcParams['ytick.labelsize'] = 10
        plt.rcParams['legend.fontsize'] = 10
        plt.rcParams['figure.titlesize'] = 14
        plt.rcParams['axes.unicode_minus'] = False
        
    def load_data(self):
        """加载所有控制器的实验数据"""
        print("加载实验数据...")
        
        for controller in self.controllers:
            filename = f"{controller}_ground_effect_test.csv"
            filepath = os.path.join(self.data_dir, filename)
            
            if not os.path.exists(filepath):
                print(f"警告: 找不到文件 {filepath}")
                continue
                
            data_points = []
            try:
                with open(filepath, 'r') as csvfile:
                    reader = csv.DictReader(csvfile)
                    for row in reader:
                        data_points.append({
                            'timestamp': float(row['timestamp']),
                            'height': float(row['height']),
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'target_height': float(row['target_height'])
                        })
                        
                self.data[controller] = data_points
                print(f"加载 {controller} 控制器数据: {len(data_points)} 个点")
                
            except Exception as e:
                print(f"加载 {controller} 数据失败: {e}")
                
        if not self.data:
            print("错误: 没有找到有效的数据文件")
            return False
            
        return True
    
    def calculate_metrics(self):
        """计算性能指标"""
        self.metrics = {}
        
        for controller, data_points in self.data.items():
            if not data_points:
                continue
                
            # 转换为numpy数组方便计算
            heights = np.array([p['height'] for p in data_points])
            targets = np.array([p['target_height'] for p in data_points])
            x_pos = np.array([p['x'] for p in data_points])
            y_pos = np.array([p['y'] for p in data_points])
            
            # 基本指标
            height_errors = np.abs(heights - targets)
            horizontal_drift = np.sqrt(x_pos**2 + y_pos**2)
            
            # 地面效应区域数据（高度≤1.5m）
            ground_effect_mask = heights <= self.ground_effect_zone
            ge_heights = heights[ground_effect_mask]
            ge_targets = targets[ground_effect_mask]
            ge_horizontal_drift = horizontal_drift[ground_effect_mask]
            
            # 计算指标
            metrics = {
                'avg_height_error': np.mean(height_errors),
                'max_height_error': np.max(height_errors),
                'height_error_std': np.std(height_errors),
                'avg_horizontal_drift': np.mean(horizontal_drift),
                'max_horizontal_drift': np.max(horizontal_drift),
                'final_landing_error': horizontal_drift[-1] if len(horizontal_drift) > 0 else 0,
            }
            
            # 地面效应区域指标
            if len(ge_heights) > 0:
                ge_height_errors = np.abs(ge_heights - ge_targets)
                metrics.update({
                    'ground_effect_height_error': np.mean(ge_height_errors),
                    'ground_effect_drift': np.mean(ge_horizontal_drift),
                    'ground_effect_stability': np.std(ge_height_errors),
                    'ground_effect_points': len(ge_heights)
                })
            else:
                metrics.update({
                    'ground_effect_height_error': 0,
                    'ground_effect_drift': 0,
                    'ground_effect_stability': 0,
                    'ground_effect_points': 0
                })
            
            self.metrics[controller] = metrics
            
        print("性能指标计算完成")
    
    def create_visualizations(self):
        """创建可视化图表"""
        print("生成可视化图表...")
        
        # 创建图表
        fig = plt.figure(figsize=(20, 16))
        
        # 1. 高度跟踪对比
        ax1 = plt.subplot(3, 3, 1)
        self.plot_height_tracking(ax1)
        
        # 2. 水平漂移对比  
        ax2 = plt.subplot(3, 3, 2)
        self.plot_horizontal_drift(ax2)
        
        # 3. 地面效应区域放大图
        ax3 = plt.subplot(3, 3, 3)
        self.plot_ground_effect_zone(ax3)
        
        # 4. 高度误差统计
        ax4 = plt.subplot(3, 3, 4)
        self.plot_height_error_stats(ax4)
        
        # 5. 水平漂移统计
        ax5 = plt.subplot(3, 3, 5)
        self.plot_drift_stats(ax5)
        
        # 6. 地面效应性能对比
        ax6 = plt.subplot(3, 3, 6)
        self.plot_ground_effect_performance(ax6)
        
        # 7. 轨迹对比（俯视图）
        ax7 = plt.subplot(3, 3, 7)
        self.plot_trajectory_comparison(ax7)
        
        # 8. 性能雷达图
        ax8 = plt.subplot(3, 3, 8, projection='polar')
        self.plot_performance_radar(ax8)
        
        # 9. 综合评分
        ax9 = plt.subplot(3, 3, 9)
        self.plot_overall_scores(ax9)
        
        plt.tight_layout()
        
        # 保存图表
        output_path = os.path.join(self.data_dir, 'ground_effect_analysis.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"可视化图表已保存到: {output_path}")
        
        # 不显示图表，只保存
        print("图表已保存，由于无图形界面，不显示图表")
    
    def plot_height_tracking(self, ax):
        """绘制高度跟踪对比"""
        colors = {'PID': 'blue', 'UDE': 'green', 'ADRC': 'red'}
        
        for controller, data_points in self.data.items():
            if not data_points:
                continue
            heights = [p['height'] for p in data_points]
            targets = [p['target_height'] for p in data_points]
            time_indices = range(len(heights))
            
            ax.plot(time_indices, heights, label=f'{controller} 实际', 
                   color=colors[controller], linewidth=2)
            ax.plot(time_indices, targets, '--', label=f'{controller} 目标', 
                   color=colors[controller], alpha=0.7)
        
        # 标记地面效应区域
        ax.axhspan(0, self.ground_effect_zone, alpha=0.2, color='yellow', 
                   label='Ground Effect Zone')
        
        ax.set_xlabel('Time Step')
        ax.set_ylabel('Height (m)')
        ax.set_title('Height Tracking Comparison')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def plot_horizontal_drift(self, ax):
        """绘制水平漂移对比"""
        colors = {'PID': 'blue', 'UDE': 'green', 'ADRC': 'red'}
        
        for controller, data_points in self.data.items():
            if not data_points:
                continue
            drift = [np.sqrt(p['x']**2 + p['y']**2) for p in data_points]
            time_indices = range(len(drift))
            
            ax.plot(time_indices, drift, label=controller, 
                   color=colors[controller], linewidth=2)
        
        ax.set_xlabel('时间步')
        ax.set_ylabel('水平漂移距离 (m)')
        ax.set_title('水平漂移对比')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def plot_ground_effect_zone(self, ax):
        """绘制地面效应区域放大图"""
        colors = {'PID': 'blue', 'UDE': 'green', 'ADRC': 'red'}
        
        for controller, data_points in self.data.items():
            if not data_points:
                continue
            
            # 只显示地面效应区域的数据
            ge_data = [(p['height'], np.sqrt(p['x']**2 + p['y']**2)) 
                       for p in data_points if p['height'] <= self.ground_effect_zone]
            
            if ge_data:
                heights, drifts = zip(*ge_data)
                ax.scatter(heights, drifts, label=controller, 
                          color=colors[controller], alpha=0.7, s=20)
        
        ax.set_xlabel('高度 (m)')
        ax.set_ylabel('水平漂移 (m)')
        ax.set_title('地面效应区域详情 (≤1.5m)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, self.ground_effect_zone)
    
    def plot_height_error_stats(self, ax):
        """绘制高度误差统计"""
        controllers = []
        avg_errors = []
        max_errors = []
        
        for controller in self.controllers:
            if controller in self.metrics:
                controllers.append(controller)
                avg_errors.append(self.metrics[controller]['avg_height_error'])
                max_errors.append(self.metrics[controller]['max_height_error'])
        
        x = np.arange(len(controllers))
        width = 0.35
        
        ax.bar(x - width/2, avg_errors, width, label='平均误差', alpha=0.8)
        ax.bar(x + width/2, max_errors, width, label='最大误差', alpha=0.8)
        
        ax.set_xlabel('控制器')
        ax.set_ylabel('高度误差 (m)')
        ax.set_title('高度误差统计')
        ax.set_xticks(x)
        ax.set_xticklabels(controllers)
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def plot_drift_stats(self, ax):
        """绘制漂移统计"""
        controllers = []
        avg_drifts = []
        max_drifts = []
        
        for controller in self.controllers:
            if controller in self.metrics:
                controllers.append(controller)
                avg_drifts.append(self.metrics[controller]['avg_horizontal_drift'])
                max_drifts.append(self.metrics[controller]['max_horizontal_drift'])
        
        x = np.arange(len(controllers))
        width = 0.35
        
        ax.bar(x - width/2, avg_drifts, width, label='平均漂移', alpha=0.8)
        ax.bar(x + width/2, max_drifts, width, label='最大漂移', alpha=0.8)
        
        ax.set_xlabel('控制器')
        ax.set_ylabel('水平漂移 (m)')
        ax.set_title('水平漂移统计')
        ax.set_xticks(x)
        ax.set_xticklabels(controllers)
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def plot_ground_effect_performance(self, ax):
        """绘制地面效应性能对比"""
        controllers = []
        ge_errors = []
        ge_drifts = []
        
        for controller in self.controllers:
            if controller in self.metrics:
                controllers.append(controller)
                ge_errors.append(self.metrics[controller]['ground_effect_height_error'])
                ge_drifts.append(self.metrics[controller]['ground_effect_drift'])
        
        x = np.arange(len(controllers))
        width = 0.35
        
        ax.bar(x - width/2, ge_errors, width, label='地面效应高度误差', alpha=0.8)
        ax.bar(x + width/2, ge_drifts, width, label='地面效应漂移', alpha=0.8)
        
        ax.set_xlabel('控制器')
        ax.set_ylabel('误差/漂移 (m)')
        ax.set_title('地面效应区域性能对比')
        ax.set_xticks(x)
        ax.set_xticklabels(controllers)
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    def plot_trajectory_comparison(self, ax):
        """绘制轨迹对比（俯视图）"""
        colors = {'PID': 'blue', 'UDE': 'green', 'ADRC': 'red'}
        
        for controller, data_points in self.data.items():
            if not data_points:
                continue
            
            x_pos = [p['x'] for p in data_points]
            y_pos = [p['y'] for p in data_points]
            
            ax.plot(x_pos, y_pos, label=controller, 
                   color=colors[controller], linewidth=2, alpha=0.7)
            
            # 标记起点和终点
            if len(x_pos) > 0:
                ax.plot(x_pos[0], y_pos[0], 'o', color=colors[controller], markersize=8)
                ax.plot(x_pos[-1], y_pos[-1], 's', color=colors[controller], markersize=8)
        
        ax.plot(0, 0, 'k*', markersize=15, label='目标点')
        ax.set_xlabel('X位置 (m)')
        ax.set_ylabel('Y位置 (m)')
        ax.set_title('飞行轨迹对比 (俯视图)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
    
    def plot_performance_radar(self, ax):
        """绘制性能雷达图"""
        if len(self.metrics) < 2:
            ax.text(0.5, 0.5, '数据不足', transform=ax.transAxes, 
                   ha='center', va='center', fontsize=14)
            return
        
        # 性能指标（越小越好的指标需要反转）
        metrics_names = ['高度精度', '水平稳定性', '地面效应抑制', '整体稳定性']
        
        angles = np.linspace(0, 2 * np.pi, len(metrics_names), endpoint=False).tolist()
        angles += angles[:1]  # 闭合雷达图
        
        colors = {'PID': 'blue', 'UDE': 'green', 'ADRC': 'red'}
        
        for controller in self.controllers:
            if controller not in self.metrics:
                continue
                
            m = self.metrics[controller]
            
            # 计算归一化分数（0-1，越接近1越好）
            values = [
                1.0 / (1.0 + m['avg_height_error']),  # 高度精度
                1.0 / (1.0 + m['avg_horizontal_drift']),  # 水平稳定性
                1.0 / (1.0 + m['ground_effect_drift']),  # 地面效应抑制
                1.0 / (1.0 + m['height_error_std'])  # 整体稳定性
            ]
            values += values[:1]  # 闭合
            
            ax.plot(angles, values, 'o-', linewidth=2, label=controller,
                   color=colors[controller])
            ax.fill(angles, values, alpha=0.25, color=colors[controller])
        
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(metrics_names)
        ax.set_ylim(0, 1)
        ax.set_title('综合性能雷达图')
        ax.legend()
        ax.grid(True)
    
    def plot_overall_scores(self, ax):
        """绘制综合评分"""
        controllers = []
        scores = []
        
        for controller in self.controllers:
            if controller not in self.metrics:
                continue
                
            m = self.metrics[controller]
            
            # 计算综合评分（0-100分）
            height_score = max(0, 100 - m['avg_height_error'] * 100)
            drift_score = max(0, 100 - m['avg_horizontal_drift'] * 50)
            stability_score = max(0, 100 - m['height_error_std'] * 200)
            ge_score = max(0, 100 - m['ground_effect_drift'] * 100)
            
            overall_score = (height_score + drift_score + stability_score + ge_score) / 4
            
            controllers.append(controller)
            scores.append(overall_score)
        
        colors = ['blue', 'green', 'red'][:len(controllers)]
        bars = ax.bar(controllers, scores, color=colors, alpha=0.8)
        
        # 添加分数标签
        for bar, score in zip(bars, scores):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 1,
                   f'{score:.1f}', ha='center', va='bottom', fontsize=12, fontweight='bold')
        
        ax.set_ylabel('综合评分')
        ax.set_title('控制器综合评分对比')
        ax.set_ylim(0, 100)
        ax.grid(True, alpha=0.3)
        
        # 添加评分说明
        ax.text(0.02, 0.98, '评分标准:\n90-100: 优秀\n80-90: 良好\n70-80: 一般\n<70: 需改进', 
               transform=ax.transAxes, fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    def generate_detailed_report(self):
        """生成详细分析报告"""
        report_file = os.path.join(self.data_dir, 'detailed_analysis_report.txt')
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("地面效应实验详细分析报告\n")
            f.write("=" * 50 + "\n\n")
            
            # 实验概述
            f.write("实验概述:\n")
            f.write(f"- 测试控制器: {', '.join(self.controllers)}\n")
            f.write(f"- 地面效应区域阈值: {self.ground_effect_zone}m\n")
            f.write(f"- 数据点总数: {sum(len(data) for data in self.data.values())}\n\n")
            
            # 各控制器详细指标
            f.write("控制器性能指标:\n")
            f.write("-" * 30 + "\n")
            
            for controller in self.controllers:
                if controller not in self.metrics:
                    continue
                    
                m = self.metrics[controller]
                f.write(f"\n{controller} 控制器:\n")
                f.write(f"  高度控制性能:\n")
                f.write(f"    - 平均高度误差: {m['avg_height_error']:.3f}m\n")
                f.write(f"    - 最大高度误差: {m['max_height_error']:.3f}m\n")
                f.write(f"    - 高度误差标准差: {m['height_error_std']:.3f}m\n")
                
                f.write(f"  水平位置控制性能:\n")
                f.write(f"    - 平均水平漂移: {m['avg_horizontal_drift']:.3f}m\n")
                f.write(f"    - 最大水平漂移: {m['max_horizontal_drift']:.3f}m\n")
                f.write(f"    - 最终降落误差: {m['final_landing_error']:.3f}m\n")
                
                f.write(f"  地面效应区域性能:\n")
                f.write(f"    - 地面效应区域数据点: {m['ground_effect_points']}个\n")
                f.write(f"    - 地面效应高度误差: {m['ground_effect_height_error']:.3f}m\n")
                f.write(f"    - 地面效应水平漂移: {m['ground_effect_drift']:.3f}m\n")
                f.write(f"    - 地面效应稳定性: {m['ground_effect_stability']:.3f}m\n")
            
            # 性能排名
            f.write(f"\n性能排名:\n")
            f.write("-" * 20 + "\n")
            
            # 高度精度排名
            height_ranking = sorted([(c, m['avg_height_error']) for c, m in self.metrics.items()], 
                                  key=lambda x: x[1])
            f.write("高度控制精度排名 (误差从小到大):\n")
            for i, (controller, error) in enumerate(height_ranking):
                f.write(f"  {i+1}. {controller}: {error:.3f}m\n")
            
            # 水平稳定性排名
            drift_ranking = sorted([(c, m['avg_horizontal_drift']) for c, m in self.metrics.items()], 
                                 key=lambda x: x[1])
            f.write("\n水平稳定性排名 (漂移从小到大):\n")
            for i, (controller, drift) in enumerate(drift_ranking):
                f.write(f"  {i+1}. {controller}: {drift:.3f}m\n")
            
            # 地面效应抑制能力排名
            ge_ranking = sorted([(c, m['ground_effect_drift']) for c, m in self.metrics.items()], 
                              key=lambda x: x[1])
            f.write("\n地面效应抑制能力排名 (漂移从小到大):\n")
            for i, (controller, ge_drift) in enumerate(ge_ranking):
                f.write(f"  {i+1}. {controller}: {ge_drift:.3f}m\n")
            
            # 结论和建议
            f.write(f"\n结论和建议:\n")
            f.write("-" * 20 + "\n")
            
            best_height = height_ranking[0][0]
            best_drift = drift_ranking[0][0]
            best_ge = ge_ranking[0][0]
            
            f.write(f"- 高度控制最佳: {best_height}\n")
            f.write(f"- 水平稳定性最佳: {best_drift}\n")
            f.write(f"- 地面效应抑制最佳: {best_ge}\n\n")
            
            # 应用建议
            f.write("应用建议:\n")
            f.write("- PID控制器: 适合标准飞行任务，调节简单\n")
            f.write("- UDE控制器: 适合有外部扰动的环境\n")
            f.write("- ADRC控制器: 适合复杂环境和要求高精度的任务\n\n")
            
            f.write("注意事项:\n")
            f.write("- 地面效应在高度1.5米以下开始显现\n")
            f.write("- 不同控制器的参数调节会显著影响性能\n")
            f.write("- 实际应用中需要考虑计算资源和实时性要求\n")
        
        print(f"详细分析报告已保存到: {report_file}")
    
    def run_analysis(self):
        """运行完整分析"""
        print("开始地面效应数据分析...")
        
        if not self.load_data():
            return False
            
        self.calculate_metrics()
        self.create_visualizations()
        self.generate_detailed_report()
        
        print("\n分析完成！")
        print(f"输出文件位置: {self.data_dir}")
        return True

if __name__ == '__main__':
    analyzer = GroundEffectAnalyzer()
    analyzer.run_analysis()