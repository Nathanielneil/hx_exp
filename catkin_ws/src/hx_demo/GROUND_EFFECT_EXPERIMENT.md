# 地面效应实验指南

## 🎯 实验目的

本实验旨在评估不同控制算法（PID、UDE、ADRC）在无人机降落过程中应对地面效应的性能差异。

## 📖 地面效应简介

**地面效应（Ground Effect）**是指无人机在接近地面时，螺旋桨下洗气流遇到地面反射，导致：
- **升力增加**: 有效升力增大，无人机更容易"浮起"
- **阻力减小**: 诱导阻力降低
- **控制敏感性变化**: 控制响应可能变得不稳定
- **水平漂移**: 由于气流不均匀可能导致水平位置偏移

通常在高度小于螺旋桨直径1-2倍时（约1.5米以下）开始显现。

## 🔬 评价指标

### 主要性能指标

1. **水平位置稳定性**
   - `avg_horizontal_drift`: 平均水平漂移距离
   - `max_horizontal_drift`: 最大水平漂移距离
   - `final_landing_error`: 最终降落位置误差

2. **高度控制精度**
   - `avg_height_error`: 平均高度跟踪误差
   - `max_height_error`: 最大高度跟踪误差
   - `height_error_std`: 高度误差标准差

3. **速度控制稳定性**
   - `avg_velocity_error`: 平均垂直速度误差
   - `velocity_error_std`: 垂直速度误差标准差

4. **控制输出平滑性**
   - `roll_cmd_std`: Roll控制命令标准差
   - `pitch_cmd_std`: Pitch控制命令标准差
   - `throttle_cmd_std`: 油门控制命令标准差

### 地面效应专项指标

5. **地面效应区域性能**（高度≤1.5m）
   - `ground_effect_horizontal_drift`: 地面效应区域水平漂移
   - `ground_effect_height_error`: 地面效应区域高度误差
   - `ground_effect_roll_variation`: 地面效应区域Roll控制变化
   - `ground_effect_pitch_variation`: 地面效应区域Pitch控制变化

## 🚀 快速开始

### 基础实验

```bash
# 1. 启动AirSim环境
cd /home/ubuntu/NGW/data/code_hx/src/hx_exp_rc
./scripts/start_airsim.sh

# 2. 运行地面效应实验（默认参数）
roslaunch hx_demo ground_effect_experiment.launch

# 3. 查看结果
# 数据保存在: hx_demo/ground_effect_data/
# 包含CSV数据文件、分析报告和对比图表
```

### 自定义参数实验

```bash
# 慢速降落实验（更明显的地面效应）
roslaunch hx_demo ground_effect_experiment.launch \
  takeoff_height:=4.0 \
  landing_speed:=0.3 \
  hover_duration:=8.0

# 快速降落实验（更具挑战性）
roslaunch hx_demo ground_effect_experiment.launch \
  takeoff_height:=3.0 \
  landing_speed:=0.8 \
  hover_duration:=3.0

# 仅测试特定控制器（PID=1, UDE=2, ADRC=3）
roslaunch hx_demo ground_effect_experiment.launch \
  controllers:="[1]" \
  auto_repeat:=false

# 自定义数据保存路径
roslaunch hx_demo ground_effect_experiment.launch \
  data_dir:="/path/to/your/data"
```

## 📊 预定义实验场景

使用配置文件运行预定义场景：

```bash
# 慢速降落场景
rosparam load $(rospack find hx_demo)/config/ground_effect_scenarios.yaml slow_landing
roslaunch hx_demo ground_effect_experiment.launch \
  takeoff_height:=4.0 landing_speed:=0.3 hover_duration:=8.0

# 精密测试场景
rosparam load $(rospack find hx_demo)/config/ground_effect_scenarios.yaml precision_test
roslaunch hx_demo ground_effect_experiment.launch \
  takeoff_height:=3.0 landing_speed:=0.2 hover_duration:=10.0

# 鲁棒性测试场景
rosparam load $(rospack find hx_demo)/config/ground_effect_scenarios.yaml robustness_test
roslaunch hx_demo ground_effect_experiment.launch \
  takeoff_height:=4.0 landing_speed:=0.7 hover_duration:=2.0
```

## 📈 实验流程

### 自动化实验流程

1. **起飞阶段**: 使用指定控制器起飞到目标高度
2. **悬停阶段**: 在目标高度悬停稳定，消除起飞扰动
3. **降落阶段**: 以恒定速度缓慢降落，记录所有数据
4. **分析阶段**: 降落完成后分析数据并生成报告
5. **切换控制器**: 自动切换到下一个控制器重复实验

### 实验参数建议

| 场景 | 起飞高度 | 降落速度 | 悬停时间 | 适用场合 |
|------|----------|----------|----------|----------|
| **标准测试** | 3.0m | 0.5m/s | 5s | 常规对比 |
| **精密测试** | 3.0m | 0.2m/s | 10s | 高精度要求 |
| **鲁棒性测试** | 4.0m | 0.8m/s | 2s | 挑战性环境 |
| **地面效应强化** | 2.0m | 0.4m/s | 6s | 突出地面效应 |

## 📋 结果分析

### 输出文件

实验完成后会生成以下文件：

```
ground_effect_data/
├── PID_landing_data_20231214_143022.csv      # PID控制器数据
├── UDE_landing_data_20231214_143155.csv      # UDE控制器数据  
├── ADRC_landing_data_20231214_143328.csv     # ADRC控制器数据
├── ground_effect_report_20231214_143500.txt  # 综合分析报告
└── comparison_plots_20231214_143500.png      # 对比图表
```

### 数据文件格式

CSV文件包含以下字段：
- `timestamp`: 时间戳
- `controller`: 控制器名称
- `target_height, actual_height`: 目标/实际高度
- `target_x/y, actual_x/y`: 目标/实际水平位置
- `velocity_x/y/z`: 三轴速度
- `roll_cmd, pitch_cmd, throttle_cmd`: 控制命令
- `horizontal_drift`: 水平漂移距离
- `height_error`: 高度误差
- `velocity_z_error`: 垂直速度误差

### 分析报告内容

1. **实验参数记录**
2. **各控制器详细性能指标**
3. **对比分析和排名**
4. **最佳控制器推荐**

## 🎯 预期结果分析

### 典型控制器特性

**PID控制器**
- ✅ 响应速度快，调节简单
- ❌ 地面效应下可能振荡
- 📊 预期：中等的水平稳定性，较快的响应

**UDE控制器**  
- ✅ 扰动估计能力强
- ✅ 对地面效应扰动有一定补偿
- 📊 预期：较好的地面效应抑制能力

**ADRC控制器**
- ✅ 自抗扰能力最强
- ✅ 理论上最适合处理地面效应
- 📊 预期：最佳的整体性能和稳定性

### 评价维度

1. **稳定性排序**: 通常 ADRC > UDE > PID
2. **精度排序**: 取决于参数调节，可能有变化
3. **响应速度**: 通常 PID > UDE ≈ ADRC
4. **鲁棒性**: ADRC > UDE > PID

## 🔧 高级使用

### 自定义评价指标

修改Python脚本中的`analyze_current_experiment()`函数来添加自定义指标。

### 批量实验

```bash
#!/bin/bash
# 批量运行多个场景

scenarios=("slow_landing" "fast_landing" "precision_test" "robustness_test")

for scenario in "${scenarios[@]}"; do
    echo "Running $scenario experiment..."
    # 根据scenario加载对应参数并运行实验
    roslaunch hx_demo ground_effect_experiment.launch # 加相应参数
    sleep 5  # 等待实验完成
done
```

### 实时监控

```bash
# 实时查看实验数据
rostopic echo /Drone1/hx_uav/state
rostopic echo /Drone1/hx_uav/controller_output

# 查看实验进度
tail -f ground_effect_data/ground_effect_report_*.txt
```

## 🐛 故障排除

### 常见问题

1. **无人机不降落**
   - 检查降落速度设置是否过小
   - 确认AirSim物理参数正确

2. **数据文件为空**
   - 检查文件权限
   - 确认Python依赖（pandas, matplotlib）已安装

3. **控制器切换失败**
   - 检查控制器参数配置
   - 确认ROS话题连接正常

4. **实验中断**
   - 使用`Ctrl+C`安全停止
   - 部分结果仍会保存到数据文件

### 参数调节建议

- **降落速度过快**: 减小`landing_speed`参数
- **地面效应不明显**: 降低`takeoff_height`或减小`landing_speed`  
- **数据采集不足**: 增加`hover_duration`参数
- **控制器不稳定**: 检查控制器参数配置

## 📊 结果解读指导

### 优秀表现指标

- 水平漂移 < 0.2m
- 高度误差 < 0.1m  
- 速度误差 < 0.1m/s
- 控制命令标准差 < 0.1rad

### 性能对比方法

1. **定量对比**: 使用生成的数值指标
2. **定性观察**: 观看实际飞行过程
3. **统计分析**: 多次实验取平均值
4. **可视化分析**: 查看生成的对比图表

---

**开始你的地面效应控制器性能评估实验！** 🚁

通过这个实验，你将深入了解不同控制算法在复杂飞行环境下的性能差异，为实际应用选择最适合的控制策略。