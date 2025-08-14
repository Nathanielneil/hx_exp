# HX ROS-AirSim系统使用指南

本指南提供HX ROS-AirSim无人机控制系统的详细使用说明。

## 🚀 快速启动

### 方法1: 分步启动（推荐）

```bash
# 1. 启动AirSim环境
cd /home/ubuntu/NGW/data/code_hx/src/hx_exp_rc
./scripts/start_airsim.sh

# 2. 等待AirSim完全启动后，新开终端启动ROS系统
source ~/catkin_ws/devel/setup.bash
roslaunch hx_uav_control hx_uav_control_main.launch

# 3. 运行演示程序（可选）
roslaunch hx_demo basic_position_control.launch
```

### 方法2: 一键启动

```bash
# 启动完整系统并运行基础演示
roslaunch hx_exp_rc hx_complete_system.launch demo_mode:=basic

# 启动完整系统并运行圆形轨迹演示
roslaunch hx_exp_rc hx_complete_system.launch demo_mode:=circle

# 启动完整系统并运行Python演示
roslaunch hx_exp_rc hx_complete_system.launch demo_mode:=python

# 启动完整系统进入手动控制模式
roslaunch hx_exp_rc hx_complete_system.launch demo_mode:=manual
```

## 🎮 手动控制

### 交互式命令工具

```bash
# 启动交互式控制工具
rosrun hx_uav_control hx_command_pub

# 菜单选项:
# 1 - 起飞
# 2 - 降落  
# 3 - 移动到指定位置
# 4 - 悬停
# 5 - 圆形轨迹
# 6 - 8字轨迹
# 7 - 切换到PID控制器
# 8 - 切换到UDE控制器
# 9 - 切换到ADRC控制器
```

### 命令行直接控制

```bash
# 起飞
rosrun hx_uav_control hx_command_pub takeoff

# 移动到指定位置 (x, y, z)
rosrun hx_uav_control hx_command_pub move 2.0 2.0 3.0

# 悬停
rosrun hx_uav_control hx_command_pub hold

# 降落
rosrun hx_uav_control hx_command_pub land

# 圆形轨迹
rosrun hx_uav_control hx_command_pub circle

# 8字轨迹
rosrun hx_uav_control hx_command_pub eight
```

## 📊 系统监控

### 查看系统状态

```bash
# 查看UAV状态
rostopic echo /Drone1/hx_uav/state

# 查看控制状态
rostopic echo /Drone1/hx_uav/control_state

# 查看控制器输出
rostopic echo /Drone1/hx_uav/controller_output

# 查看AirSim状态
rostopic echo /Drone1/hx_uav/airsim_state
```

### 可视化工具

```bash
# 查看话题连接图
rqt_graph

# 查看话题列表
rostopic list

# 实时监控话题频率
rostopic hz /Drone1/hx_uav/state

# 启动RViz可视化
rviz
```

## 🔧 系统配置

### 主要参数文件

编辑 `hx_uav_control/config/hx_controller_params.yaml`:

```yaml
# 控制频率
control_frequency: 50.0
state_frequency: 20.0

# PID控制器参数
pid_gain:
  Kp_xy: 0.8          # 水平位置增益
  Kv_xy: 0.5          # 水平速度增益
  quad_mass: 1.5      # 无人机质量

# 安全参数  
safety:
  max_velocity: 5.0   # 最大速度
  max_height: 20.0    # 最大高度
  flight_boundary: 50.0  # 飞行边界
```

### AirSim配置

AirSim配置文件位置：`airsim_env/settings.json`

关键配置项：
- RPC端口: 41451
- 车辆名称: Drone1
- API控制: 启用
- 传感器: IMU, GPS, 磁力计, 气压计

## 🏃‍♂️ 运行演示程序

### 1. 基础位置控制演示

```bash
# 演示内容: 起飞 → 方形路径 → 降落
roslaunch hx_demo basic_position_control.launch
```

### 2. 圆形轨迹演示

```bash  
# 演示内容: 起飞 → 圆形轨迹 → 控制器切换 → 降落
roslaunch hx_demo circle_trajectory.launch
```

### 3. Python脚本演示

```bash
# 演示内容: 起飞 → 三角形路径 → 降落
rosrun hx_demo takeoff_land_demo.py
```

## 🎯 控制器切换

系统支持三种控制算法在线切换：

### PID控制器
- **特点**: 经典PID控制，调节简单
- **适用**: 标准飞行任务
- **切换**: `rosrun hx_uav_control hx_command_pub` 选择选项7

### UDE控制器  
- **特点**: 不确定性和扰动估计
- **适用**: 有外部扰动环境
- **切换**: `rosrun hx_uav_control hx_command_pub` 选择选项8

### ADRC控制器
- **特点**: 自抗扰控制，鲁棒性强
- **适用**: 复杂动态环境  
- **切换**: `rosrun hx_uav_control hx_command_pub` 选择选项9

## 🐛 故障排除

### 常见问题1: AirSim连接失败

```bash
# 检查AirSim是否运行
netstat -tlnp | grep 41451

# 检查AirSim进程
ps aux | grep Blocks

# 重启AirSim
./scripts/start_airsim.sh
```

### 常见问题2: 控制无响应

```bash
# 检查话题连接
rostopic list | grep hx_uav

# 检查Python环境
conda activate jz
python3 -c "import airsim; print('AirSim API OK')"

# 重启控制节点
rosnode kill /hx_airsim_interface_python
roslaunch hx_uav_control hx_uav_control_main.launch
```

### 常见问题3: 编译错误

```bash
# 检查依赖
rosdep install --from-paths src --ignore-src -r -y

# 清理编译
cd ~/catkin_ws
catkin_make clean
catkin_make

# 检查环境变量
echo $ROS_PACKAGE_PATH
```

### 常见问题4: 消息无法接收

```bash
# 检查话题发布频率
rostopic hz /Drone1/hx_uav/state

# 检查消息内容
rostopic echo /Drone1/hx_uav/state --noarr

# 检查节点状态
rosnode info /hx_airsim_interface_python
```

## 📈 性能调优

### 控制频率优化

根据硬件性能调整频率：

```yaml
# 高性能机器
control_frequency: 100.0
state_frequency: 50.0

# 一般机器  
control_frequency: 50.0
state_frequency: 20.0

# 低性能机器
control_frequency: 30.0
state_frequency: 10.0
```

### 控制器参数调优

**PID参数调优原则**:
1. 先调Kp，观察响应速度
2. 再调Kd，减少震荡
3. 最后调Ki，消除稳态误差

**安全参数设置**:
- 室内: `flight_boundary: 5.0`
- 室外: `flight_boundary: 50.0`
- 测试: `max_velocity: 2.0`
- 正式: `max_velocity: 5.0`

## 🔄 多机控制

```bash
# 启动3架无人机
roslaunch hx_uav_control hx_multi_uav.launch num_uavs:=3

# 分别控制不同无人机
rosrun hx_uav_control hx_command_pub /Drone1/hx_uav/command
rosrun hx_uav_control hx_command_pub /Drone2/hx_uav/command
rosrun hx_uav_control hx_command_pub /Drone3/hx_uav/command
```

## 📝 日志和数据记录

### 启用ROS bag记录

```bash
# 记录所有话题
rosbag record -a

# 记录特定话题
rosbag record /Drone1/hx_uav/state /Drone1/hx_uav/controller_output

# 播放记录
rosbag play your_bag_file.bag
```

### 查看系统日志

```bash
# 查看ROS日志
roscd && cd ../log
tail -f latest/hx_uav_control_node-*.log

# 查看Python节点日志
roscd && cd ../log  
tail -f latest/hx_airsim_interface_python-*.log
```

---

## ⚡ 使用技巧

1. **开发调试**: 使用`roslaunch`的`--screen`参数查看详细输出
2. **性能监控**: 使用`htop`监控CPU使用率  
3. **网络延迟**: 本地运行AirSim以减少网络延迟
4. **参数调试**: 使用`rosparam set`动态调整参数
5. **安全第一**: 始终在安全环境中测试新参数

**系统就绪，开始你的无人机控制之旅！** 🚁