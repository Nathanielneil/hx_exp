#!/usr/bin/env python3
"""
HX AirSim控制器 - 基于Python的增强版本
HX AirSim Controller - Enhanced Python version

保持原有控制算法核心不变，增加以下功能：
- 键盘控制
- 轨迹跟踪 
- 控制器对比
- 演示程序
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'controllers'))

import airsim
import numpy as np
import time
import threading
import json
from typing import Optional, List, Tuple, Dict, Any
from dataclasses import dataclass
from enum import Enum

from controllers.pid_controller import PIDController
from controllers.ude_controller import UDEController
from controllers.adrc_controller import ADRCController

class ControlMode(Enum):
    """控制模式枚举"""
    MANUAL = "MANUAL"
    POSITION_HOLD = "POSITION_HOLD"
    VELOCITY_CONTROL = "VELOCITY_CONTROL"
    TRAJECTORY_CONTROL = "TRAJECTORY_CONTROL"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"

class ControllerType(Enum):
    """控制器类型枚举"""
    PID = "PID"
    UDE = "UDE"
    ADRC = "ADRC"

@dataclass
class UAVState:
    """UAV状态信息"""
    position: np.ndarray = None
    velocity: np.ndarray = None
    orientation: np.ndarray = None  # quaternion [w, x, y, z]
    angular_velocity: np.ndarray = None
    timestamp: float = 0.0
    connected: bool = False
    armed: bool = False
    
    def __post_init__(self):
        if self.position is None:
            self.position = np.zeros(3)
        if self.velocity is None:
            self.velocity = np.zeros(3)
        if self.orientation is None:
            self.orientation = np.array([1.0, 0.0, 0.0, 0.0])
        if self.angular_velocity is None:
            self.angular_velocity = np.zeros(3)

@dataclass
class TrajectoryPoint:
    """轨迹点"""
    position: np.ndarray
    velocity: np.ndarray = None
    acceleration: np.ndarray = None
    yaw: float = 0.0
    time_from_start: float = 0.0
    
    def __post_init__(self):
        if self.velocity is None:
            self.velocity = np.zeros(3)
        if self.acceleration is None:
            self.acceleration = np.zeros(3)

class HXAirSimController:
    """HX AirSim控制器主类"""
    
    def __init__(self, vehicle_name="Drone1"):
        """初始化控制器"""
        self.vehicle_name = vehicle_name
        self.client = None
        self.running = False
        
        # 控制状态
        self.control_mode = ControlMode.MANUAL
        self.controller_type = ControllerType.PID
        
        # UAV状态
        self.uav_state = UAVState()
        self.target_position = np.zeros(3)
        self.target_velocity = np.zeros(3)
        self.target_yaw = 0.0
        
        # 控制器
        self.pid_controller = None
        self.ude_controller = None
        self.adrc_controller = None
        self.current_controller = None
        
        # 轨迹跟踪
        self.trajectory_points = []
        self.current_trajectory_index = 0
        self.trajectory_start_time = 0
        
        # 控制参数
        self.control_frequency = 50.0  # Hz
        self.position_tolerance = 0.2  # m
        self.velocity_tolerance = 0.1  # m/s
        
        # 安全参数
        self.max_velocity = 5.0  # m/s
        self.max_acceleration = 2.0  # m/s²
        self.safety_enabled = True
        
        # 线程控制
        self.control_thread = None
        self.state_update_thread = None
        
    def initialize(self) -> bool:
        """初始化AirSim连接和控制器"""
        try:
            print("正在连接到AirSim...")
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            
            print("启用API控制...")
            self.client.enableApiControl(True, self.vehicle_name)
            self.client.armDisarm(True, self.vehicle_name)
            
            # 初始化控制器
            self.initialize_controllers()
            
            print("HX AirSim控制器初始化完成")
            return True
            
        except Exception as e:
            print(f"初始化失败: {e}")
            return False
    
    def initialize_controllers(self):
        """初始化所有控制器"""
        # 通用参数 (可从配置文件加载)
        common_params = {
            'quad_mass': 1.5,
            'hov_percent': 0.6,  # AirSim的悬停油门通常较高
            'tilt_angle_max': 15.0,
            'pxy_int_max': 0.5,
            'pz_int_max': 0.5,
        }
        
        # 初始化PID控制器 - 保持原有算法不变
        self.pid_controller = PIDController()
        pid_params = {
            **common_params,
            'Kp_xy': 3.0,
            'Kp_z': 3.5,
            'Kv_xy': 2.0,
            'Kv_z': 2.5,
            'Kvi_xy': 0.2,
            'Kvi_z': 0.3
        }
        self.pid_controller.init_from_dict(pid_params)
        
        # 初始化UDE控制器 - 保持原有算法不变
        self.ude_controller = UDEController()
        ude_params = {
            **common_params,
            'Kp_xy': 0.5,
            'Kp_z': 0.5,
            'Kd_xy': 2.0,
            'Kd_z': 2.0,
            'T_ude': 1.0
        }
        self.ude_controller.init_from_dict(ude_params)
        
        # 初始化ADRC控制器 - 保持原有算法不变
        self.adrc_controller = ADRCController()
        adrc_params = {
            **common_params,
            'beta_max': 0.5,
            'C1': 0.5,
            'C2': 0.5,
            'sigmaD': 0.5,
            'amesogain_l': 0.5,
            'method_choose': 1
        }
        self.adrc_controller.init_from_dict(adrc_params)
        
        # 默认使用PID控制器
        self.current_controller = self.pid_controller
        
        print("控制器初始化完成 - 保持原有算法不变")
    
    def start(self):
        """启动控制系统"""
        if self.running:
            return
            
        self.running = True
        
        # 启动状态更新线程
        self.state_update_thread = threading.Thread(target=self._state_update_loop)
        self.state_update_thread.daemon = True
        self.state_update_thread.start()
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print("HX控制系统已启动")
    
    def stop(self):
        """停止控制系统"""
        self.running = False
        
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
        if self.state_update_thread:
            self.state_update_thread.join(timeout=2.0)
            
        if self.client:
            self.client.enableApiControl(False, self.vehicle_name)
            
        print("HX控制系统已停止")
    
    def _state_update_loop(self):
        """状态更新循环"""
        while self.running:
            try:
                self._update_uav_state()
                time.sleep(1.0 / 20.0)  # 20Hz状态更新
            except Exception as e:
                print(f"状态更新错误: {e}")
                time.sleep(0.1)
    
    def _control_loop(self):
        """主控制循环"""
        while self.running:
            try:
                if self.control_mode != ControlMode.MANUAL:
                    self._execute_control()
                time.sleep(1.0 / self.control_frequency)
            except Exception as e:
                print(f"控制循环错误: {e}")
                time.sleep(0.1)
    
    def _update_uav_state(self):
        """更新UAV状态"""
        try:
            # 获取状态
            kinematics = self.client.simGetGroundTruthKinematics(self.vehicle_name)
            
            # 位置 (NED -> ENU)
            pos_ned = kinematics.position
            self.uav_state.position = np.array([pos_ned.x_val, -pos_ned.y_val, -pos_ned.z_val])
            
            # 速度 (NED -> ENU)
            vel_ned = kinematics.linear_velocity
            self.uav_state.velocity = np.array([vel_ned.x_val, -vel_ned.y_val, -vel_ned.z_val])
            
            # 姿态 (四元数)
            q_ned = kinematics.orientation
            self.uav_state.orientation = np.array([q_ned.w_val, q_ned.x_val, -q_ned.y_val, -q_ned.z_val])
            
            # 角速度
            ang_vel = kinematics.angular_velocity
            self.uav_state.angular_velocity = np.array([ang_vel.x_val, -ang_vel.y_val, -ang_vel.z_val])
            
            self.uav_state.timestamp = time.time()
            self.uav_state.connected = True
            self.uav_state.armed = True
            
        except Exception as e:
            print(f"状态更新失败: {e}")
            self.uav_state.connected = False
    
    def _execute_control(self):
        """执行控制逻辑，带安全检查"""
        try:
            # 安全检查
            if not self._safety_check():
                print("WARNING: 安全检查失败，切换到紧急模式")
                self.set_control_mode(ControlMode.EMERGENCY)
                return
            
            if self.control_mode == ControlMode.POSITION_HOLD:
                self._execute_position_control()
            elif self.control_mode == ControlMode.VELOCITY_CONTROL:
                self._execute_velocity_control()
            elif self.control_mode == ControlMode.TRAJECTORY_CONTROL:
                self._execute_trajectory_control()
            elif self.control_mode == ControlMode.LANDING:
                self._execute_landing()
            elif self.control_mode == ControlMode.EMERGENCY:
                self._execute_emergency()
                
        except Exception as e:
            print(f"控制执行错误: {e}")
            self.set_control_mode(ControlMode.EMERGENCY)
    
    def _safety_check(self):
        """安全检查"""
        if self.uav_state.position is None:
            return False
        
        # 高度检查 - 避免过低或过高
        height = -self.uav_state.position[2]  # NED坐标系，Z负值是高度
        if height < 0.5:  # 太低，可能撞地
            print(f"WARNING: 高度过低: {height:.2f}m")
            # 自动增加高度
            self.target_position[2] = -2.0  # 提升到2米
            return True
        elif height > 20.0:  # 太高
            print(f"WARNING: 高度过高: {height:.2f}m")
            return False
        
        # 速度检查
        if self.uav_state.velocity is not None:
            speed = np.linalg.norm(self.uav_state.velocity)
            if speed > 8.0:  # 速度过快
                print(f"WARNING: 速度过快: {speed:.2f}m/s")
                return False
        
        # 位置边界检查
        pos = self.uav_state.position
        if abs(pos[0]) > 50 or abs(pos[1]) > 50:  # 飞行范围限制
            print(f"WARNING: 超出飞行范围: ({pos[0]:.1f}, {pos[1]:.1f})")
            return False
        
        return True
    
    def _execute_position_control(self):
        """执行位置控制 - 使用原有PID算法"""
        # 设置控制器状态
        self.current_controller.set_current_state(
            self.uav_state.position,
            self.uav_state.velocity,
            self.uav_state.orientation
        )
        
        self.current_controller.set_desired_state(
            self.target_position,
            np.zeros(3),  # 定点悬停，期望速度为0
            np.zeros(3),  # 期望加速度为0
            self.target_yaw
        )
        
        # 计算控制输出 - 保持原有算法不变
        u_att = self.current_controller.update(self.control_frequency)
        
        # 发送控制命令到AirSim
        self._send_attitude_command(u_att)
    
    def _execute_velocity_control(self):
        """执行速度控制"""
        # 直接发送速度命令
        self.client.moveByVelocityAsync(
            self.target_velocity[0],
            -self.target_velocity[1],  # ENU -> NED
            -self.target_velocity[2],
            1.0 / self.control_frequency,
            vehicle_name=self.vehicle_name
        )
    
    def _execute_trajectory_control(self):
        """执行轨迹跟踪控制"""
        if not self.trajectory_points:
            self.set_control_mode(ControlMode.POSITION_HOLD)
            return
            
        # 获取当前轨迹目标点
        target_point = self._get_current_trajectory_target()
        if target_point is None:
            print("轨迹跟踪完成")
            self.set_control_mode(ControlMode.POSITION_HOLD)
            return
            
        # 设置目标状态
        self.target_position = target_point.position
        self.target_yaw = target_point.yaw
        
        # 设置控制器状态
        self.current_controller.set_current_state(
            self.uav_state.position,
            self.uav_state.velocity,
            self.uav_state.orientation
        )
        
        self.current_controller.set_desired_state(
            target_point.position,
            target_point.velocity,
            target_point.acceleration,
            target_point.yaw
        )
        
        # 计算控制输出
        u_att = self.current_controller.update(self.control_frequency)
        
        # 发送控制命令
        self._send_attitude_command(u_att)
    
    def _execute_landing(self):
        """执行降落"""
        self.client.landAsync(vehicle_name=self.vehicle_name)
        
        # 检查是否已着陆
        if self.uav_state.position[2] < 0.3:
            print("降落完成")
            self.set_control_mode(ControlMode.MANUAL)
    
    def _execute_emergency(self):
        """执行紧急停止"""
        print("紧急停止！")
        self.client.hoverAsync(vehicle_name=self.vehicle_name)
    
    def _send_attitude_command(self, u_att):
        """发送姿态控制命令到AirSim"""
        roll, pitch, yaw, throttle = u_att
        
        # 限制控制输出
        roll = np.clip(roll, -0.3, 0.3)  # 限制到17度左右
        pitch = np.clip(pitch, -0.3, 0.3)
        yaw_rate = np.clip(yaw, -0.5, 0.5)
        
        # 使用姿态角控制，这是正确的方法
        # moveByRollPitchYawZAsync: 直接设置姿态角和高度
        self.client.moveByRollPitchYawZAsync(
            roll,           # roll角度
            -pitch,         # pitch角度 (ENU -> NED坐标转换)
            yaw_rate,       # yaw角速率 
            self.target_position[2],  # Z位置(高度)
            1.0 / self.control_frequency,  # 持续时间
            vehicle_name=self.vehicle_name
        )
    
    def _get_current_trajectory_target(self) -> Optional[TrajectoryPoint]:
        """获取当前轨迹目标点"""
        if not self.trajectory_points:
            return None
            
        current_time = time.time() - self.trajectory_start_time
        
        # 找到对应的轨迹点
        while (self.current_trajectory_index < len(self.trajectory_points) and
               self.trajectory_points[self.current_trajectory_index].time_from_start < current_time):
            self.current_trajectory_index += 1
            
        if self.current_trajectory_index >= len(self.trajectory_points):
            return None
            
        return self.trajectory_points[self.current_trajectory_index]
    
    # 公共接口方法
    def takeoff(self, altitude=3.0):
        """改进的渐进式起飞"""
        print(f"起飞到 {altitude} 米高度...")
        
        try:
            # 首先解锁和启动电机
            self.client.enableApiControl(True, self.vehicle_name)
            self.client.armDisarm(True, self.vehicle_name)
            time.sleep(1)
            
            # 获取当前位置
            current_pos = self.client.getMultirotorState(self.vehicle_name).kinematics_estimated.position
            start_x, start_y = current_pos.x_val, current_pos.y_val
            
            # 分阶段缓慢起飞，避免地面效应
            print("第1阶段: 缓慢离地...")
            # 先到1米高度
            self.client.moveToPositionAsync(start_x, start_y, -1.0, 2.0, 
                                          vehicle_name=self.vehicle_name).join()
            time.sleep(2)
            
            print("第2阶段: 到达目标高度...")
            # 再到目标高度
            self.client.moveToPositionAsync(start_x, start_y, -altitude, 3.0,
                                          vehicle_name=self.vehicle_name).join()
            time.sleep(1)
            
            # 设置起飞后的位置控制（注意：AirSim使用NED坐标系，Z轴负值表示高度）
            self.target_position = np.array([0.0, 0.0, -altitude])
            self.target_yaw = 0.0
            self.set_control_mode(ControlMode.POSITION_HOLD)
            
            print("起飞完成，开始位置控制模式")
            
        except Exception as e:
            print(f"起飞失败: {e}")
            self.emergency_stop()
    
    def land(self):
        """降落"""
        print("开始降落...")
        self.set_control_mode(ControlMode.LANDING)
    
    def set_target_position(self, x, y, z, yaw=0.0):
        """设置目标位置"""
        self.target_position = np.array([x, y, z])
        self.target_yaw = yaw
        self.set_control_mode(ControlMode.POSITION_HOLD)
        print(f"目标位置设置为: [{x:.2f}, {y:.2f}, {z:.2f}], yaw: {yaw:.2f}")
    
    def set_target_velocity(self, vx, vy, vz):
        """设置目标速度"""
        self.target_velocity = np.array([vx, vy, vz])
        self.set_control_mode(ControlMode.VELOCITY_CONTROL)
        print(f"目标速度设置为: [{vx:.2f}, {vy:.2f}, {vz:.2f}]")
    
    def set_trajectory(self, trajectory_points: List[TrajectoryPoint]):
        """设置轨迹"""
        self.trajectory_points = trajectory_points
        self.current_trajectory_index = 0
        self.trajectory_start_time = time.time()
        self.set_control_mode(ControlMode.TRAJECTORY_CONTROL)
        print(f"轨迹设置完成，共 {len(trajectory_points)} 个点")
    
    def set_control_mode(self, mode: ControlMode):
        """设置控制模式"""
        if mode != self.control_mode:
            print(f"控制模式: {self.control_mode.value} -> {mode.value}")
            self.control_mode = mode
    
    def set_controller_type(self, controller_type: ControllerType):
        """设置控制器类型"""
        if controller_type != self.controller_type:
            print(f"控制器类型: {self.controller_type.value} -> {controller_type.value}")
            self.controller_type = controller_type
            
            # 切换控制器
            if controller_type == ControllerType.PID:
                self.current_controller = self.pid_controller
            # TODO: 添加其他控制器类型
    
    def get_state(self) -> UAVState:
        """获取当前状态"""
        return self.uav_state
    
    def get_position(self) -> np.ndarray:
        """获取当前位置"""
        return self.uav_state.position.copy()
    
    def get_tracking_error(self) -> float:
        """获取跟踪误差"""
        if self.current_controller:
            return self.current_controller.tracking_error.pos_error_mean
        return 0.0
    
    def emergency_stop(self):
        """紧急停止"""
        self.set_control_mode(ControlMode.EMERGENCY)
    
    def switch_controller(self, controller_type: ControllerType):
        """切换控制器"""
        if controller_type == ControllerType.PID:
            self.current_controller = self.pid_controller
            self.controller_type = ControllerType.PID
        elif controller_type == ControllerType.UDE:
            self.current_controller = self.ude_controller
            self.controller_type = ControllerType.UDE
        elif controller_type == ControllerType.ADRC:
            self.current_controller = self.adrc_controller
            self.controller_type = ControllerType.ADRC
        else:
            print(f"未知控制器类型: {controller_type}")
            return False
            
        print(f"已切换到 {controller_type.value} 控制器")
        return True
    
    def get_current_controller_name(self) -> str:
        """获取当前控制器名称"""
        return self.controller_type.value
    
    def get_control_mode(self) -> ControlMode:
        """获取当前控制模式"""
        return self.control_mode
        
    def print_status(self):
        """打印状态信息"""
        print("=" * 50)
        print("HX AirSim控制器状态")
        print("=" * 50)
        print(f"连接状态: {'连接' if self.uav_state.connected else '断开'}")
        print(f"控制模式: {self.control_mode.value}")
        print(f"控制器类型: {self.controller_type.value}")
        print(f"当前位置: [{self.uav_state.position[0]:.2f}, {self.uav_state.position[1]:.2f}, {self.uav_state.position[2]:.2f}]")
        print(f"当前速度: [{self.uav_state.velocity[0]:.2f}, {self.uav_state.velocity[1]:.2f}, {self.uav_state.velocity[2]:.2f}]")
        print(f"目标位置: [{self.target_position[0]:.2f}, {self.target_position[1]:.2f}, {self.target_position[2]:.2f}]")
        print(f"跟踪误差: {self.get_tracking_error():.3f} m")
        print("=" * 50)