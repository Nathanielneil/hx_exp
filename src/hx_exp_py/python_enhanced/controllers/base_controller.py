#!/usr/bin/env python3
"""
基础控制器类 - 保持原有Prometheus控制算法不变
Base Controller Class - Keeping original Prometheus control algorithms unchanged
"""

import numpy as np
import math
from abc import ABC, abstractmethod
from typing import NamedTuple, Optional
from dataclasses import dataclass

@dataclass
class ControllerParams:
    """控制器参数基类"""
    quad_mass: float = 1.0
    hov_percent: float = 0.5
    tilt_angle_max: float = 10.0  # degrees
    g: np.ndarray = None
    
    def __post_init__(self):
        if self.g is None:
            self.g = np.array([0.0, 0.0, 9.8])

@dataclass 
class DesiredState:
    """期望状态 - 完全对应原始Prometheus结构"""
    pos: np.ndarray = None
    vel: np.ndarray = None
    acc: np.ndarray = None
    yaw: float = 0.0
    
    def __post_init__(self):
        if self.pos is None:
            self.pos = np.zeros(3)
        if self.vel is None:
            self.vel = np.zeros(3)
        if self.acc is None:
            self.acc = np.zeros(3)

@dataclass
class CurrentState:
    """当前状态 - 完全对应原始Prometheus结构"""
    pos: np.ndarray = None
    vel: np.ndarray = None
    q: np.ndarray = None  # quaternion [w, x, y, z]
    yaw: float = 0.0
    
    def __post_init__(self):
        if self.pos is None:
            self.pos = np.zeros(3)
        if self.vel is None:
            self.vel = np.zeros(3)
        if self.q is None:
            self.q = np.array([1.0, 0.0, 0.0, 0.0])  # identity quaternion

class TrackingErrorEvaluation:
    """跟踪误差评估 - 保持原始逻辑不变"""
    def __init__(self):
        self.pos_error_mean = 0.0
        self.vel_error_mean = 0.0
        self.error_history = []
        self.max_history = 100
        
    def input_error(self, pos_error, vel_error):
        """输入误差进行统计"""
        pos_norm = np.linalg.norm(pos_error)
        vel_norm = np.linalg.norm(vel_error)
        
        self.error_history.append((pos_norm, vel_norm))
        
        if len(self.error_history) > self.max_history:
            self.error_history.pop(0)
            
        # 计算均值
        if self.error_history:
            pos_errors = [err[0] for err in self.error_history]
            vel_errors = [err[1] for err in self.error_history]
            self.pos_error_mean = np.mean(pos_errors)
            self.vel_error_mean = np.mean(vel_errors)

class GeometryUtils:
    """几何工具类 - 保持原始实现不变"""
    
    @staticmethod
    def get_yaw_from_quaternion(q):
        """从四元数获取yaw角 - 保持原始算法不变"""
        # q = [w, x, y, z]
        w, x, y, z = q[0], q[1], q[2], q[3]
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """四元数转旋转矩阵 - 保持原始算法不变"""
        # q = [w, x, y, z]
        w, x, y, z = q[0], q[1], q[2], q[3]
        
        # 归一化
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm == 0:
            return np.eye(3)
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        # 旋转矩阵
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])
        
        return R
    
    @staticmethod
    def rotz(yaw):
        """绕Z轴旋转矩阵 - 保持原始实现不变"""
        c = math.cos(yaw)
        s = math.sin(yaw)
        
        R = np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])
        
        return R
    
    @staticmethod
    def to_rad(degrees):
        """角度转弧度"""
        return degrees * math.pi / 180.0

class BaseController(ABC):
    """控制器基类 - 抽象接口"""
    
    def __init__(self):
        self.ctrl_param = ControllerParams()
        self.desired_state = DesiredState()
        self.current_state = CurrentState()
        self.tracking_error = TrackingErrorEvaluation()
        self.initialized = False
        
    def set_desired_state(self, pos, vel=None, acc=None, yaw=0.0):
        """设置期望状态"""
        self.desired_state.pos = np.array(pos) if pos is not None else np.zeros(3)
        self.desired_state.vel = np.array(vel) if vel is not None else np.zeros(3)
        self.desired_state.acc = np.array(acc) if acc is not None else np.zeros(3)
        self.desired_state.yaw = yaw
        
    def set_current_state(self, pos, vel, quaternion):
        """设置当前状态"""
        self.current_state.pos = np.array(pos)
        self.current_state.vel = np.array(vel)
        self.current_state.q = np.array(quaternion)  # [w, x, y, z]
        self.current_state.yaw = GeometryUtils.get_yaw_from_quaternion(self.current_state.q)
        
    @abstractmethod
    def update(self, controller_hz):
        """
        更新控制器
        输入: 控制器频率
        输出: [roll, pitch, yaw, throttle]
        """
        pass
        
    def printf_param(self):
        """打印参数 - 可被子类重写"""
        print(f"Controller Parameters:")
        print(f"quad_mass: {self.ctrl_param.quad_mass}")
        print(f"hov_percent: {self.ctrl_param.hov_percent}")
        print(f"tilt_angle_max: {self.ctrl_param.tilt_angle_max}")
        
    def printf_result(self, u_att):
        """打印结果 - 可被子类重写"""
        print(f"Controller Output:")
        print(f"Roll: {u_att[0] * 180 / math.pi:.2f} deg")
        print(f"Pitch: {u_att[1] * 180 / math.pi:.2f} deg") 
        print(f"Yaw: {u_att[2] * 180 / math.pi:.2f} deg")
        print(f"Throttle: {u_att[3]:.3f}")
        print(f"Pos error mean: {self.tracking_error.pos_error_mean:.3f} m")
        print(f"Vel error mean: {self.tracking_error.vel_error_mean:.3f} m/s")