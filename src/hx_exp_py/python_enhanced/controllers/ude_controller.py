#!/usr/bin/env python3
"""
UDE位置控制器 - 完全保持原有Prometheus算法不变
UDE Position Controller - Keeping original Prometheus algorithm unchanged

UDE = Uncertainty and Disturbance Estimator
不确定性和干扰估计器
"""

import numpy as np
import math
from .base_controller import BaseController, ControllerParams, GeometryUtils

class UDEControllerParams(ControllerParams):
    """UDE控制器参数 - 完全对应原始Prometheus参数结构"""
    def __init__(self):
        super().__init__()
        # 控制增益矩阵
        self.Kp = np.zeros((3, 3))  # 位置增益
        self.Kd = np.zeros((3, 3))  # 速度增益 (PD控制)
        
        # UDE特有参数
        self.T_ude = 1.0  # UDE时间常数
        
        # 积分限幅 (用于u_d限制)
        self.int_max = [1.0, 1.0, 1.0]  # xyz u_d上限
        
        # 默认参数值 - 保持与原始C++代码一致
        self.quad_mass = 1.0
        self.hov_percent = 0.5
        self.tilt_angle_max = 20.0  # degrees
        
        # 设置默认增益 - 对应原始代码的默认值
        self.Kp[0, 0] = self.Kp[1, 1] = 0.5  # Kp_xy
        self.Kp[2, 2] = 0.5                   # Kp_z
        self.Kd[0, 0] = self.Kd[1, 1] = 2.0  # Kd_xy
        self.Kd[2, 2] = 2.0                   # Kd_z

class UDEController(BaseController):
    """UDE位置控制器 - 完全保持原始Prometheus实现不变"""
    
    def __init__(self, params=None):
        super().__init__()
        
        # 使用UDE专用参数
        self.ctrl_param = params if params else UDEControllerParams()
        
        # UDE控制器特有变量 - 对应原始代码
        self.u_l = np.zeros(3)        # nominal control (PD) - 标称控制
        self.u_d = np.zeros(3)        # UDE control (disturbance estimator) - 干扰估计
        self.integral = np.zeros(3)   # 积分项 - 用于UDE算法
        
        # 期望力向量 - 对应原始代码的F_des
        self.F_des = np.zeros(3)
        
        self.initialized = True
        
    def init_from_dict(self, param_dict):
        """从参数字典初始化 - 对应原始代码的init函数"""
        # 基础参数
        self.ctrl_param.quad_mass = param_dict.get('quad_mass', 1.0)
        self.ctrl_param.hov_percent = param_dict.get('hov_percent', 0.5)
        self.ctrl_param.tilt_angle_max = param_dict.get('tilt_angle_max', 20.0)
        
        # UDE特有参数
        self.ctrl_param.T_ude = param_dict.get('T_ude', 1.0)
        
        # 积分上限
        pxy_int_max = param_dict.get('pxy_int_max', 1.0)
        pz_int_max = param_dict.get('pz_int_max', 1.0)
        self.ctrl_param.int_max = [pxy_int_max, pxy_int_max, pz_int_max]
        
        # 控制增益 - 完全对应原始代码的参数设置
        Kp_xy = param_dict.get('Kp_xy', 0.5)
        Kp_z = param_dict.get('Kp_z', 0.5)
        Kd_xy = param_dict.get('Kd_xy', 2.0)
        Kd_z = param_dict.get('Kd_z', 2.0)
        
        self.ctrl_param.Kp[0, 0] = self.ctrl_param.Kp[1, 1] = Kp_xy
        self.ctrl_param.Kp[2, 2] = Kp_z
        self.ctrl_param.Kd[0, 0] = self.ctrl_param.Kd[1, 1] = Kd_xy
        self.ctrl_param.Kd[2, 2] = Kd_z
        
        # 重力向量
        self.ctrl_param.g = np.array([0.0, 0.0, 9.8])
        
        # 初始化控制变量
        self.u_l = np.zeros(3)
        self.u_d = np.zeros(3)
        self.integral = np.zeros(3)
    
    def update(self, controller_hz):
        """
        UDE控制器更新 - 完全保持原始Prometheus算法不变
        
        输入: 控制器频率
        输出: [roll, pitch, yaw, throttle] - 对应原始代码的u_att
        """
        
        dt = 1.0 / controller_hz  # 对应原始代码第97行
        
        # 位置误差 - 对应原始代码第99行
        pos_error = self.desired_state.pos - self.current_state.pos
        # 速度误差 - 对应原始代码第101行
        vel_error = self.desired_state.vel - self.current_state.vel
        # 误差评估 - 对应原始代码第103行
        self.tracking_error.input_error(pos_error, vel_error)
        
        # 限制最大误差 - 对应原始代码第105-121行
        max_pos_error = 3.0
        max_vel_error = 3.0
        
        for i in range(3):
            if abs(pos_error[i]) > max_pos_error:
                pos_error[i] = max_pos_error if pos_error[i] > 0 else -max_pos_error
                
            if abs(vel_error[i]) > max_vel_error:
                vel_error[i] = max_vel_error if vel_error[i] > 0 else -max_vel_error
        
        # UDE算法 - 对应原始代码第123-125行
        # u_l为标称控制(PD)，u_d为UDE控制(干扰估计器)
        self.u_l = (self.desired_state.acc + 
                   self.ctrl_param.Kp @ pos_error + 
                   self.ctrl_param.Kd @ vel_error)
        
        self.u_d = (-1.0 / self.ctrl_param.T_ude * 
                   (self.ctrl_param.Kp @ self.integral + 
                    self.ctrl_param.Kd @ pos_error + vel_error))
        
        # 更新积分项 - 对应原始代码第127-144行
        for i in range(3):
            int_start_error = 0.5
            if abs(pos_error[i]) < int_start_error:
                self.integral[i] += pos_error[i] * dt
            else:
                self.integral[i] = 0
                
            # u_d饱和限制
            if abs(self.u_d[i]) > self.ctrl_param.int_max[i]:
                # print("u_d saturation!")  # 对应原始代码PCOUT
                self.u_d[i] = (self.ctrl_param.int_max[i] if self.u_d[i] > 0 
                              else -self.ctrl_param.int_max[i])
        
        # 期望加速度 - 对应原始代码第147行
        u_v = self.u_l - self.u_d
        
        # 期望力 - 对应原始代码第149-150行
        # 期望力 = 质量*控制量 + 重力抵消
        self.F_des = u_v * self.ctrl_param.quad_mass + self.ctrl_param.quad_mass * self.ctrl_param.g
        
        # 推力限制 - 对应原始代码第152-161行
        # 如果向上推力小于重力的一半或者向上推力大于重力的两倍
        min_thrust = 0.5 * self.ctrl_param.quad_mass * self.ctrl_param.g[2]
        max_thrust = 2.0 * self.ctrl_param.quad_mass * self.ctrl_param.g[2]
        
        if self.F_des[2] < min_thrust:
            self.F_des = self.F_des / self.F_des[2] * min_thrust
        elif self.F_des[2] > max_thrust:
            self.F_des = self.F_des / self.F_des[2] * max_thrust
            
        # 角度限制幅度 - 对应原始代码第163-175行
        tilt_angle_max_rad = GeometryUtils.to_rad(self.ctrl_param.tilt_angle_max)
        
        # Pitch角度限制
        if abs(self.F_des[0] / self.F_des[2]) > math.tan(tilt_angle_max_rad):
            # print("pitch too tilt")  # 对应PCOUT
            self.F_des[0] = (math.copysign(1, self.F_des[0]) * 
                           self.F_des[2] * math.tan(tilt_angle_max_rad))
            
        # Roll角度限制  
        if abs(self.F_des[1] / self.F_des[2]) > math.tan(tilt_angle_max_rad):
            # print("roll too tilt")  # 对应PCOUT
            self.F_des[1] = (math.copysign(1, self.F_des[1]) * 
                           self.F_des[2] * math.tan(tilt_angle_max_rad))
            
        # 坐标系转换 - 对应原始代码第177-182行
        # F_des是位于ENU坐标系的,F_c是FLU
        wRc = GeometryUtils.rotz(self.current_state.yaw)
        F_c = wRc.T @ self.F_des
        fx, fy, fz = F_c[0], F_c[1], F_c[2]
        
        # 期望姿态角计算 - 对应原始代码第184-187行
        u_att = np.zeros(4)
        u_att[0] = math.atan2(-fy, fz)  # roll
        u_att[1] = math.atan2(fx, fz)   # pitch
        u_att[2] = self.desired_state.yaw  # yaw
        
        # 油门计算 - 对应原始代码第189-200行
        # 无人机姿态的矩阵形式
        wRb_odom = GeometryUtils.quaternion_to_rotation_matrix(self.current_state.q)
        # 第三列
        z_b_curr = wRb_odom[:, 2]
        # 机体系下的电机推力 相当于Rb * F_enu 惯性系到机体系
        u1 = np.dot(self.F_des, z_b_curr)
        # 悬停油门与电机参数有关系,也取决于质量
        full_thrust = self.ctrl_param.quad_mass * self.ctrl_param.g[2] / self.ctrl_param.hov_percent
        
        # 油门 = 期望推力/最大推力
        # 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
        u_att[3] = u1 / full_thrust
        
        # 油门限制 - 对应原始代码第202-212行
        if u_att[3] < 0.1:
            u_att[3] = 0.1
            # print("throttle too low")  # 对应PCOUT
        
        if u_att[3] > 1.0:
            u_att[3] = 1.0
            # print("throttle too high")  # 对应PCOUT
            
        return u_att
        
    def printf_param(self):
        """打印参数 - 对应原始代码的printf_param函数"""
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>UDE Parameter <<<<<<<<<<<<<<<<<<<<<<<<<")
        print(f"ctrl_param.quad_mass     : {self.ctrl_param.quad_mass}")
        print(f"ctrl_param.hov_percent   : {self.ctrl_param.hov_percent}")
        print(f"pxy_int_max              : {self.ctrl_param.int_max[0]}")
        print(f"pz_int_max               : {self.ctrl_param.int_max[2]}")
        print(f"ude_gain/Kp_xy           : {self.ctrl_param.Kp[0, 0]}")
        print(f"ude_gain/Kp_z            : {self.ctrl_param.Kp[2, 2]}")
        print(f"ude_gain/Kd_xy           : {self.ctrl_param.Kd[0, 0]}")
        print(f"ude_gain/Kd_z            : {self.ctrl_param.Kd[2, 2]}")
        print(f"ude_gain/T_ude           : {self.ctrl_param.T_ude}")
        print(f"ude_gain/tilt_angle_max  : {self.ctrl_param.tilt_angle_max}")
        
    def printf_result(self, u_att):
        """打印结果 - 对应原始代码的printf_result函数"""
        print("----> UDE Position Controller Debug Info      : ")
        print(f"----> pos_des         : {self.desired_state.pos[0]:.2f} [m] {self.desired_state.pos[1]:.2f} [m] {self.desired_state.pos[2]:.2f} [m]")
        print(f"----> vel_des         : {self.desired_state.vel[0]:.2f} [m/s] {self.desired_state.vel[1]:.2f} [m/s] {self.desired_state.vel[2]:.2f} [m/s]")
        print(f"----> acc_des         : {self.desired_state.acc[0]:.2f} [m/s²] {self.desired_state.acc[1]:.2f} [m/s²] {self.desired_state.acc[2]:.2f} [m/s²]")
        print(f"----> pos_now         : {self.current_state.pos[0]:.2f} [m] {self.current_state.pos[1]:.2f} [m] {self.current_state.pos[2]:.2f} [m]")
        print(f"----> vel_now         : {self.current_state.vel[0]:.2f} [m/s] {self.current_state.vel[1]:.2f} [m/s] {self.current_state.vel[2]:.2f} [m/s]")
        print(f"----> u_l [X Y Z]     : {self.u_l[0]:.2f} [N] {self.u_l[1]:.2f} [N] {self.u_l[2]:.2f} [N]")
        print(f"----> int [X Y Z]     : {self.integral[0]:.2f} [N] {self.integral[1]:.2f} [N] {self.integral[2]:.2f} [N]")
        print(f"----> u_d [X Y Z]     : {self.u_d[0]:.2f} [N] {self.u_d[1]:.2f} [N] {self.u_d[2]:.2f} [N]")
        print(f"----> F_des [X Y Z]   : {self.F_des[0]:.2f} [N] {self.F_des[1]:.2f} [N] {self.F_des[2]:.2f} [N]")
        print(f"----> u_att [X Y Z]   : {u_att[0]:.2f} [rad] {u_att[1]:.2f} [rad] {u_att[2]:.2f} [rad]")
        print(f"----> u_throttle      : {u_att[3]:.3f}")
        print(f"----> pos_error_mean  : {self.tracking_error.pos_error_mean:.3f} [m]")
        print(f"----> vel_error_mean  : {self.tracking_error.vel_error_mean:.3f} [m/s]")