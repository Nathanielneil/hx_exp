#!/usr/bin/env python3
"""
PID位置控制器 - 完全保持原有Prometheus算法不变
PID Position Controller - Keeping original Prometheus algorithm unchanged
"""

import numpy as np
import math
from .base_controller import BaseController, ControllerParams, GeometryUtils

class PIDControllerParams(ControllerParams):
    """PID控制器参数 - 完全对应原始Prometheus参数结构"""
    def __init__(self):
        super().__init__()
        # 控制增益矩阵
        self.Kp = np.zeros((3, 3))  # 位置增益
        self.Kv = np.zeros((3, 3))  # 速度增益  
        self.Kvi = np.zeros((3, 3)) # 积分增益
        self.Ka = np.zeros((3, 3))  # 加速度增益 (预留)
        
        # 积分限幅
        self.int_max = [0.5, 0.5, 0.5]  # xyz积分上限
        
        # 默认参数值 - 保持与原始C++代码一致
        self.quad_mass = 1.0
        self.hov_percent = 0.5
        self.tilt_angle_max = 10.0  # degrees
        
        # 设置默认增益 - 对应原始代码的默认值
        self.Kp[0, 0] = self.Kp[1, 1] = 2.0  # Kp_xy
        self.Kp[2, 2] = 2.0                   # Kp_z
        self.Kv[0, 0] = self.Kv[1, 1] = 2.0  # Kv_xy
        self.Kv[2, 2] = 2.0                   # Kv_z
        self.Kvi[0, 0] = self.Kvi[1, 1] = 0.3 # Kvi_xy
        self.Kvi[2, 2] = 0.3                  # Kvi_z

class PIDController(BaseController):
    """PID位置控制器 - 完全保持原始Prometheus实现不变"""
    
    def __init__(self, params=None):
        super().__init__()
        
        # 使用PID专用参数
        self.ctrl_param = params if params else PIDControllerParams()
        
        # 积分项 - 对应原始代码的int_e_v
        self.int_e_v = np.zeros(3)
        
        # 期望力向量 - 对应原始代码的F_des
        self.F_des = np.zeros(3)
        
        # 飞行模式 (用于积分控制)
        self.flight_mode = "OFFBOARD"  # 对应原始代码的uav_state.mode
        
        self.initialized = True
        
    def init_from_dict(self, param_dict):
        """从参数字典初始化 - 对应原始代码的init函数"""
        # 基础参数
        self.ctrl_param.quad_mass = param_dict.get('quad_mass', 1.0)
        self.ctrl_param.hov_percent = param_dict.get('hov_percent', 0.5)
        self.ctrl_param.tilt_angle_max = param_dict.get('tilt_angle_max', 10.0)
        
        # 积分上限
        pxy_int_max = param_dict.get('pxy_int_max', 0.5)
        pz_int_max = param_dict.get('pz_int_max', 0.5)
        self.ctrl_param.int_max = [pxy_int_max, pxy_int_max, pz_int_max]
        
        # 控制增益 - 完全对应原始代码的参数设置
        Kp_xy = param_dict.get('Kp_xy', 2.0)
        Kp_z = param_dict.get('Kp_z', 2.0)
        Kv_xy = param_dict.get('Kv_xy', 2.0)
        Kv_z = param_dict.get('Kv_z', 2.0)
        Kvi_xy = param_dict.get('Kvi_xy', 0.3)
        Kvi_z = param_dict.get('Kvi_z', 0.3)
        
        self.ctrl_param.Kp[0, 0] = self.ctrl_param.Kp[1, 1] = Kp_xy
        self.ctrl_param.Kp[2, 2] = Kp_z
        self.ctrl_param.Kv[0, 0] = self.ctrl_param.Kv[1, 1] = Kv_xy
        self.ctrl_param.Kv[2, 2] = Kv_z
        self.ctrl_param.Kvi[0, 0] = self.ctrl_param.Kvi[1, 1] = Kvi_xy
        self.ctrl_param.Kvi[2, 2] = Kvi_z
        
        # 重力向量
        self.ctrl_param.g = np.array([0.0, 0.0, 9.8])
        
    def set_flight_mode(self, mode):
        """设置飞行模式"""
        self.flight_mode = mode
        
    def update(self, controller_hz):
        """
        PID控制器更新 - 完全保持原始Prometheus算法不变
        
        输入: 控制器频率
        输出: [roll, pitch, yaw, throttle] - 对应原始代码的u_att
        """
        
        # 定点控制的时候才积分，即追踪轨迹或者速度追踪时不进行积分
        # 完全对应原始代码第103-108行
        if (self.desired_state.vel[0] != 0.0 or 
            self.desired_state.vel[1] != 0.0 or 
            self.desired_state.vel[2] != 0.0):
            # print("Reset integration.")  # 对应原始代码的PCOUT
            self.int_e_v = np.zeros(3)
            
        # 位置误差 - 对应原始代码第111-112行
        pos_error = self.desired_state.pos - self.current_state.pos
        vel_error = self.desired_state.vel - self.current_state.vel
        
        # 跟踪误差统计
        self.tracking_error.input_error(pos_error, vel_error)
        
        # 限制最大误差 - 对应原始代码第117-130行
        max_pos_error = 3.0
        max_vel_error = 3.0
        
        for i in range(3):
            if abs(pos_error[i]) > max_pos_error:
                pos_error[i] = 1.0 if pos_error[i] > 0 else -1.0
            if abs(vel_error[i]) > max_vel_error:
                vel_error[i] = 2.0 if vel_error[i] > 0 else -2.0
                
        # 积分项 - XY - 完全对应原始代码第132-149行
        for i in range(2):
            # 只有在pos_error比较小时，才会启动积分
            int_start_error = 0.2
            if (abs(pos_error[i]) < int_start_error and 
                self.flight_mode == "OFFBOARD"):
                self.int_e_v[i] += pos_error[i] / controller_hz
                if abs(self.int_e_v[i]) > self.ctrl_param.int_max[i]:
                    # print("int_e_v saturation [xy]")  # 对应PCOUT
                    self.int_e_v[i] = (self.ctrl_param.int_max[i] if self.int_e_v[i] > 0 
                                      else -self.ctrl_param.int_max[i])
            else:
                self.int_e_v[i] = 0
                
        # 积分项 - Z - 完全对应原始代码第151-165行
        int_start_error = 0.5
        if (abs(pos_error[2]) < int_start_error and 
            self.flight_mode == "OFFBOARD"):
            self.int_e_v[2] += pos_error[2] / controller_hz
            if abs(self.int_e_v[2]) > self.ctrl_param.int_max[2]:
                # print("int_e_v saturation [z]")  # 对应PCOUT
                self.int_e_v[2] = (self.ctrl_param.int_max[2] if self.int_e_v[2] > 0 
                                  else -self.ctrl_param.int_max[2])
        else:
            self.int_e_v[2] = 0
            
        # 期望加速度 - 对应原始代码第168行
        # 期望加速度 = 期望加速度 + Kp * 位置误差 + Kv * 速度误差 + Kvi * 积分项
        des_acc = (self.desired_state.acc + 
                  self.ctrl_param.Kp @ pos_error + 
                  self.ctrl_param.Kv @ vel_error + 
                  self.ctrl_param.Kvi @ self.int_e_v)
        
        # 期望力 - 对应原始代码第173行
        # F_des是基于模型的位置控制器计算得到的三轴期望推力（惯性系），量纲为牛
        self.F_des = des_acc * self.ctrl_param.quad_mass + self.ctrl_param.quad_mass * self.ctrl_param.g
        
        # 推力限制 - 对应原始代码第175-184行
        # 如果向上推力小于重力的一半或者向上推力大于重力的两倍
        min_thrust = 0.5 * self.ctrl_param.quad_mass * self.ctrl_param.g[2]
        max_thrust = 2.0 * self.ctrl_param.quad_mass * self.ctrl_param.g[2]
        
        if self.F_des[2] < min_thrust:
            self.F_des = self.F_des / self.F_des[2] * min_thrust
        elif self.F_des[2] > max_thrust:
            self.F_des = self.F_des / self.F_des[2] * max_thrust
            
        # 角度限制幅度 - 对应原始代码第187-198行
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
            
        # 坐标系转换 - 对应原始代码第200-205行
        # F_des是位于ENU坐标系的,F_c是FLU
        wRc = GeometryUtils.rotz(self.current_state.yaw)
        F_c = wRc.T @ self.F_des
        fx, fy, fz = F_c[0], F_c[1], F_c[2]
        
        # 期望姿态角计算 - 对应原始代码第207-210行
        u_att = np.zeros(4)
        u_att[0] = math.atan2(-fy, fz)  # roll
        u_att[1] = math.atan2(fx, fz)   # pitch
        u_att[2] = self.desired_state.yaw  # yaw
        
        # 油门计算 - 对应原始代码第212-223行
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
        
        # 油门限制 - 对应原始代码第225-235行
        if u_att[3] < 0.1:
            u_att[3] = 0.1
            # print("throttle too low")  # 对应PCOUT
        
        if u_att[3] > 1.0:
            u_att[3] = 1.0
            # print("throttle too high")  # 对应PCOUT
            
        return u_att
        
    def printf_param(self):
        """打印参数 - 对应原始代码的printf_param函数"""
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<")
        print(f"ctrl_param.quad_mass     : {self.ctrl_param.quad_mass}")
        print(f"ctrl_param.hov_percent   : {self.ctrl_param.hov_percent}")
        print(f"pxy_int_max              : {self.ctrl_param.int_max[0]}")
        print(f"pz_int_max               : {self.ctrl_param.int_max[2]}")
        print(f"Kp_xy         : {self.ctrl_param.Kp[0, 0]}")
        print(f"Kp_z          : {self.ctrl_param.Kp[2, 2]}")
        print(f"Kv_xy         : {self.ctrl_param.Kv[0, 0]}")
        print(f"Kv_z          : {self.ctrl_param.Kv[2, 2]}")
        print(f"Kvi_xy        : {self.ctrl_param.Kvi[0, 0]}")
        print(f"Kvi_z         : {self.ctrl_param.Kvi[2, 2]}")
        print(f"Ka_xy         : {self.ctrl_param.Ka[0, 0]}")
        print(f"Ka_z          : {self.ctrl_param.Ka[2, 2]}")
        print(f"tilt_angle_max: {self.ctrl_param.tilt_angle_max}")
        
    def printf_result(self, u_att):
        """打印结果 - 对应原始代码的printf_result函数"""
        print("----> PID Position Controller Debug Info      : ")
        print(f"----> pos_des         : {self.desired_state.pos[0]:.2f} [m] {self.desired_state.pos[1]:.2f} [m] {self.desired_state.pos[2]:.2f} [m]")
        print(f"----> vel_des         : {self.desired_state.vel[0]:.2f} [m/s] {self.desired_state.vel[1]:.2f} [m/s] {self.desired_state.vel[2]:.2f} [m/s]")
        print(f"----> acc_des         : {self.desired_state.acc[0]:.2f} [m/s²] {self.desired_state.acc[1]:.2f} [m/s²] {self.desired_state.acc[2]:.2f} [m/s²]")
        print(f"----> pos_now         : {self.current_state.pos[0]:.2f} [m] {self.current_state.pos[1]:.2f} [m] {self.current_state.pos[2]:.2f} [m]")
        print(f"----> vel_now         : {self.current_state.vel[0]:.2f} [m/s] {self.current_state.vel[1]:.2f} [m/s] {self.current_state.vel[2]:.2f} [m/s]")
        print(f"----> int_e_v         : {self.int_e_v[0]:.2f} [N] {self.int_e_v[1]:.2f} [N] {self.int_e_v[2]:.2f} [N]")
        print(f"----> F_des           : {self.F_des[0]:.2f} [N] {self.F_des[1]:.2f} [N] {self.F_des[2]:.2f} [N]")
        print(f"----> u_attitude      : {u_att[0]*180/math.pi:.2f} [deg] {u_att[1]*180/math.pi:.2f} [deg] {u_att[2]*180/math.pi:.2f} [deg]")
        print(f"----> u_throttle      : {u_att[3]:.3f} [0-1]")
        print(f"----> pos_error_mean  : {self.tracking_error.pos_error_mean:.3f} [m]")
        print(f"----> vel_error_mean  : {self.tracking_error.vel_error_mean:.3f} [m/s]")