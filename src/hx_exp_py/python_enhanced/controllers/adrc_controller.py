#!/usr/bin/env python3
"""
ADRC位置控制器 - 完全保持原有Prometheus算法不变
ADRC Position Controller - Keeping original Prometheus algorithm unchanged

ADRC = Active Disturbance Rejection Control
自抗扰控制器
"""

import numpy as np
import math
from .base_controller import BaseController, ControllerParams, GeometryUtils

class ESO_Param:
    """ESO参数结构 - 对应原始Prometheus的ESO_Param结构"""
    def __init__(self):
        self.z1 = 0.0  # ESO状态1
        self.z2 = 0.0  # ESO状态2
        self.z3 = 0.0  # ESO状态3

class ADRCControllerParams(ControllerParams):
    """ADRC控制器参数 - 完全对应原始Prometheus参数结构"""
    def __init__(self):
        super().__init__()
        
        # 基础参数
        self.quad_mass = 1.0
        self.hov_percent = 0.5
        self.g = 9.8
        
        # 积分限幅
        self.int_max = [0.5, 0.5, 0.5]  # xyz积分上限
        
        # ADRC特有参数
        self.beta_max = 0.5
        self.C = [0.5, 0.5]  # C1, C2参数
        self.sigma_D = 0.5
        self.amesogain_l = 0.5  # ESO增益
        
class ADRCController(BaseController):
    """ADRC位置控制器 - 完全保持原始Prometheus实现不变"""
    
    def __init__(self, params=None):
        super().__init__()
        
        # 使用ADRC专用参数
        self.ctrl_param = params if params else ADRCControllerParams()
        
        # ADRC控制器特有变量 - 对应原始代码
        self.int_e_v = np.zeros(3)      # 积分项
        
        # 控制变量 - 对应原始代码
        self.u = 0.0                    # 总升力
        self.un = 0.0                   # 标称系统输入
        self.uf = 0.0                   # 反馈控制
        self.uc = 0.0                   # 补偿控制
        self.uo = 0.0                   # 第一次拨杆输入
        self.hatd = 0.0                 # 观测器输出
        self.hx = 0.0                   # 自适应基函数变量
        
        # ESO相关变量
        self.dhat = np.zeros(3)         # 观测器状态
        
        # 期望力向量
        self.F_des = np.zeros(3)        # 期望力
        self.u_att = np.zeros(4)        # 期望姿态+油门
        
        # 滑模相关变量
        self.s = 0.0                    # 滑模状态
        self.z = np.zeros(2)            # 滑模辅助状态
        self.xbeta = np.zeros(2)        # 滑模辅助状态
        self.TD = np.zeros(2)           # 跟踪微分器
        
        # UDE相关变量(在ADRC中有备用实现)
        self.u_l = np.zeros(3)          # UDE标称控制
        self.u_d = np.zeros(3)          # UDE干扰估计
        self.integral = np.zeros(3)     # UDE积分项
        
        # 自适应参数
        self.para_w = [0.0] * 9         # 自适应权重参数
        
        # ESO参数
        self.ESO_x = ESO_Param()
        self.ESO_y = ESO_Param()  
        self.ESO_z = ESO_Param()
        
        # 初始化标志和方法选择
        self.con_init = False
        self.method_choose = 1          # 油门计算方法选择
        self.flag = 0
        
        # 虚拟位置
        self.xi_v = np.zeros(3)
        self.u_v = np.zeros(3)          # 位置控制器输出
        
        # 归一化参数
        self.u_att_min = 0.0
        self.u_att_max = 1.0
        self.Thrust_des_min = 0.0
        self.Thrust_des_max = 2.5 * self.ctrl_param.quad_mass * self.ctrl_param.g
        self.Thr_alpha = 0.5
        self.Thr_k1 = 1.0
        self.Thr_k2 = 0.0
        self.Thr_k3 = 1.0
        
        # 标称系统状态初始化变量
        self.ezn = 0.0
        self.ezn2 = 0.0
        self.ezn0 = 0.0
        self.ezn20 = 0.0
        
        self.initialized = True
        
    def init_from_dict(self, param_dict):
        """从参数字典初始化 - 对应原始代码的init函数"""
        # 基础参数
        self.ctrl_param.quad_mass = param_dict.get('quad_mass', 1.0)
        self.ctrl_param.hov_percent = param_dict.get('hov_percent', 0.5)
        self.ctrl_param.g = param_dict.get('g', 9.8)
        
        # 积分上限
        pxy_int_max = param_dict.get('pxy_int_max', 0.5)
        pz_int_max = param_dict.get('pz_int_max', 0.5)
        self.ctrl_param.int_max = [pxy_int_max, pxy_int_max, pz_int_max]
        
        # ADRC特有参数
        self.ctrl_param.beta_max = param_dict.get('beta_max', 0.5)
        self.ctrl_param.C[0] = param_dict.get('C1', 0.5)
        self.ctrl_param.C[1] = param_dict.get('C2', 0.5)
        self.ctrl_param.sigma_D = param_dict.get('sigmaD', 0.5)
        self.ctrl_param.amesogain_l = param_dict.get('amesogain_l', 0.5)
        
        # 油门计算方法
        self.method_choose = param_dict.get('method_choose', 1)
        
        # 重新计算归一化参数
        self.Thrust_des_max = 2.5 * self.ctrl_param.quad_mass * self.ctrl_param.g
        
        # 初始化所有控制变量
        self.u = 0.0
        self.un = 0.0
        self.uc = 0.0
        self.uf = 0.0
        self.uo = 0.0
        self.hatd = 0.0
        self.hx = 0.0
        self.F_des = np.zeros(3)
        self.u_att = np.zeros(4)
        self.dhat = np.zeros(3)
        self.xi_v = np.zeros(3)
        self.u_l = np.zeros(3)
        self.u_d = np.zeros(3)
        self.s = 0.0
        self.z = np.zeros(2)
        self.integral = np.zeros(3)
        self.xbeta = np.zeros(2)
        self.TD = np.zeros(2)
        self.u_v = np.zeros(3)
        self.con_init = False
        self.para_w = [0.0] * 9
        self.flag = 0
    
    def update(self, controller_hz):
        """
        ADRC控制器更新 - 完全保持原始Prometheus算法不变
        
        输入: 控制器频率
        输出: [roll, pitch, yaw, throttle] - 对应原始代码的u_att
        """
        
        T = 1.0 / controller_hz  # 对应原始代码第197行，采样周期
        
        # 定义位置误差和速度误差 - 对应原始代码第200-201行
        pos_error = self.current_state.pos - self.desired_state.pos
        vel_error = self.current_state.vel - self.desired_state.vel
        
        # 误差评估 - 对应原始代码第203行
        self.tracking_error.input_error(pos_error, vel_error)
        
        # z方向的跟踪误差和速度误差 - 对应原始代码第205-206行
        ez = pos_error[2]    # 仿真中的xi
        ez2 = vel_error[2]
        
        # 设置初值 - 对应原始代码第208-222行
        if not self.con_init:
            self.ezn = ez        # nominal系统状态，仿真中的xin
            self.ezn2 = ez2
            self.ezn0 = ez       # nominal系统状态初值
            self.ezn20 = ez2
            self.con_init = True
        
        # 定义滑模面指数变量 - 对应原始代码第227-228行
        beta1 = 1 + min(self.ctrl_param.beta_max, pow(abs(self.ezn), 0.2)) * (1 if abs(self.ezn) - 1 > 0 else -1)
        beta2 = 1 + min(self.ctrl_param.beta_max, pow(abs(self.ezn2), 0.2)) * (1 if abs(self.ezn2) - 1 > 0 else -1)
        
        # 对应原始代码第230-231行
        self.xbeta[0] = pow(abs(self.ezn), beta1) * (1 if self.ezn > 0 else -1)
        self.xbeta[1] = pow(abs(self.ezn2), beta2) * (1 if self.ezn2 > 0 else -1)
        
        # 滑模面 - 对应原始代码第233行
        self.s = (self.ctrl_param.C[0] * self.ezn + self.ctrl_param.C[1] * self.ezn2 + 
                 self.ctrl_param.C[0] * self.z[0] + self.ctrl_param.C[1] * self.z[1] - 
                 self.ctrl_param.C[0] * self.ezn0 - self.ctrl_param.C[1] * self.ezn20)
        
        # 标称控制 - 对应原始代码第235-236行
        # 这是一个复杂的控制律，完全保持原始算法
        denominator = self.ctrl_param.C[1] * T / self.ctrl_param.quad_mass
        if abs(denominator) > 1e-6:  # 避免除零
            self.un = pow(denominator, -1) * (
                -0.8 * self.s - 
                self.ctrl_param.C[0] * (self.ezn + T * self.ezn2) - 
                self.ctrl_param.C[1] * self.ezn2 + 
                self.ctrl_param.C[1] * T * self.ctrl_param.g + 
                self.ctrl_param.C[1] * T * self.desired_state.acc[2] - 
                self.ctrl_param.C[0] * self.z[0] - 
                self.ctrl_param.C[1] * self.z[1] - 
                self.ctrl_param.C[0] * T * self.xbeta[0] - 
                self.ctrl_param.C[1] * T * self.xbeta[1] + 
                self.ctrl_param.C[0] * self.ezn0 + 
                self.ctrl_param.C[1] * self.ezn20
            )
        else:
            self.un = 0.0
        
        # 标称系统迭代 - 对应原始代码第239-240行
        self.ezn = self.ezn + T * self.ezn2
        self.ezn2 = (self.ezn2 + T / self.ctrl_param.quad_mass * self.un - 
                     T * self.ctrl_param.g - T * self.desired_state.acc[2])
        
        # 反馈控制 - 对应原始代码第243行
        self.uf = -self.ctrl_param.quad_mass * (0.15 * (ez - self.ezn) + 3 * (self.TD[1] - self.ezn2))
        
        # 自适应基函数变量 - 对应原始代码第245行
        self.hx = self.TD[1] + self.desired_state.vel[2]
        
        # 基函数 - 对应原始代码第247行
        phi = [
            1,
            math.sin(self.current_state.pos[2]),
            math.sin(self.hx),
            math.cos(self.current_state.pos[2]),
            math.cos(self.hx),
            math.sin(2 * self.current_state.pos[2]),
            math.sin(2 * self.hx),
            math.cos(2 * self.current_state.pos[2]),
            math.cos(2 * self.hx)
        ]
        
        # 自适应项计算 - 对应原始代码第249-255行
        apd = 0.0
        for j in range(9):
            apd += self.para_w[j] * phi[j]
        
        # 干扰估计 - 对应原始代码第258行
        self.hatd = self.dhat[2] + apd
        
        # 补偿控制 - 对应原始代码第260行
        self.uc = -self.ctrl_param.quad_mass * self.hatd
        
        # 总控制量 - 对应原始代码第266行
        self.u = self.un + self.uc + self.uf
        
        # ESO扩张状态观测器 - 对应原始代码第275-278行
        ee = ez - self.dhat[0]
        self.dhat[0] += T * (self.dhat[1] + 3 * self.ctrl_param.amesogain_l * ee)
        self.dhat[1] += T * (-self.ctrl_param.g + 1 / self.ctrl_param.quad_mass * self.u - 
                            self.desired_state.acc[2] + self.hatd + 
                            3 * self.ctrl_param.amesogain_l * self.ctrl_param.amesogain_l * ee)
        self.dhat[2] += T * (self.ctrl_param.amesogain_l * self.ctrl_param.amesogain_l * 
                            self.ctrl_param.amesogain_l * ee)
        
        # 自适应模型 - 对应原始代码第280-285行
        en = self.TD[1] - self.ezn2
        for j in range(9):
            self.para_w[j] += T * 0.8 * (en * phi[j] - 0.9 * abs(en) * self.para_w[j])
        
        # 跟踪微分器TD - 对应原始代码第287-288行
        self.TD[0] += T * self.TD[1]
        denominator = T * 2 * T
        if abs(denominator) > 1e-6:  # 避免除零
            self.TD[1] -= T * (1 / denominator * (self.TD[0] - ez) + 
                              (T + 2 * T) / denominator * self.TD[1])
        
        # 滑模辅助状态更新 - 对应原始代码第291行
        self.z += T * self.xbeta
        
        # 期望推力 - 对应原始代码第304行
        Thrust_des = self.u
        
        # 姿态角计算 - 对应原始代码第306-308行
        self.u_att[2] = self.desired_state.yaw
        
        # 防止除零保护
        thrust_mass_ratio = Thrust_des / self.ctrl_param.quad_mass
        if abs(thrust_mass_ratio) > 1e-6:
            # 计算roll和pitch角度
            numerator_roll = (math.sin(self.u_att[2]) * self.u_v[0] - 
                             math.cos(self.u_att[2]) * self.u_v[1])
            self.u_att[0] = math.asin(np.clip(numerator_roll / thrust_mass_ratio, -1.0, 1.0))
            
            denominator_pitch = self.u_v[2] + self.ctrl_param.g
            if abs(denominator_pitch) > 1e-6:
                numerator_pitch = (math.cos(self.u_att[2]) * self.u_v[0] + 
                                  math.sin(self.u_att[2]) * self.u_v[1])
                self.u_att[1] = math.atan(numerator_pitch / denominator_pitch)
            else:
                self.u_att[1] = 0.0
        else:
            self.u_att[0] = 0.0
            self.u_att[1] = 0.0
        
        # ESO状态更新 - 对应原始代码第412-422行
        self.ESO_x.z1 = 0
        self.ESO_x.z2 = 0
        self.ESO_x.z3 = 0
        
        self.ESO_y.z1 = 0
        self.ESO_y.z2 = 0
        self.ESO_y.z3 = 0
        
        self.ESO_z.z1 = self.dhat[0]
        self.ESO_z.z2 = self.dhat[1]
        self.ESO_z.z3 = self.dhat[2]
        
        # 油门计算 - 对应原始代码第435-480行
        # 根据method_choose选择不同的计算方法
        if self.method_choose == 1:
            # 法一：过原点与点(hov_percent, mg)的线性关系
            full_thrust = self.ctrl_param.quad_mass * self.ctrl_param.g / self.ctrl_param.hov_percent
            self.u_att[3] = Thrust_des / full_thrust
        elif self.method_choose == 2:
            # 法二：线性关系
            Thr_x1 = self.ctrl_param.hov_percent
            Thr_y1 = self.ctrl_param.quad_mass * self.ctrl_param.g
            Thr_x2 = 1.0
            Thr_y2 = 2.5 * self.ctrl_param.quad_mass * self.ctrl_param.g
            
            Thr_k = (Thr_y2 - Thr_y1) / (Thr_x2 - Thr_x1)
            Thr_b = Thr_y2 - Thr_k * Thr_x2
            self.u_att[3] = (Thrust_des - Thr_b) / Thr_k
        elif self.method_choose == 3:
            # 法三：二次函数关系
            Thrust_des_nor = ((Thrust_des - self.Thrust_des_min) / 
                             (self.Thrust_des_max - self.Thrust_des_min))
            if abs(2.0 * self.Thr_alpha) > 1e-6:
                u_att_nor = (-(1.0 - self.Thr_alpha) + 
                           math.sqrt((1.0 - self.Thr_alpha)**2 + 4.0 * self.Thr_alpha * Thrust_des_nor)) / (2.0 * self.Thr_alpha)
                self.u_att[3] = u_att_nor * (self.u_att_max - self.u_att_min) + self.u_att_min
            else:
                self.u_att[3] = self.ctrl_param.hov_percent
        elif self.method_choose == 4:
            # 法四：二次函数关系
            Thrust_des_nor = ((Thrust_des - self.Thrust_des_min) / 
                             (self.Thrust_des_max - self.Thrust_des_min))
            if abs(2.0 * self.Thr_k1) > 1e-6:
                u_att_nor = ((-self.Thr_k2 + 
                            math.sqrt(self.Thr_k2**2 + 4.0 * self.Thr_k1 * Thrust_des_nor / self.Thr_k3)) / 
                           (2.0 * self.Thr_k1))
                self.u_att[3] = u_att_nor * (self.u_att_max - self.u_att_min) + self.u_att_min
            else:
                self.u_att[3] = self.ctrl_param.hov_percent
        else:
            # 默认方法
            full_thrust = self.ctrl_param.quad_mass * self.ctrl_param.g / self.ctrl_param.hov_percent
            self.u_att[3] = Thrust_des / full_thrust
        
        # 油门限制 - 对应原始代码第482-492行
        if self.u_att[3] < 0.3:
            self.u_att[3] = 0.3
            # print("throttle too low")  # 对应原始代码PCOUT
        
        if self.u_att[3] > 0.7:
            self.u_att[3] = 0.7
            # print("throttle too high")  # 对应原始代码PCOUT
        
        self.flag += 1
        
        return self.u_att
        
    def printf_param(self):
        """打印参数 - 对应原始代码的printf_param函数"""
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>ADRC Parameter <<<<<<<<<<<<<<<<<<<<<<<<<")
        print(f"ctrl_param.quad_mass     : {self.ctrl_param.quad_mass}")
        print(f"ctrl_param.hov_percent   : {self.ctrl_param.hov_percent}")
        print(f"pxy_int_max              : {self.ctrl_param.int_max[0]}")
        print(f"pz_int_max               : {self.ctrl_param.int_max[2]}")
        print(f"beta_max                 : {self.ctrl_param.beta_max}")
        print(f"C1                       : {self.ctrl_param.C[0]}")
        print(f"C2                       : {self.ctrl_param.C[1]}")
        print(f"sigma_D                  : {self.ctrl_param.sigma_D}")
        print(f"amesogain_l              : {self.ctrl_param.amesogain_l}")
        print(f"method_choose            : {self.method_choose}")
        
    def printf_result(self, u_att):
        """打印结果 - 对应原始代码的printf_result函数"""
        print("----> ADRC Position Controller Debug Info      : ")
        print(f"----> pos_des         : {self.desired_state.pos[0]:.2f} [m] {self.desired_state.pos[1]:.2f} [m] {self.desired_state.pos[2]:.2f} [m]")
        print(f"----> vel_des         : {self.desired_state.vel[0]:.2f} [m/s] {self.desired_state.vel[1]:.2f} [m/s] {self.desired_state.vel[2]:.2f} [m/s]")
        print(f"----> acc_des         : {self.desired_state.acc[0]:.2f} [m/s²] {self.desired_state.acc[1]:.2f} [m/s²] {self.desired_state.acc[2]:.2f} [m/s²]")
        print(f"----> pos_now         : {self.current_state.pos[0]:.2f} [m] {self.current_state.pos[1]:.2f} [m] {self.current_state.pos[2]:.2f} [m]")
        print(f"----> vel_now         : {self.current_state.vel[0]:.2f} [m/s] {self.current_state.vel[1]:.2f} [m/s] {self.current_state.vel[2]:.2f} [m/s]")
        print(f"----> u (Thrust)      : {self.u:.2f} [N]")
        print(f"----> un (nominal)    : {self.un:.2f} [N]")
        print(f"----> uf (feedback)   : {self.uf:.2f} [N]")
        print(f"----> uc (compensation): {self.uc:.2f} [N]")
        print(f"----> hatd (disturbance): {self.hatd:.2f} [N]")
        print(f"----> ESO_z [z1 z2 z3]: {self.ESO_z.z1:.3f} {self.ESO_z.z2:.3f} {self.ESO_z.z3:.3f}")
        print(f"----> u_att [X Y Z]   : {u_att[0]:.2f} [rad] {u_att[1]:.2f} [rad] {u_att[2]:.2f} [rad]")
        print(f"----> u_throttle      : {u_att[3]:.3f}")
        print(f"----> pos_error_mean  : {self.tracking_error.pos_error_mean:.3f} [m]")
        print(f"----> vel_error_mean  : {self.tracking_error.vel_error_mean:.3f} [m/s]")