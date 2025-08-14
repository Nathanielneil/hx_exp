#!/usr/bin/env python3
"""
键盘控制界面 - 保持原有控制算法不变的Python版本
Keyboard Control Interface - Python version keeping original control algorithms unchanged
"""

import sys
import os
import threading
import time
import signal
import numpy as np

# 尝试导入键盘输入库
try:
    import keyboard
    HAS_KEYBOARD = True
except ImportError:
    HAS_KEYBOARD = False
    print("Warning: keyboard library not available. Install with: pip install keyboard")

try:
    import getch
    HAS_GETCH = True
except ImportError:
    HAS_GETCH = False

# 如果都没有，使用简单的input方式
if not HAS_KEYBOARD and not HAS_GETCH:
    print("Warning: Using simple input mode. For better experience, install keyboard library")

from hx_airsim_controller import HXAirSimController, ControllerType, ControlMode

class KeyboardController:
    """键盘控制器 - 提供键盘操作UAV的界面"""
    
    def __init__(self):
        self.controller = HXAirSimController()
        self.running = False
        self.control_thread = None
        
        # 控制参数
        self.pos_step = 1.0      # 位置步长 (米)
        self.vel_step = 1.0      # 速度步长 (米/秒)
        self.yaw_step = 0.5      # 航向步长 (弧度)
        self.height_step = 0.5   # 高度步长 (米)
        
        # 当前控制状态
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_vel = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        
        # 控制器类型
        self.controller_types = ['PID', 'UDE', 'ADRC']
        self.current_controller_index = 0
        
    def initialize(self):
        """初始化控制器"""
        if not self.controller.initialize():
            print("❌ 控制器初始化失败")
            return False
        
        print("✅ 控制器初始化成功")
        return True
    
    def start(self):
        """启动键盘控制"""
        if not self.initialize():
            return False
        
        self.controller.start()
        self.running = True
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print("\n" + "="*60)
        print("🎮 HX UAV 键盘控制界面")
        print("保持原有Prometheus控制算法核心不变")
        print("="*60)
        
        if HAS_KEYBOARD:
            self._keyboard_loop()
        else:
            self._simple_input_loop()
        
        return True
    
    def stop(self):
        """停止控制"""
        self.running = False
        if self.control_thread:
            self.control_thread.join()
        self.controller.stop()
    
    def _control_loop(self):
        """控制循环线程"""
        while self.running:
            try:
                # 更新当前状态
                self.current_pos = self.controller.get_position()
                
                # 简单的状态显示（每2秒更新一次）
                time.sleep(2)
                
            except Exception as e:
                print(f"控制循环错误: {e}")
                break
    
    def _keyboard_loop(self):
        """使用keyboard库的键盘循环"""
        print(self._get_help_text())
        
        try:
            while self.running:
                time.sleep(0.1)  # 防止CPU占用过高
                
                # 检查各种按键
                if keyboard.is_pressed('q'):
                    print("\n退出键盘控制...")
                    break
                elif keyboard.is_pressed('h'):
                    print(self._get_help_text())
                    time.sleep(0.5)  # 防止重复触发
                elif keyboard.is_pressed('s'):
                    self._print_status()
                    time.sleep(0.5)
                elif keyboard.is_pressed('t'):
                    self._takeoff()
                    time.sleep(1)
                elif keyboard.is_pressed('l'):
                    self._land()
                    time.sleep(1)
                elif keyboard.is_pressed('c'):
                    self._switch_controller()
                    time.sleep(0.5)
                elif keyboard.is_pressed('m'):
                    self._switch_control_mode()
                    time.sleep(0.5)
                # 位置控制
                elif keyboard.is_pressed('w'):
                    self._move_position(1.0, 0.0, 0.0)  # 前进
                elif keyboard.is_pressed('s'):
                    self._move_position(-1.0, 0.0, 0.0)  # 后退
                elif keyboard.is_pressed('a'):
                    self._move_position(0.0, -1.0, 0.0)  # 左移
                elif keyboard.is_pressed('d'):
                    self._move_position(0.0, 1.0, 0.0)  # 右移
                elif keyboard.is_pressed('up'):
                    self._move_position(0.0, 0.0, -self.height_step)  # 上升
                elif keyboard.is_pressed('down'):
                    self._move_position(0.0, 0.0, self.height_step)  # 下降
                elif keyboard.is_pressed('left'):
                    self._change_yaw(-self.yaw_step)  # 左转
                elif keyboard.is_pressed('right'):
                    self._change_yaw(self.yaw_step)  # 右转
                # 速度控制
                elif keyboard.is_pressed('i'):
                    self._set_velocity(1.0, 0.0, 0.0)  # 前进速度
                elif keyboard.is_pressed('k'):
                    self._set_velocity(-1.0, 0.0, 0.0)  # 后退速度
                elif keyboard.is_pressed('j'):
                    self._set_velocity(0.0, -1.0, 0.0)  # 左移速度
                elif keyboard.is_pressed('l'):
                    self._set_velocity(0.0, 1.0, 0.0)  # 右移速度
                elif keyboard.is_pressed('space'):
                    self._stop_velocity()  # 停止
                    time.sleep(0.5)
                    
        except KeyboardInterrupt:
            print("\n用户中断")
        except Exception as e:
            print(f"\n键盘控制错误: {e}")
    
    def _simple_input_loop(self):
        """简单输入模式的控制循环"""
        print(self._get_simple_help_text())
        
        try:
            while self.running:
                try:
                    cmd = input("\n输入命令 (h=帮助, q=退出): ").strip().lower()
                    
                    if cmd == 'q':
                        print("退出键盘控制...")
                        break
                    elif cmd == 'h':
                        print(self._get_simple_help_text())
                    elif cmd == 's':
                        self._print_status()
                    elif cmd == 't':
                        self._takeoff()
                    elif cmd == 'land':
                        self._land()
                    elif cmd == 'c':
                        self._switch_controller()
                    elif cmd == 'm':
                        self._switch_control_mode()
                    elif cmd.startswith('pos'):
                        self._handle_position_command(cmd)
                    elif cmd.startswith('vel'):
                        self._handle_velocity_command(cmd)
                    elif cmd.startswith('yaw'):
                        self._handle_yaw_command(cmd)
                    else:
                        print("未知命令，输入 'h' 查看帮助")
                        
                except KeyboardInterrupt:
                    print("\n用户中断")
                    break
                except Exception as e:
                    print(f"命令处理错误: {e}")
                    
        except Exception as e:
            print(f"输入循环错误: {e}")
    
    def _handle_position_command(self, cmd):
        """处理位置命令"""
        try:
            parts = cmd.split()
            if len(parts) == 4:  # pos x y z
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                self.controller.set_target_position(x, y, z, self.current_yaw)
                print(f"设置目标位置: [{x:.1f}, {y:.1f}, {z:.1f}]")
            else:
                print("位置命令格式: pos <x> <y> <z>")
        except ValueError:
            print("位置参数必须是数字")
    
    def _handle_velocity_command(self, cmd):
        """处理速度命令"""
        try:
            parts = cmd.split()
            if len(parts) == 4:  # vel vx vy vz
                vx, vy, vz = float(parts[1]), float(parts[2]), float(parts[3])
                self.controller.set_target_velocity(vx, vy, vz)
                print(f"设置目标速度: [{vx:.1f}, {vy:.1f}, {vz:.1f}]")
            else:
                print("速度命令格式: vel <vx> <vy> <vz>")
        except ValueError:
            print("速度参数必须是数字")
    
    def _handle_yaw_command(self, cmd):
        """处理航向命令"""
        try:
            parts = cmd.split()
            if len(parts) == 2:  # yaw angle
                yaw = float(parts[1])
                self.current_yaw = yaw
                pos = self.controller.get_position()
                self.controller.set_target_position(pos[0], pos[1], pos[2], yaw)
                print(f"设置目标航向: {yaw:.2f} rad ({yaw*180/np.pi:.1f}°)")
            else:
                print("航向命令格式: yaw <angle_rad>")
        except ValueError:
            print("航向参数必须是数字")
    
    def _move_position(self, dx, dy, dz):
        """相对位置移动"""
        current_pos = self.controller.get_position()
        new_pos = [
            current_pos[0] + dx * self.pos_step,
            current_pos[1] + dy * self.pos_step,
            current_pos[2] + dz * self.height_step
        ]
        
        self.controller.set_target_position(new_pos[0], new_pos[1], new_pos[2], self.current_yaw)
        print(f"移动到: [{new_pos[0]:.1f}, {new_pos[1]:.1f}, {new_pos[2]:.1f}]")
    
    def _change_yaw(self, dyaw):
        """改变航向"""
        self.current_yaw += dyaw
        pos = self.controller.get_position()
        self.controller.set_target_position(pos[0], pos[1], pos[2], self.current_yaw)
        print(f"航向: {self.current_yaw:.2f} rad ({self.current_yaw*180/np.pi:.1f}°)")
    
    def _set_velocity(self, vx, vy, vz):
        """设置速度"""
        self.controller.set_target_velocity(vx * self.vel_step, vy * self.vel_step, vz * self.vel_step)
        print(f"速度: [{vx*self.vel_step:.1f}, {vy*self.vel_step:.1f}, {vz*self.vel_step:.1f}]")
    
    def _stop_velocity(self):
        """停止速度"""
        self.controller.set_target_velocity(0.0, 0.0, 0.0)
        print("停止移动")
    
    def _takeoff(self):
        """起飞"""
        print("起飞到3米高度...")
        self.controller.takeoff(3.0)
        self.current_yaw = 0.0
    
    def _land(self):
        """降落"""
        print("开始降落...")
        self.controller.land()
    
    def _switch_controller(self):
        """切换控制器"""
        self.current_controller_index = (self.current_controller_index + 1) % len(self.controller_types)
        controller_type = self.controller_types[self.current_controller_index]
        
        # 根据类型创建控制器
        if controller_type == 'PID':
            controller_enum = ControllerType.PID
        elif controller_type == 'UDE':
            controller_enum = ControllerType.UDE
        elif controller_type == 'ADRC':
            controller_enum = ControllerType.ADRC
        else:
            controller_enum = ControllerType.PID
        
        self.controller.switch_controller(controller_enum)
        print(f"切换到 {controller_type} 控制器")
    
    def _switch_control_mode(self):
        """切换控制模式"""
        # 简单地在位置控制和速度控制之间切换
        current_mode = self.controller.get_control_mode()
        if current_mode == ControlMode.POSITION_HOLD:
            self.controller.set_control_mode(ControlMode.VELOCITY_CONTROL)
            print("切换到速度控制模式")
        else:
            self.controller.set_control_mode(ControlMode.POSITION_HOLD)
            print("切换到位置控制模式")
    
    def _print_status(self):
        """打印状态信息"""
        try:
            pos = self.controller.get_position()
            vel = self.controller.get_velocity()
            error = self.controller.get_tracking_error()
            mode = self.controller.get_control_mode()
            controller_type = self.controller_types[self.current_controller_index]
            
            print("\n" + "="*50)
            print("📊 UAV 状态信息")
            print("="*50)
            print(f"位置:     [{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}] m")
            print(f"速度:     [{vel[0]:6.2f}, {vel[1]:6.2f}, {vel[2]:6.2f}] m/s")
            print(f"航向:     {self.current_yaw:6.2f} rad ({self.current_yaw*180/np.pi:6.1f}°)")
            print(f"跟踪误差: {error:.3f} m")
            print(f"控制器:   {controller_type}")
            print(f"控制模式: {mode.name if hasattr(mode, 'name') else str(mode)}")
            print("="*50)
            
        except Exception as e:
            print(f"状态查询错误: {e}")
    
    def _get_help_text(self):
        """获取帮助文本 - 键盘模式"""
        return """
🎮 键盘控制帮助 (保持原有控制算法不变)
=========================================================
基本控制:
  Q         - 退出程序
  H         - 显示帮助
  S         - 显示状态
  T         - 起飞 (3米)
  L         - 降落
  
位置控制 (使用原有PID/UDE/ADRC算法):
  W/S       - 前进/后退
  A/D       - 左移/右移
  ↑/↓       - 上升/下降
  ←/→       - 左转/右转
  
速度控制:
  I/K       - 前进/后退速度
  J/L       - 左移/右移速度
  空格      - 停止移动
  
控制器切换:
  C         - 切换控制器 (PID → UDE → ADRC)
  M         - 切换控制模式 (位置 ↔ 速度)

注意: 所有控制算法完全保持原有Prometheus实现不变
=========================================================
"""
    
    def _get_simple_help_text(self):
        """获取帮助文本 - 简单输入模式"""
        return """
🎮 键盘控制帮助 (保持原有控制算法不变)
=========================================================
可用命令:
  h              - 显示帮助
  q              - 退出程序
  s              - 显示状态
  t              - 起飞 (3米)
  land           - 降落
  c              - 切换控制器 (PID → UDE → ADRC)
  m              - 切换控制模式
  
位置控制 (使用原有算法):
  pos <x> <y> <z>     - 飞到指定位置
  yaw <angle_rad>     - 设置航向角 (弧度)
  
速度控制:
  vel <vx> <vy> <vz>  - 设置速度

示例:
  pos 5 0 -3         - 飞到 (5, 0, -3) 位置
  vel 1 0 0          - 以 1m/s 前进
  yaw 1.57           - 航向设为 90°

注意: 所有控制算法完全保持原有Prometheus实现不变
=========================================================
"""

def signal_handler(sig, frame):
    """信号处理器"""
    print("\n\n收到中断信号，正在安全退出...")
    global keyboard_controller
    if 'keyboard_controller' in globals():
        keyboard_controller.stop()
    sys.exit(0)

def main():
    """主函数"""
    global keyboard_controller
    
    # 设置信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    print("启动HX UAV键盘控制...")
    print("保持原有Prometheus控制算法核心不变")
    
    keyboard_controller = KeyboardController()
    
    try:
        keyboard_controller.start()
    except Exception as e:
        print(f"启动失败: {e}")
    finally:
        keyboard_controller.stop()
        print("键盘控制已退出")

if __name__ == "__main__":
    main()