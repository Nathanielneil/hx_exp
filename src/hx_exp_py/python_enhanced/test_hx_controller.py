#!/usr/bin/env python3
"""
测试HX控制器 - 验证Python版本功能
Test HX Controller - Verify Python version functionality
"""

import time
import numpy as np
from hx_airsim_controller import HXAirSimController

def test_controller_initialization():
    """测试控制器初始化"""
    print("=" * 50)
    print("测试HX控制器初始化")
    print("=" * 50)
    
    controller = HXAirSimController()
    
    if controller.initialize():
        print("控制器初始化成功")
        
        # 启动控制系统
        controller.start()
        time.sleep(1)
        
        # 测试状态获取
        state = controller.get_state()
        print(f"状态获取成功: 连接={state.connected}")
        
        # 打印状态
        controller.print_status()
        
        # 停止系统
        controller.stop()
        print("控制器停止成功")
        
        return True
    else:
        print("控制器初始化失败")
        return False

def test_simple_control():
    """测试简单控制功能"""
    print("=" * 50)
    print("测试简单控制功能")
    print("使用原有控制算法")
    print("=" * 50)
    
    controller = HXAirSimController()
    
    if not controller.initialize():
        print("控制器初始化失败")
        return False
    
    try:
        controller.start()
        time.sleep(2)
        
        print("1. 当前位置:")
        controller.print_status()
        
        print("\n2. 设置目标位置 [0, 0, -3] (向上3米)...")
        controller.set_target_position(0.0, 0.0, -3.0, 0.0)
        
        # 模拟几秒控制
        for i in range(10):
            time.sleep(1)
            pos = controller.get_position()
            error = controller.get_tracking_error()
            print(f"   时刻 {i+1}s: 位置=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}], 误差={error:.3f}m")
        
        print("\n3. 最终状态:")
        controller.print_status()
        
        return True
        
    except Exception as e:
        print(f"测试过程中出错: {e}")
        return False
    finally:
        controller.stop()

if __name__ == "__main__":
    print("HX AirSim控制器测试")
    print("保持原有Prometheus算法不变")
    
    # 测试初始化
    if test_controller_initialization():
        print("\n" + "="*50)
        input("按回车键继续测试控制功能...")
        
        # 测试控制功能
        test_simple_control()
    
    print("\n测试完成")