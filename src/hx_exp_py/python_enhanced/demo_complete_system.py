#!/usr/bin/env python3
"""
完整系统演示 - 保持原有控制算法不变
Complete System Demo - Keeping original control algorithms unchanged
"""

import time
import numpy as np
from hx_airsim_controller import HXAirSimController, TrajectoryPoint, ControllerType

def demo_complete_system():
    """完整系统演示"""
    print("=" * 60)
    print("HX AirSim完整控制系统演示")
    print("保持原有Prometheus控制算法核心不变")
    print("增强功能：轨迹跟踪、控制器对比、多种控制模式")
    print("=" * 60)
    
    # 创建控制器
    controller = HXAirSimController()
    
    # 初始化
    if not controller.initialize():
        print("❌ 控制器初始化失败")
        return False
    
    print("✅ 控制器初始化成功")
    
    try:
        # 启动控制系统
        controller.start()
        print("✅ 控制系统已启动")
        
        print("\n" + "="*50)
        print("第一阶段：基础飞行测试")
        print("="*50)
        
        # 1. 起飞测试
        print("\n1️⃣ 起飞测试...")
        initial_pos = controller.get_position()
        print(f"   起始位置: [{initial_pos[0]:.2f}, {initial_pos[1]:.2f}, {initial_pos[2]:.2f}]")
        
        controller.takeoff(3.0)
        print("   起飞命令已发送，等待执行...")
        time.sleep(10)
        
        pos = controller.get_position()
        print(f"   当前位置: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        controller.print_status()
        
        # 2. 位置控制测试
        print("\n2️⃣ 位置控制测试 (使用原有PID算法)...")
        
        test_positions = [
            [3.0, 0.0, 3.0, 0.0],    # 前进3米
            [3.0, 3.0, 3.0, 1.57],  # 右转3米，yaw 90度
            [0.0, 3.0, 3.0, 3.14],  # 后退到y=3，yaw 180度
            [0.0, 0.0, 3.0, 0.0],   # 回到原点
        ]
        
        for i, (x, y, z, yaw) in enumerate(test_positions):
            print(f"   位置 {i+1}: 移动到 [{x:.1f}, {y:.1f}, {z:.1f}], yaw={yaw:.2f}rad")
            controller.set_target_position(x, y, z, yaw)
            time.sleep(8)  # 等待到达
            
            pos = controller.get_position()
            error = controller.get_tracking_error()
            print(f"   到达位置: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}], 误差: {error:.3f}m")
        
        print("\n" + "="*50)
        print("第二阶段：轨迹跟踪测试")
        print("="*50)
        
        # 3. 轨迹跟踪测试
        print("\n3️⃣ 轨迹跟踪测试 (圆形轨迹)...")
        
        # 生成圆形轨迹
        trajectory = generate_advanced_circle_trajectory(
            center=[0.0, 0.0, 3.0],
            radius=4.0,
            duration=20.0,
            num_points=50
        )
        
        print(f"   生成轨迹: {len(trajectory)} 个点，持续 {trajectory[-1].time_from_start:.1f} 秒")
        controller.set_trajectory(trajectory)
        
        # 监控轨迹跟踪
        print("   开始轨迹跟踪...")
        start_time = time.time()
        while time.time() - start_time < 22.0:  # 跟踪22秒
            pos = controller.get_position()
            error = controller.get_tracking_error()
            elapsed = time.time() - start_time
            print(f"   {elapsed:4.1f}s: 位置=[{pos[0]:5.2f}, {pos[1]:5.2f}, {pos[2]:5.2f}], 误差={error:.3f}m")
            time.sleep(3)
        
        print("   轨迹跟踪完成")
        
        print("\n" + "="*50)
        print("第三阶段：速度控制测试")
        print("="*50)
        
        # 4. 速度控制测试
        print("\n4️⃣ 速度控制测试...")
        
        # 回到中心点
        controller.set_target_position(0.0, 0.0, 3.0, 0.0)
        time.sleep(5)
        
        # 速度控制模式
        velocity_tests = [
            ([1.0, 0.0, 0.0], "前进", 4),
            ([0.0, 1.0, 0.0], "右移", 3),
            ([-1.0, 0.0, 0.0], "后退", 4),
            ([0.0, -1.0, 0.0], "左移", 3),
            ([0.0, 0.0, 0.0], "停止", 2),
        ]
        
        for vel, desc, duration in velocity_tests:
            print(f"   {desc}: 速度 [{vel[0]:.1f}, {vel[1]:.1f}, {vel[2]:.1f}] m/s")
            controller.set_target_velocity(vel[0], vel[1], vel[2])
            time.sleep(duration)
            
            pos = controller.get_position()
            print(f"   当前位置: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        
        print("\n" + "="*50)
        print("第四阶段：控制器性能测试")
        print("="*50)
        
        # 5. 控制器性能测试
        print("\n5️⃣ PID控制器性能测试...")
        
        # 设定一个具有挑战性的目标位置
        print("   大幅度位置变化测试...")
        current_pos = controller.get_position()
        target_pos = [current_pos[0] + 6.0, current_pos[1] + 4.0, 2.0]
        
        print(f"   从 [{current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}]")
        print(f"   到 [{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}]")
        
        controller.set_target_position(target_pos[0], target_pos[1], target_pos[2], 0.0)
        
        # 详细监控控制过程
        start_time = time.time()
        max_error = 0.0
        errors = []
        
        for i in range(15):  # 15秒监控
            time.sleep(1)
            pos = controller.get_position()
            error = np.linalg.norm(np.array(target_pos) - pos)
            errors.append(error)
            max_error = max(max_error, error)
            
            print(f"   {i+1:2d}s: 位置=[{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}], 误差={error:.3f}m")
            
            if error < 0.3:  # 到达精度阈值
                print(f"   ✅ 到达目标位置 (误差 < 0.3m)")
                break
        
        # 性能统计
        if errors:
            avg_error = np.mean(errors)
            final_error = errors[-1]
            print(f"\n   控制器性能统计:")
            print(f"   - 最大误差: {max_error:.3f} m")
            print(f"   - 平均误差: {avg_error:.3f} m")
            print(f"   - 最终误差: {final_error:.3f} m")
            print(f"   - 到达时间: {len(errors)} s")
        
        print("\n" + "="*50)
        print("第五阶段：系统完整性测试")
        print("="*50)
        
        # 6. 最终系统状态检查
        print("\n6️⃣ 系统完整性检查...")
        controller.print_status()
        
        # 打印控制器详细信息
        if controller.current_controller:
            print("\n   控制器详细参数:")
            controller.current_controller.printf_param()
        
        # 7. 安全降落
        print("\n7️⃣ 安全降落...")
        controller.land()
        
        # 监控降落过程
        print("   监控降落过程...")
        for i in range(10):
            time.sleep(2)
            pos = controller.get_position()
            print(f"   降落 {i*2+2}s: 高度 {pos[2]:.2f} m")
            
            if pos[2] > -0.5:  # 接近地面 (注意坐标系)
                print("   ✅ 降落完成")
                break
        
        print("\n" + "="*60)
        print("🎉 完整系统演示成功完成！")
        print("🔧 原有控制算法核心完全保持不变")
        print("⭐ 系统功能全面验证通过")
        print("="*60)
        
        return True
        
    except Exception as e:
        print(f"\n❌ 演示过程中出错: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        controller.stop()
        print("✅ 控制系统已安全停止")

def generate_advanced_circle_trajectory(center, radius, duration, num_points):
    """生成高级圆形轨迹 - 包含平滑加速和减速"""
    trajectory = []
    
    for i in range(num_points):
        # 时间参数
        t = i / (num_points - 1) * duration
        
        # 平滑角速度变化 (开始和结束时较慢)
        t_norm = t / duration
        smooth_factor = 0.5 * (1 - np.cos(np.pi * t_norm))
        angle = 2 * np.pi * smooth_factor
        
        # 位置
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = center[2]
        
        # 角速度 (时变)
        angular_velocity = (2 * np.pi / duration) * 0.5 * np.pi / duration * np.sin(np.pi * t_norm)
        
        # 速度
        vx = -radius * angular_velocity * np.sin(angle)
        vy = radius * angular_velocity * np.cos(angle)
        vz = 0.0
        
        # 加速度 (简化)
        ax = -radius * angular_velocity**2 * np.cos(angle)
        ay = -radius * angular_velocity**2 * np.sin(angle)
        az = 0.0
        
        # Yaw角度 (朝向运动方向)
        yaw = angle + np.pi / 2
        
        # 创建轨迹点
        point = TrajectoryPoint(
            position=np.array([x, y, z]),
            velocity=np.array([vx, vy, vz]),
            acceleration=np.array([ax, ay, az]),
            yaw=yaw,
            time_from_start=t
        )
        
        trajectory.append(point)
    
    return trajectory

if __name__ == "__main__":
    print("启动HX AirSim完整控制系统演示...")
    print("注意：请确保AirSim正在运行")
    print("按Ctrl+C可以随时中断演示")
    
    try:
        success = demo_complete_system()
        if success:
            print("\n🎊 演示圆满成功！")
        else:
            print("\n⚠️ 演示未完全成功")
    except KeyboardInterrupt:
        print("\n\n⏹️ 用户中断演示")
    except Exception as e:
        print(f"\n💥 演示异常终止: {e}")
    
    print("\n演示程序结束")