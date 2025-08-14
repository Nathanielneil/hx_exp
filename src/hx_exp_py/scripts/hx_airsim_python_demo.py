#!/usr/bin/env python3

"""
HX AirSim Python Demo (No ROS Dependencies)
Direct AirSim API demonstration of takeoff, flight patterns, and landing
"""

import airsim
import time
import math
import numpy as np

class HXAirSimDemo:
    def __init__(self):
        self.client = None
        
    def connect(self):
        """Connect to AirSim"""
        try:
            print("Connecting to AirSim...")
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()
            print("✓ Connected to AirSim!")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def prepare_drone(self):
        """Enable API control and arm the drone"""
        try:
            print("Preparing drone...")
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            print("✓ Drone prepared!")
            return True
        except Exception as e:
            print(f"✗ Failed to prepare drone: {e}")
            return False
    
    def basic_takeoff_land_demo(self, height=3.0, hover_time=5.0):
        """Basic takeoff, hover, and landing demo"""
        print(f"\n--- Basic Takeoff and Landing Demo ---")
        print(f"Height: {height}m, Hover time: {hover_time}s")
        
        try:
            # Takeoff
            print("Taking off...")
            self.client.takeoffAsync().join()
            
            # Move to target height
            print(f"Moving to {height}m altitude...")
            self.client.moveToZAsync(-height, 2).join()
            
            # Hover
            print(f"Hovering for {hover_time} seconds...")
            time.sleep(hover_time)
            
            # Get position
            state = self.client.getMultirotorState()
            pos = state.kinematics_estimated.position
            print(f"Current position: x={pos.x_val:.2f}, y={pos.y_val:.2f}, z={pos.z_val:.2f}")
            
            # Land
            print("Landing...")
            self.client.landAsync().join()
            
            print("✓ Basic demo completed!")
            return True
            
        except Exception as e:
            print(f"✗ Demo failed: {e}")
            return False
    
    def circular_trajectory_demo(self, radius=3.0, height=5.0, speed=1.0):
        """Circular trajectory flight demo"""
        print(f"\n--- Circular Trajectory Demo ---")
        print(f"Radius: {radius}m, Height: {height}m, Speed: {speed}m/s")
        
        try:
            # Takeoff
            print("Taking off...")
            self.client.takeoffAsync().join()
            
            # Move to height
            print(f"Moving to {height}m altitude...")
            self.client.moveToZAsync(-height, 2).join()
            
            # Move to circle start point
            print("Moving to circle start position...")
            self.client.moveToPositionAsync(radius, 0, -height, speed).join()
            
            # Fly in circle
            print("Flying circular trajectory...")
            circle_duration = 2 * math.pi * radius / speed
            num_points = 32
            dt = circle_duration / num_points
            
            for i in range(num_points + 1):
                angle = 2 * math.pi * i / num_points
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                z = -height
                
                # Move to next point
                self.client.moveToPositionAsync(x, y, z, speed)
                time.sleep(dt)
                
                if i % 8 == 0:  # Progress update every 8 points
                    progress = (i / num_points) * 100
                    print(f"Circle progress: {progress:.1f}%")
            
            print("Circle completed!")
            
            # Return to center
            print("Returning to center...")
            self.client.moveToPositionAsync(0, 0, -height, speed).join()
            
            # Land
            print("Landing...")
            self.client.landAsync().join()
            
            print("✓ Circular trajectory demo completed!")
            return True
            
        except Exception as e:
            print(f"✗ Demo failed: {e}")
            return False
    
    def waypoint_demo(self, height=4.0, speed=2.0):
        """Waypoint navigation demo"""
        print(f"\n--- Waypoint Navigation Demo ---")
        
        waypoints = [
            (0, 0, -height),    # Start
            (5, 0, -height),    # Forward
            (5, 5, -height),    # Right
            (0, 5, -height),    # Back
            (0, 0, -height),    # Return to start
        ]
        
        try:
            # Takeoff
            print("Taking off...")
            self.client.takeoffAsync().join()
            
            # Visit each waypoint
            for i, (x, y, z) in enumerate(waypoints):
                print(f"Moving to waypoint {i+1}: ({x}, {y}, {-z})")
                self.client.moveToPositionAsync(x, y, z, speed).join()
                time.sleep(1)  # Brief pause at each waypoint
            
            # Land
            print("Landing...")
            self.client.landAsync().join()
            
            print("✓ Waypoint demo completed!")
            return True
            
        except Exception as e:
            print(f"✗ Demo failed: {e}")
            return False
    
    def cleanup(self):
        """Disable API control and cleanup"""
        if self.client:
            try:
                self.client.armDisarm(False)
                self.client.enableApiControl(False)
                print("✓ Cleanup completed")
            except:
                pass

def main():
    print("=" * 60)
    print("HX AirSim Python Demo")
    print("=" * 60)
    
    demo = HXAirSimDemo()
    
    # Connect to AirSim
    if not demo.connect():
        return
    
    # Prepare drone
    if not demo.prepare_drone():
        return
    
    try:
        while True:
            print("\nAvailable demos:")
            print("1) Basic Takeoff and Landing")
            print("2) Circular Trajectory Flight")
            print("3) Waypoint Navigation")
            print("4) Exit")
            
            choice = input("Select demo (1-4): ").strip()
            
            if choice == "1":
                height = float(input("Enter takeoff height (default 3.0m): ") or "3.0")
                hover_time = float(input("Enter hover time (default 5.0s): ") or "5.0")
                demo.basic_takeoff_land_demo(height, hover_time)
                
            elif choice == "2":
                radius = float(input("Enter circle radius (default 3.0m): ") or "3.0")
                height = float(input("Enter flight height (default 5.0m): ") or "5.0")
                speed = float(input("Enter flight speed (default 1.0m/s): ") or "1.0")
                demo.circular_trajectory_demo(radius, height, speed)
                
            elif choice == "3":
                height = float(input("Enter flight height (default 4.0m): ") or "4.0")
                speed = float(input("Enter flight speed (default 2.0m/s): ") or "2.0")
                demo.waypoint_demo(height, speed)
                
            elif choice == "4":
                break
                
            else:
                print("Invalid choice!")
    
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    
    except Exception as e:
        print(f"Demo error: {e}")
    
    finally:
        demo.cleanup()
        print("Demo finished!")

if __name__ == "__main__":
    main()