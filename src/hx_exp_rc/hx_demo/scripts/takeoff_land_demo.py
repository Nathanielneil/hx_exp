#!/usr/bin/env python3

import rospy
from hx_msgs.msg import UAVCommand, UAVState, UAVControlState
from geometry_msgs.msg import Point, Vector3
import time

class TakeoffLandDemo:
    def __init__(self):
        rospy.init_node('takeoff_land_demo', anonymous=True)
        
        # Publishers
        self.command_pub = rospy.Publisher('/Drone1/hx_uav/command', UAVCommand, queue_size=10)
        
        # Subscribers
        self.state_sub = rospy.Subscriber('/Drone1/hx_uav/state', UAVState, self.state_callback)
        self.control_state_sub = rospy.Subscriber('/Drone1/hx_uav/control_state', UAVControlState, self.control_state_callback)
        
        # State variables
        self.current_state = None
        self.current_control_state = None
        
        rospy.loginfo("[Takeoff Land Demo] Initialized - Waiting for UAV connection...")
        
        # Wait for subscribers to connect
        while self.command_pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        
        self.run_demo()
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def control_state_callback(self, msg):
        self.current_control_state = msg
    
    def wait_for_state(self, timeout=10.0):
        """Wait for state messages to be received"""
        start_time = time.time()
        while (self.current_state is None or self.current_control_state is None) and not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logerr("[Demo] Timeout waiting for state messages")
                return False
            rospy.sleep(0.1)
        return True
    
    def send_takeoff_command(self):
        """Send takeoff command"""
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.Agent_CMD = UAVCommand.Takeoff
        cmd.Control_mode = UAVCommand.PID_CONTROL
        cmd.Command_ID = "python_takeoff"
        
        self.command_pub.publish(cmd)
        rospy.loginfo("[Demo] Takeoff command sent")
    
    def send_move_command(self, x, y, z, vx=0.0, vy=0.0, vz=0.0):
        """Send move command to specific position"""
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.Agent_CMD = UAVCommand.Move_ENU
        cmd.Control_mode = UAVCommand.PID_CONTROL
        
        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        
        cmd.linear_vel.x = vx
        cmd.linear_vel.y = vy
        cmd.linear_vel.z = vz
        
        cmd.Command_ID = "python_move"
        
        self.command_pub.publish(cmd)
        rospy.loginfo(f"[Demo] Move command sent to [{x:.1f}, {y:.1f}, {z:.1f}]")
    
    def send_hold_command(self):
        """Send hold position command"""
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.Agent_CMD = UAVCommand.Hold
        cmd.Command_ID = "python_hold"
        
        self.command_pub.publish(cmd)
        rospy.loginfo("[Demo] Hold command sent")
    
    def send_land_command(self):
        """Send land command"""
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.Agent_CMD = UAVCommand.Land
        cmd.Command_ID = "python_land"
        
        self.command_pub.publish(cmd)
        rospy.loginfo("[Demo] Land command sent")
    
    def wait_for_takeoff_complete(self, timeout=10.0):
        """Wait for takeoff to complete"""
        start_time = time.time()
        while not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logwarn("[Demo] Takeoff timeout")
                return False
                
            if (self.current_control_state and 
                self.current_control_state.control_state == UAVControlState.COMMAND_CONTROL and
                self.current_state and self.current_state.position[2] > 2.5):
                rospy.loginfo("[Demo] Takeoff completed")
                return True
                
            rospy.sleep(0.1)
        return False
    
    def wait_for_position(self, target_x, target_y, target_z, tolerance=0.5, timeout=15.0):
        """Wait for UAV to reach target position"""
        start_time = time.time()
        while not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logwarn(f"[Demo] Position timeout for target [{target_x:.1f}, {target_y:.1f}, {target_z:.1f}]")
                return False
                
            if self.current_state:
                pos = self.current_state.position
                distance = ((pos[0] - target_x)**2 + (pos[1] - target_y)**2 + (pos[2] - target_z)**2)**0.5
                
                if distance < tolerance:
                    rospy.loginfo(f"[Demo] Reached target position [{target_x:.1f}, {target_y:.1f}, {target_z:.1f}]")
                    return True
                    
            rospy.sleep(0.1)
        return False
    
    def run_demo(self):
        """Execute the main demo sequence"""
        
        # Wait for initial state
        if not self.wait_for_state():
            return
        
        rospy.loginfo("=== HX UAV Takeoff-Land Demo Starting ===")
        
        try:
            # Step 1: Takeoff
            rospy.loginfo("[Demo] Step 1: Takeoff")
            self.send_takeoff_command()
            if not self.wait_for_takeoff_complete():
                rospy.logerr("[Demo] Takeoff failed")
                return
            
            rospy.sleep(2.0)  # Brief pause
            
            # Step 2: Move to position 1
            rospy.loginfo("[Demo] Step 2: Move to position [3, 0, 3]")
            self.send_move_command(3.0, 0.0, 3.0)
            if not self.wait_for_position(3.0, 0.0, 3.0):
                rospy.logwarn("[Demo] Position 1 timeout, continuing...")
            
            rospy.sleep(3.0)  # Hold position
            
            # Step 3: Move to position 2
            rospy.loginfo("[Demo] Step 3: Move to position [0, 3, 3]")
            self.send_move_command(0.0, 3.0, 3.0)
            if not self.wait_for_position(0.0, 3.0, 3.0):
                rospy.logwarn("[Demo] Position 2 timeout, continuing...")
            
            rospy.sleep(3.0)  # Hold position
            
            # Step 4: Move to position 3
            rospy.loginfo("[Demo] Step 4: Move to position [-3, 0, 3]")
            self.send_move_command(-3.0, 0.0, 3.0)
            if not self.wait_for_position(-3.0, 0.0, 3.0):
                rospy.logwarn("[Demo] Position 3 timeout, continuing...")
            
            rospy.sleep(3.0)  # Hold position
            
            # Step 5: Return to center
            rospy.loginfo("[Demo] Step 5: Return to center [0, 0, 3]")
            self.send_move_command(0.0, 0.0, 3.0)
            if not self.wait_for_position(0.0, 0.0, 3.0):
                rospy.logwarn("[Demo] Return to center timeout, continuing...")
            
            rospy.sleep(3.0)  # Hold at center
            
            # Step 6: Land
            rospy.loginfo("[Demo] Step 6: Landing")
            self.send_land_command()
            
            # Wait for landing
            rospy.sleep(10.0)
            
            rospy.loginfo("=== HX UAV Takeoff-Land Demo Completed Successfully ===")
            
        except rospy.ROSInterruptException:
            rospy.loginfo("[Demo] Demo interrupted by user")
        except Exception as e:
            rospy.logerr(f"[Demo] Demo failed with exception: {e}")

if __name__ == '__main__':
    try:
        demo = TakeoffLandDemo()
    except rospy.ROSInterruptException:
        pass