#!/usr/bin/env python3

import rospy
import airsim
import numpy as np
import time
import threading
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from hx_msgs.msg import UAVState, AirSimState
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs

class HXAirSimInterfacePython:
    def __init__(self):
        rospy.init_node('hx_airsim_interface_python', anonymous=True)
        
        # Parameters
        self.airsim_ip = rospy.get_param('~airsim_ip', 'localhost')
        self.airsim_port = rospy.get_param('~airsim_port', 41451)
        self.vehicle_name = rospy.get_param('~vehicle_name', 'Drone1')
        self.state_update_frequency = rospy.get_param('~state_update_frequency', 50.0)
        self.use_ned_frame = rospy.get_param('~use_ned_frame', True)
        
        # AirSim client
        self.client = None
        self.connected = False
        self.running = False
        
        # Publishers
        self.uav_state_pub = rospy.Publisher(f'/{self.vehicle_name}/hx_uav/state', UAVState, queue_size=10)
        self.airsim_state_pub = rospy.Publisher(f'/{self.vehicle_name}/hx_uav/airsim_state', AirSimState, queue_size=10)
        self.imu_pub = rospy.Publisher(f'/{self.vehicle_name}/hx_uav/imu', Imu, queue_size=10)
        self.pose_pub = rospy.Publisher(f'/{self.vehicle_name}/hx_uav/pose', PoseStamped, queue_size=10)
        
        # Subscribers
        self.command_sub = rospy.Subscriber(f'/{self.vehicle_name}/hx_uav/airsim_command', 
                                          TwistStamped, self.airsim_command_callback)
        
        # State variables
        self.current_uav_state = UAVState()
        self.current_airsim_state = AirSimState()
        
        # Threading
        self.state_update_thread = None
        self.thread_lock = threading.Lock()
        
        rospy.loginfo(f"[HX AirSim Interface Python] Initializing for vehicle: {self.vehicle_name}")
        
        # Initialize connection
        if self.connect_to_airsim():
            self.start()
        else:
            rospy.logerr("[HX AirSim Interface Python] Failed to initialize AirSim connection")

    def connect_to_airsim(self):
        """Connect to AirSim"""
        try:
            rospy.loginfo(f"[HX AirSim Interface Python] Connecting to AirSim at {self.airsim_ip}:{self.airsim_port}")
            
            self.client = airsim.MultirotorClient(ip=self.airsim_ip, port=self.airsim_port)
            self.client.confirmConnection()
            
            # Enable API control
            self.client.enableApiControl(True, self.vehicle_name)
            
            # Arm the vehicle
            self.client.armDisarm(True, self.vehicle_name)
            
            self.connected = True
            rospy.loginfo(f"[HX AirSim Interface Python] Successfully connected to AirSim for vehicle: {self.vehicle_name}")
            return True
            
        except Exception as e:
            rospy.logerr(f"[HX AirSim Interface Python] Failed to connect to AirSim: {e}")
            return False

    def start(self):
        """Start the interface"""
        if not self.connected:
            rospy.logerr("[HX AirSim Interface Python] Cannot start - not connected to AirSim")
            return
            
        self.running = True
        
        # Start state update thread
        self.state_update_thread = threading.Thread(target=self.state_update_loop)
        self.state_update_thread.daemon = True
        self.state_update_thread.start()
        
        rospy.loginfo("[HX AirSim Interface Python] Interface started successfully")

    def stop(self):
        """Stop the interface"""
        self.running = False
        
        if self.state_update_thread and self.state_update_thread.is_alive():
            self.state_update_thread.join(timeout=2.0)
        
        if self.client and self.connected:
            try:
                self.client.enableApiControl(False, self.vehicle_name)
            except:
                pass
        
        rospy.loginfo("[HX AirSim Interface Python] Interface stopped")

    def state_update_loop(self):
        """State update loop running in separate thread"""
        rate = rospy.Rate(self.state_update_frequency)
        
        while self.running and not rospy.is_shutdown():
            try:
                self.update_vehicle_state()
                self.publish_all_states()
            except Exception as e:
                rospy.logwarn(f"[HX AirSim Interface Python] State update error: {e}")
                
            rate.sleep()

    def update_vehicle_state(self):
        """Update vehicle state from AirSim"""
        if not self.connected or not self.client:
            return
            
        try:
            # Get vehicle pose and state
            pose = self.client.simGetVehiclePose(self.vehicle_name)
            state = self.client.getMultirotorState(self.vehicle_name)
            imu_data = self.client.getImuData(vehicle_name=self.vehicle_name)
            
            with self.thread_lock:
                # Update UAV state
                current_time = rospy.Time.now()
                self.current_uav_state.header.stamp = current_time
                self.current_uav_state.header.frame_id = "world"
                
                # Position (NED to ENU conversion if needed)
                if self.use_ned_frame:
                    # AirSim uses NED, ROS typically uses ENU
                    self.current_uav_state.position[0] = pose.position.x_val
                    self.current_uav_state.position[1] = pose.position.y_val  
                    self.current_uav_state.position[2] = -pose.position.z_val  # Convert NED Z to ENU Z
                else:
                    self.current_uav_state.position[0] = pose.position.x_val
                    self.current_uav_state.position[1] = pose.position.y_val
                    self.current_uav_state.position[2] = pose.position.z_val
                
                # Velocity
                vel = state.kinematics_estimated.linear_velocity
                if self.use_ned_frame:
                    self.current_uav_state.velocity[0] = vel.x_val
                    self.current_uav_state.velocity[1] = vel.y_val
                    self.current_uav_state.velocity[2] = -vel.z_val
                else:
                    self.current_uav_state.velocity[0] = vel.x_val
                    self.current_uav_state.velocity[1] = vel.y_val
                    self.current_uav_state.velocity[2] = vel.z_val
                
                # Orientation (quaternion)
                self.current_uav_state.attitude_q.w = pose.orientation.w_val
                self.current_uav_state.attitude_q.x = pose.orientation.x_val
                self.current_uav_state.attitude_q.y = pose.orientation.y_val
                self.current_uav_state.attitude_q.z = pose.orientation.z_val
                
                # Convert quaternion to euler angles
                euler = self.quaternion_to_euler(pose.orientation)
                self.current_uav_state.attitude[0] = euler[0]  # roll
                self.current_uav_state.attitude[1] = euler[1]  # pitch
                self.current_uav_state.attitude[2] = euler[2]  # yaw
                
                # System status
                self.current_uav_state.connected = self.connected
                self.current_uav_state.armed = True  # Assume armed if connected
                self.current_uav_state.odom_valid = True
                self.current_uav_state.mode = "GUIDED"
                
                # Update AirSim specific state
                self.current_airsim_state.header.stamp = current_time
                self.current_airsim_state.header.frame_id = "world"
                
                # Pose
                self.current_airsim_state.pose.position.x = pose.position.x_val
                self.current_airsim_state.pose.position.y = pose.position.y_val
                self.current_airsim_state.pose.position.z = pose.position.z_val
                self.current_airsim_state.pose.orientation.w = pose.orientation.w_val
                self.current_airsim_state.pose.orientation.x = pose.orientation.x_val
                self.current_airsim_state.pose.orientation.y = pose.orientation.y_val
                self.current_airsim_state.pose.orientation.z = pose.orientation.z_val
                
                # Velocities
                self.current_airsim_state.linear_velocity.x = vel.x_val
                self.current_airsim_state.linear_velocity.y = vel.y_val
                self.current_airsim_state.linear_velocity.z = vel.z_val
                
                ang_vel = state.kinematics_estimated.angular_velocity
                self.current_airsim_state.angular_velocity.x = ang_vel.x_val
                self.current_airsim_state.angular_velocity.y = ang_vel.y_val
                self.current_airsim_state.angular_velocity.z = ang_vel.z_val
                
                # Status
                self.current_airsim_state.api_control_enabled = True
                self.current_airsim_state.vehicle_ready = True
                self.current_airsim_state.collision_detected = False  # Could check collision info
                
        except Exception as e:
            rospy.logwarn(f"[HX AirSim Interface Python] Failed to update vehicle state: {e}")
            self.connected = False

    def publish_all_states(self):
        """Publish all state messages"""
        with self.thread_lock:
            self.uav_state_pub.publish(self.current_uav_state)
            self.airsim_state_pub.publish(self.current_airsim_state)
            
            # Publish IMU data
            self.publish_imu()
            
            # Publish pose
            self.publish_pose()

    def publish_imu(self):
        """Publish IMU data"""
        try:
            imu_data = self.client.getImuData(vehicle_name=self.vehicle_name)
            
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "base_link"
            
            # Orientation
            imu_msg.orientation = self.current_uav_state.attitude_q
            
            # Angular velocity
            imu_msg.angular_velocity.x = imu_data.angular_velocity.x_val
            imu_msg.angular_velocity.y = imu_data.angular_velocity.y_val
            imu_msg.angular_velocity.z = imu_data.angular_velocity.z_val
            
            # Linear acceleration
            imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
            imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y_val
            imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z_val
            
            self.imu_pub.publish(imu_msg)
            
        except Exception as e:
            rospy.logwarn(f"[HX AirSim Interface Python] Failed to publish IMU: {e}")

    def publish_pose(self):
        """Publish pose data"""
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            
            pose_msg.pose.position.x = self.current_uav_state.position[0]
            pose_msg.pose.position.y = self.current_uav_state.position[1]
            pose_msg.pose.position.z = self.current_uav_state.position[2]
            
            pose_msg.pose.orientation = self.current_uav_state.attitude_q
            
            self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            rospy.logwarn(f"[HX AirSim Interface Python] Failed to publish pose: {e}")

    def airsim_command_callback(self, msg):
        """Handle AirSim control commands"""
        if not self.connected or not self.client:
            return
            
        try:
            # Extract control commands
            roll = msg.twist.angular.x
            pitch = msg.twist.angular.y
            yaw_rate = msg.twist.angular.z
            throttle = msg.twist.linear.z
            
            # Convert to AirSim command
            # For now, use moveByRollPitchYawZAsync
            z = self.current_uav_state.position[2] if self.use_ned_frame else -self.current_uav_state.position[2]
            
            self.client.moveByRollPitchYawZAsync(
                roll, -pitch, yaw_rate, z, 0.02, vehicle_name=self.vehicle_name
            )
            
        except Exception as e:
            rospy.logwarn(f"[HX AirSim Interface Python] Failed to send control command: {e}")

    @staticmethod
    def quaternion_to_euler(q):
        """Convert quaternion to euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w_val * q.x_val + q.y_val * q.z_val)
        cosr_cosp = 1 - 2 * (q.x_val * q.x_val + q.y_val * q.y_val)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w_val * q.y_val - q.z_val * q.x_val)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w_val * q.z_val + q.x_val * q.y_val)
        cosy_cosp = 1 - 2 * (q.y_val * q.y_val + q.z_val * q.z_val)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]

    def run(self):
        """Main run loop"""
        rospy.loginfo("[HX AirSim Interface Python] Running... Press Ctrl+C to exit")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("[HX AirSim Interface Python] Shutting down...")
        finally:
            self.stop()

if __name__ == '__main__':
    try:
        interface = HXAirSimInterfacePython()
        interface.run()
    except rospy.ROSInterruptException:
        pass