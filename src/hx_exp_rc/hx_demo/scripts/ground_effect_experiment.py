#!/usr/bin/env python3

import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import time
from datetime import datetime
from hx_msgs.msg import UAVCommand, UAVState, UAVControlState, ControllerOutput
from geometry_msgs.msg import Point, Vector3

class GroundEffectExperiment:
    def __init__(self):
        rospy.init_node('ground_effect_experiment', anonymous=True)
        
        # Parameters (can be set via rosparam)
        self.takeoff_height = rospy.get_param('~takeoff_height', 3.0)
        self.landing_speed = rospy.get_param('~landing_speed', 0.5)
        self.hover_duration = rospy.get_param('~hover_duration', 5.0)
        self.data_dir = rospy.get_param('~data_dir', 'ground_effect_data')
        self.auto_repeat = rospy.get_param('~auto_repeat', True)
        self.controllers_to_test = rospy.get_param('~controllers', [1, 2, 3])  # PID, UDE, ADRC
        
        # Create data directory
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        
        # Publishers
        self.command_pub = rospy.Publisher('/Drone1/hx_uav/command', UAVCommand, queue_size=10)
        
        # Subscribers
        self.state_sub = rospy.Subscriber('/Drone1/hx_uav/state', UAVState, self.state_callback)
        self.control_state_sub = rospy.Subscriber('/Drone1/hx_uav/control_state', UAVControlState, self.control_state_callback)
        self.controller_output_sub = rospy.Subscriber('/Drone1/hx_uav/controller_output', ControllerOutput, self.controller_output_callback)
        
        # State variables
        self.current_state = None
        self.current_control_state = None
        self.current_controller_output = None
        self.takeoff_position = None
        
        # Experiment state
        self.experiment_phase = 0
        self.phase_start_time = None
        self.current_controller_idx = 0
        self.current_controller = self.controllers_to_test[0]
        
        # Data recording
        self.experiment_data = []
        self.controller_results = {}
        
        # Timer for experiment execution
        self.experiment_timer = rospy.Timer(rospy.Duration(0.05), self.experiment_timer_callback)  # 20Hz
        
        rospy.loginfo(f"[Ground Effect Experiment] Initialized")
        rospy.loginfo(f"[Ground Effect Experiment] Parameters:")
        rospy.loginfo(f"  - Takeoff Height: {self.takeoff_height} m")
        rospy.loginfo(f"  - Landing Speed: {self.landing_speed} m/s") 
        rospy.loginfo(f"  - Hover Duration: {self.hover_duration} s")
        rospy.loginfo(f"  - Controllers: {[self.get_controller_name(c) for c in self.controllers_to_test]}")
        rospy.loginfo(f"  - Data Directory: {self.data_dir}")
        
        # Wait for connections
        rospy.sleep(2.0)

    def state_callback(self, msg):
        self.current_state = msg
        
        # Record takeoff position
        if self.takeoff_position is None and msg.connected:
            self.takeoff_position = [msg.position[0], msg.position[1], msg.position[2]]
            rospy.loginfo(f"[Experiment] Takeoff position: {self.takeoff_position}")

    def control_state_callback(self, msg):
        self.current_control_state = msg

    def controller_output_callback(self, msg):
        self.current_controller_output = msg

    def experiment_timer_callback(self, event):
        if not self.current_state or not self.current_control_state:
            return
        
        current_time = rospy.Time.now()
        
        # Record data during landing phase
        if self.experiment_phase == 2:
            self.record_experiment_data(current_time)
        
        if self.experiment_phase == 0:
            self.handle_takeoff_phase(current_time)
        elif self.experiment_phase == 1:
            self.handle_hover_phase(current_time) 
        elif self.experiment_phase == 2:
            self.handle_landing_phase(current_time)
        elif self.experiment_phase == 3:
            self.handle_analysis_phase(current_time)
        elif self.experiment_phase == 4:
            self.handle_completion_phase()

    def handle_takeoff_phase(self, current_time):
        if self.phase_start_time is None:
            controller_name = self.get_controller_name(self.current_controller)
            rospy.loginfo(f"[Experiment] Phase 1: Takeoff with {controller_name} controller")
            self.send_takeoff_command()
            self.phase_start_time = current_time
        elif ((current_time - self.phase_start_time).to_sec() > 5.0 and 
              self.current_control_state.control_state == UAVControlState.COMMAND_CONTROL and
              self.current_state.position[2] >= self.takeoff_height - 0.5):
            self.experiment_phase = 1
            self.phase_start_time = None

    def handle_hover_phase(self, current_time):
        if self.phase_start_time is None:
            rospy.loginfo("[Experiment] Phase 2: Hover stabilization")
            self.send_hover_command()
            self.phase_start_time = current_time
        elif (current_time - self.phase_start_time).to_sec() > self.hover_duration:
            self.experiment_phase = 2
            self.phase_start_time = None
            self.experiment_data = []  # Clear data for new landing test

    def handle_landing_phase(self, current_time):
        if self.phase_start_time is None:
            controller_name = self.get_controller_name(self.current_controller)
            rospy.loginfo(f"[Experiment] Phase 3: Landing test with {controller_name} controller")
            rospy.loginfo(f"[Experiment] Landing speed: {self.landing_speed} m/s")
            self.phase_start_time = current_time
        
        # Calculate target height based on landing speed
        elapsed_time = (current_time - self.phase_start_time).to_sec()
        target_height = self.takeoff_height - (self.landing_speed * elapsed_time)
        
        if target_height <= 0.3 or self.current_state.position[2] <= 0.3:
            # Landing complete
            rospy.loginfo("[Experiment] Landing complete")
            self.send_land_command()
            self.experiment_phase = 3
            self.phase_start_time = None
        else:
            # Continue controlled descent
            self.send_move_command(self.takeoff_position[0], self.takeoff_position[1], target_height)

    def handle_analysis_phase(self, current_time):
        if self.phase_start_time is None:
            rospy.loginfo("[Experiment] Phase 4: Analyzing data...")
            self.analyze_current_experiment()
            self.phase_start_time = current_time
        elif (current_time - self.phase_start_time).to_sec() > 3.0:
            # Switch to next controller
            self.current_controller_idx += 1
            
            if self.current_controller_idx < len(self.controllers_to_test) and self.auto_repeat:
                self.current_controller = self.controllers_to_test[self.current_controller_idx]
                rospy.loginfo(f"[Experiment] Switching to controller {self.current_controller_idx + 1}/{len(self.controllers_to_test)}")
                self.experiment_phase = 0  # Restart with next controller
                self.phase_start_time = None
            else:
                self.experiment_phase = 4  # All controllers tested

    def handle_completion_phase(self):
        rospy.loginfo("[Experiment] All controllers tested! Generating final report...")
        self.generate_final_report()
        rospy.signal_shutdown("Experiment completed")

    def send_takeoff_command(self):
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.Agent_CMD = UAVCommand.Takeoff
        cmd.Control_mode = self.current_controller
        cmd.Command_ID = "ground_effect_takeoff"
        self.command_pub.publish(cmd)

    def send_hover_command(self):
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.Agent_CMD = UAVCommand.Hold
        cmd.Control_mode = self.current_controller
        cmd.Command_ID = "ground_effect_hover"
        self.command_pub.publish(cmd)

    def send_move_command(self, x, y, z):
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.Agent_CMD = UAVCommand.Move_ENU
        cmd.Control_mode = self.current_controller
        
        cmd.position.x = x
        cmd.position.y = y
        cmd.position.z = z
        
        # Set downward velocity for controlled descent
        cmd.linear_vel.x = 0.0
        cmd.linear_vel.y = 0.0
        cmd.linear_vel.z = -self.landing_speed
        
        cmd.Command_ID = "ground_effect_landing"
        self.command_pub.publish(cmd)

    def send_land_command(self):
        cmd = UAVCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.Agent_CMD = UAVCommand.Land
        cmd.Command_ID = "ground_effect_land"
        self.command_pub.publish(cmd)

    def record_experiment_data(self, current_time):
        if not self.current_controller_output or not self.phase_start_time:
            return
        
        # Calculate target height
        elapsed_time = (current_time - self.phase_start_time).to_sec()
        target_height = self.takeoff_height - (self.landing_speed * elapsed_time)
        
        # Record data point
        data_point = {
            'timestamp': current_time.to_sec(),
            'elapsed_time': elapsed_time,
            'controller': self.get_controller_name(self.current_controller),
            'controller_id': self.current_controller,
            
            # Height data
            'target_height': target_height,
            'actual_height': self.current_state.position[2],
            'height_error': abs(self.current_state.position[2] - target_height),
            
            # Position data
            'target_x': self.takeoff_position[0],
            'target_y': self.takeoff_position[1],
            'actual_x': self.current_state.position[0],
            'actual_y': self.current_state.position[1],
            
            # Velocity data
            'velocity_x': self.current_state.velocity[0],
            'velocity_y': self.current_state.velocity[1],
            'velocity_z': self.current_state.velocity[2],
            'target_velocity_z': -self.landing_speed,
            'velocity_z_error': abs(self.current_state.velocity[2] - (-self.landing_speed)),
            
            # Control commands
            'roll_cmd': self.current_controller_output.roll_command,
            'pitch_cmd': self.current_controller_output.pitch_command,
            'yaw_rate_cmd': self.current_controller_output.yaw_rate_command,
            'throttle_cmd': self.current_controller_output.throttle_command,
        }
        
        # Calculate derived metrics
        data_point['horizontal_drift'] = np.sqrt((data_point['actual_x'] - data_point['target_x'])**2 + 
                                               (data_point['actual_y'] - data_point['target_y'])**2)
        
        self.experiment_data.append(data_point)

    def analyze_current_experiment(self):
        if not self.experiment_data:
            rospy.logwarn("[Experiment] No data collected for analysis")
            return
        
        df = pd.DataFrame(self.experiment_data)
        controller_name = self.get_controller_name(self.current_controller)
        
        # Calculate key metrics
        metrics = {
            'controller': controller_name,
            'data_points': len(df),
            'flight_duration': df['elapsed_time'].max(),
            
            # Position metrics
            'avg_horizontal_drift': df['horizontal_drift'].mean(),
            'max_horizontal_drift': df['horizontal_drift'].max(),
            'final_landing_error': df['horizontal_drift'].iloc[-1],
            
            # Height control metrics
            'avg_height_error': df['height_error'].mean(),
            'max_height_error': df['height_error'].max(),
            'height_error_std': df['height_error'].std(),
            
            # Velocity control metrics
            'avg_velocity_error': df['velocity_z_error'].mean(),
            'max_velocity_error': df['velocity_z_error'].max(),
            'velocity_error_std': df['velocity_z_error'].std(),
            
            # Control stability metrics
            'roll_cmd_std': df['roll_cmd'].std(),
            'pitch_cmd_std': df['pitch_cmd'].std(),
            'throttle_cmd_std': df['throttle_cmd'].std(),
        }
        
        # Ground effect analysis (focus on low altitude)
        low_alt_data = df[df['actual_height'] <= 1.5]  # Ground effect typically below 1.5m
        if not low_alt_data.empty:
            metrics.update({
                'ground_effect_horizontal_drift': low_alt_data['horizontal_drift'].mean(),
                'ground_effect_height_error': low_alt_data['height_error'].mean(),
                'ground_effect_velocity_error': low_alt_data['velocity_z_error'].mean(),
                'ground_effect_roll_variation': low_alt_data['roll_cmd'].std(),
                'ground_effect_pitch_variation': low_alt_data['pitch_cmd'].std(),
            })
        
        # Store results
        self.controller_results[controller_name] = metrics
        
        # Save data to file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_filename = f"{self.data_dir}/{controller_name}_landing_data_{timestamp}.csv"
        df.to_csv(data_filename, index=False)
        
        # Print summary
        rospy.loginfo(f"=== {controller_name} Controller Ground Effect Analysis ===")
        rospy.loginfo(f"Data Points Collected: {metrics['data_points']}")
        rospy.loginfo(f"Average Horizontal Drift: {metrics['avg_horizontal_drift']:.3f} m")
        rospy.loginfo(f"Maximum Horizontal Drift: {metrics['max_horizontal_drift']:.3f} m") 
        rospy.loginfo(f"Final Landing Error: {metrics['final_landing_error']:.3f} m")
        rospy.loginfo(f"Average Height Error: {metrics['avg_height_error']:.3f} m")
        rospy.loginfo(f"Average Velocity Error: {metrics['avg_velocity_error']:.3f} m/s")
        rospy.loginfo(f"Control Stability (Roll/Pitch STD): {metrics['roll_cmd_std']:.3f}/{metrics['pitch_cmd_std']:.3f} rad")
        rospy.loginfo(f"Data saved to: {data_filename}")

    def generate_final_report(self):
        if not self.controller_results:
            rospy.logwarn("[Experiment] No results to analyze")
            return
        
        # Create comprehensive report
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_filename = f"{self.data_dir}/ground_effect_report_{timestamp}.txt"
        
        with open(report_filename, 'w') as f:
            f.write("Ground Effect Experiment Final Report\n")
            f.write("=====================================\n\n")
            
            f.write("Experiment Parameters:\n")
            f.write(f"- Takeoff Height: {self.takeoff_height} m\n")
            f.write(f"- Landing Speed: {self.landing_speed} m/s\n")
            f.write(f"- Hover Duration: {self.hover_duration} s\n")
            f.write(f"- Controllers Tested: {[self.get_controller_name(c) for c in self.controllers_to_test]}\n\n")
            
            # Individual controller results
            for controller, results in self.controller_results.items():
                f.write(f"{controller} Controller Results:\n")
                f.write("-" * 40 + "\n")
                for key, value in results.items():
                    if isinstance(value, float):
                        f.write(f"{key}: {value:.4f}\n")
                    else:
                        f.write(f"{key}: {value}\n")
                f.write("\n")
            
            # Comparative analysis
            f.write("Comparative Analysis:\n")
            f.write("-" * 40 + "\n")
            
            # Rank controllers by different metrics
            controllers = list(self.controller_results.keys())
            
            # Best horizontal stability (lowest drift)
            best_drift = min(controllers, key=lambda x: self.controller_results[x]['avg_horizontal_drift'])
            f.write(f"Best Horizontal Stability: {best_drift}\n")
            
            # Best height control
            best_height = min(controllers, key=lambda x: self.controller_results[x]['avg_height_error'])
            f.write(f"Best Height Control: {best_height}\n")
            
            # Best velocity control
            best_velocity = min(controllers, key=lambda x: self.controller_results[x]['avg_velocity_error'])
            f.write(f"Best Velocity Control: {best_velocity}\n")
            
            # Most stable control output
            best_stability = min(controllers, key=lambda x: (self.controller_results[x]['roll_cmd_std'] + 
                                                           self.controller_results[x]['pitch_cmd_std']))
            f.write(f"Most Stable Control Output: {best_stability}\n")
            
            # Best final landing accuracy
            best_landing = min(controllers, key=lambda x: self.controller_results[x]['final_landing_error'])
            f.write(f"Best Final Landing Accuracy: {best_landing}\n")
        
        # Generate plots
        self.generate_comparison_plots(timestamp)
        
        rospy.loginfo(f"[Experiment] Final report saved to: {report_filename}")
        rospy.loginfo("[Experiment] Check plots in the data directory for visual analysis")

    def generate_comparison_plots(self, timestamp):
        try:
            import matplotlib.pyplot as plt
            plt.style.use('seaborn-v0_8')
            
            controllers = list(self.controller_results.keys())
            
            # Create comparison plots
            fig, axes = plt.subplots(2, 3, figsize=(15, 10))
            fig.suptitle('Ground Effect Experiment - Controller Comparison', fontsize=16)
            
            # 1. Average Horizontal Drift
            drift_values = [self.controller_results[c]['avg_horizontal_drift'] for c in controllers]
            axes[0,0].bar(controllers, drift_values, color=['blue', 'green', 'red'])
            axes[0,0].set_title('Average Horizontal Drift')
            axes[0,0].set_ylabel('Drift (m)')
            
            # 2. Average Height Error  
            height_values = [self.controller_results[c]['avg_height_error'] for c in controllers]
            axes[0,1].bar(controllers, height_values, color=['blue', 'green', 'red'])
            axes[0,1].set_title('Average Height Error')
            axes[0,1].set_ylabel('Error (m)')
            
            # 3. Average Velocity Error
            vel_values = [self.controller_results[c]['avg_velocity_error'] for c in controllers]
            axes[0,2].bar(controllers, vel_values, color=['blue', 'green', 'red'])
            axes[0,2].set_title('Average Velocity Error')
            axes[0,2].set_ylabel('Error (m/s)')
            
            # 4. Control Stability (Roll)
            roll_values = [self.controller_results[c]['roll_cmd_std'] for c in controllers]
            axes[1,0].bar(controllers, roll_values, color=['blue', 'green', 'red'])
            axes[1,0].set_title('Roll Command Stability (STD)')
            axes[1,0].set_ylabel('Standard Deviation (rad)')
            
            # 5. Control Stability (Pitch)
            pitch_values = [self.controller_results[c]['pitch_cmd_std'] for c in controllers]
            axes[1,1].bar(controllers, pitch_values, color=['blue', 'green', 'red'])
            axes[1,1].set_title('Pitch Command Stability (STD)')
            axes[1,1].set_ylabel('Standard Deviation (rad)')
            
            # 6. Final Landing Accuracy
            landing_values = [self.controller_results[c]['final_landing_error'] for c in controllers]
            axes[1,2].bar(controllers, landing_values, color=['blue', 'green', 'red'])
            axes[1,2].set_title('Final Landing Accuracy')
            axes[1,2].set_ylabel('Landing Error (m)')
            
            plt.tight_layout()
            plt.savefig(f"{self.data_dir}/comparison_plots_{timestamp}.png", dpi=300, bbox_inches='tight')
            plt.close()
            
            rospy.loginfo("[Experiment] Comparison plots saved")
            
        except ImportError:
            rospy.logwarn("[Experiment] matplotlib not available, skipping plots")
        except Exception as e:
            rospy.logwarn(f"[Experiment] Failed to generate plots: {e}")

    def get_controller_name(self, controller_id):
        controller_names = {1: "PID", 2: "UDE", 3: "ADRC"}
        return controller_names.get(controller_id, f"Controller_{controller_id}")

    def run(self):
        rospy.loginfo("[Experiment] Ground Effect Experiment started")
        rospy.loginfo("[Experiment] Press Ctrl+C to stop")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("[Experiment] Experiment interrupted by user")
        finally:
            if self.controller_results:
                rospy.loginfo("[Experiment] Generating partial results...")
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.generate_comparison_plots(timestamp)

if __name__ == '__main__':
    try:
        experiment = GroundEffectExperiment()
        experiment.run()
    except rospy.ROSInterruptException:
        pass