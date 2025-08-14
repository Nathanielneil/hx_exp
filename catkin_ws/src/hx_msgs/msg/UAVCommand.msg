# HX UAV Command Message
# Based on Prometheus UAVCommand structure

Header header

# Agent command type
uint8 Agent_CMD
uint8 Init_Pos_Hover = 1
uint8 Move_ENU = 2  
uint8 Move_Body = 3
uint8 Hold = 4
uint8 Land = 5
uint8 Disarm = 6
uint8 Trajectory_Tracking = 7
uint8 Takeoff = 8

# Move command parameters
geometry_msgs/Point position          # Position command [ENU frame, m]
geometry_msgs/Vector3 linear_vel      # Linear velocity [m/s]  
geometry_msgs/Vector3 angular_vel     # Angular velocity [rad/s]
float64 yaw_rate                      # Yaw rate command [rad/s]

# Trajectory tracking
TrajectoryPoint[] trajectory          # Trajectory points
uint32 trajectory_flag               # Trajectory tracking mode
uint8 Circle_trajectory = 1
uint8 Eight_trajectory = 2  
uint8 Step_trajectory = 3
uint8 Line_trajectory = 4

# Controller selection
uint8 Control_mode
uint8 PID_CONTROL = 1
uint8 UDE_CONTROL = 2
uint8 ADRC_CONTROL = 3

# Command source identification
string Command_ID