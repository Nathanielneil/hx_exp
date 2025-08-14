# Controller output message

Header header

# Control output type
uint8 controller_type
uint8 PID_OUTPUT = 1
uint8 UDE_OUTPUT = 2  
uint8 ADRC_OUTPUT = 3

# Attitude control commands [rad, rad, rad/s, normalized_throttle]
float64 roll_command
float64 pitch_command  
float64 yaw_rate_command
float64 throttle_command

# Position control commands [m]
geometry_msgs/Point position_command

# Velocity control commands [m/s]
geometry_msgs/Vector3 velocity_command

# Control effort and performance
float64 control_effort_norm
float64 position_error_norm
float64 velocity_error_norm

# Controller internal states
float64[] controller_states