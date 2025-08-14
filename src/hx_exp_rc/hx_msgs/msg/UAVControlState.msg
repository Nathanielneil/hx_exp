# HX UAV Control State Message
# Control system state information

Header header

# Control state machine
uint8 control_state
uint8 INITIALIZATION = 0
uint8 TAKEOFF = 1  
uint8 COMMAND_CONTROL = 2
uint8 HOLD = 3
uint8 LAND = 4
uint8 DISARM = 5
uint8 EMERGENCY = 6

# Current control mode
uint8 control_mode
uint8 PID_CONTROL = 1
uint8 UDE_CONTROL = 2
uint8 ADRC_CONTROL = 3

# Control system status
bool control_enable
bool trajectory_tracking
bool position_control_enable

# Current command being executed
UAVCommand current_command

# Control performance metrics
float64 position_error_norm
float64 velocity_error_norm  
float64 tracking_error

# Safety status
bool safety_mode_enable
string safety_info

# Controller parameters status
bool pid_ready
bool ude_ready
bool adrc_ready