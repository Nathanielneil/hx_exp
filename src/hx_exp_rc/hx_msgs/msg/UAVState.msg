# HX UAV State Message
# Complete state information for UAV

Header header

# Position [ENU frame, m]
float64[3] position

# Velocity [ENU frame, m/s]  
float64[3] velocity

# Acceleration [ENU frame, m/s^2]
float64[3] acceleration

# Attitude quaternion
geometry_msgs/Quaternion attitude_q

# Attitude euler angles [rad]
float64[3] attitude

# Angular velocity [body frame, rad/s]
float64[3] angular_velocity

# Connection status
bool connected
bool armed
bool odom_valid
bool gps_status

# Battery and system
float64 battery_voltage
float64 battery_percentage

# Flight mode
string mode

# Time stamp
float64 time_from_start