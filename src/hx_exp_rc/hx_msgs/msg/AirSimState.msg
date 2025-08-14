# AirSim specific state information

Header header

# AirSim vehicle pose
geometry_msgs/Pose pose

# AirSim kinematics  
geometry_msgs/Vector3 linear_velocity
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 angular_acceleration

# AirSim collision info
bool collision_detected
geometry_msgs/Point collision_point
geometry_msgs/Vector3 collision_normal
string collision_object_name

# AirSim API status
bool api_control_enabled
bool vehicle_ready
float64 timestamp

# Environment info
float64 barometer_altitude
geometry_msgs/Vector3 magnetometer