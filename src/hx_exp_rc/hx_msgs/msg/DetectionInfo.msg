# Detection information for vision-based control

Header header

# Detection status
bool detected
float64 detection_confidence

# Object position in image [pixels]
int32 image_x
int32 image_y
int32 image_width  
int32 image_height

# Object position in world frame [m]
geometry_msgs/Point world_position

# Detection type
string object_type
int32 object_id

# Relative position to UAV [body frame, m]
geometry_msgs/Point relative_position