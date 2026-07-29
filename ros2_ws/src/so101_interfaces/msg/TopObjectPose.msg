# Board-relative observation only. This is never a robot motion target.
std_msgs/Header header
float64 x_m
float64 y_m
float64 yaw_rad
float32 confidence
float32 frame_age_s
bool footprint_inside
bool image_fully_visible
bool motion_authorized
bool robot_target_available
string detector_status
