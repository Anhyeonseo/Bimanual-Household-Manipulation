# Non-actionable base-frame shadow pose. Never consume this as a motion goal.
std_msgs/Header header
float64 x_m
float64 y_m
float64 z_m
float64 yaw_rad
float32 source_frame_age_s
float32 confidence
bool source_footprint_inside
bool source_image_fully_visible
bool shadow_pose_available
bool transform_validated
bool inside_workspace
bool fresh
bool motion_authorized
bool robot_target_available
string status
