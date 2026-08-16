# Generic calibrated OBB observation for upper applications.
# This message is perception only and never grants motion authority.
std_msgs/Header header
string class_name
uint32 detection_count
float32 confidence
float32 frame_age_s
float64 center_x_m
float64 center_y_m
float64 long_axis_yaw_rad
float64 image_long_axis_yaw_rad
float64 major_axis_m
float64 minor_axis_m
float32 center_x_px
float32 center_y_px
uint32 image_width_px
uint32 image_height_px
bool footprint_inside
bool image_fully_visible
string yaw_semantics
string endpoint_semantics
bool motion_authorized
