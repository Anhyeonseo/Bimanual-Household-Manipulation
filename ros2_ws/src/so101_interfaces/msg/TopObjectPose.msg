# Board-relative observation only. This is never a robot motion target.
std_msgs/Header header
float64 x_m
float64 y_m
float64 yaw_rad
float32 confidence
float32 frame_age_s
# Source-image coordinates are part of the observation contract so an upper
# application can route work by the camera image, without inferring image
# left/right from the calibrated board axes.
float32 center_x_px
float32 center_y_px
uint32 image_width_px
uint32 image_height_px
bool footprint_inside
bool image_fully_visible
bool motion_authorized
bool robot_target_available
string detector_status
