# Atomic 12-axis measured state from the STM32 resident feedback cache.
# Consumers must reject policy input when a required sample_age_ms is stale.
std_msgs/Header header
string[12] joint_names
float64[12] positions
uint32[12] sample_age_ms
uint16 present_mask
uint32 firmware_tick_ms
uint32 completed_pairs
