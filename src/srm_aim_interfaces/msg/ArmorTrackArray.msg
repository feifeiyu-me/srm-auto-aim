# Original source image Header, NOT processing completion time.
std_msgs/Header header
uint32 image_width
uint32 image_height
# Explicit frame for all positions and velocities; first version fixed optical camera.
string tracking_frame_id
bool self_motion_compensated
# Changes on reset; unique process prefix. IDs must be interpreted with this session.
string session_id
string reason
srm_aim_interfaces/ArmorTrack[] tracks
