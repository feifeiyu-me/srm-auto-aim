# Index in this source frame; not a persistent tracking ID.
uint32 input_index
srm_aim_interfaces/ArmorDetection detection
bool digit_check_enabled
bool digit_verified
# 0 means unknown/unverified; 2 only after template verification passes.
uint8 digit
# Normalized template correlation, NOT a probability; -1 if not evaluated.
float32 digit_score
bool pose_valid
bool ambiguous
string reason
# Object-to-optical-camera pose; use array.optical_frame_id, not source frame_id.
geometry_msgs/Pose pose
# Metres: axial t_z versus Euclidean norm(tvec). Zero when invalid.
float64 axial_depth_m
float64 distance_m
# Pixel RMS; -1 when unavailable.
float64 reprojection_error_px
