uint8 TENTATIVE=0
uint8 CONFIRMED=1
uint8 COASTING=2
uint8 DELETED=3
uint64 track_id
uint8 state
string reason
# Time of this filtered/predicted state; may predate array Header for deletion events.
builtin_interfaces/Time state_stamp
builtin_interfaces/Time last_observation_stamp
# False for deletion events or expired coasting states.
bool state_valid
geometry_msgs/Point position
# Camera-relative velocity in m/s, NOT world velocity.
geometry_msgs/Vector3 velocity
bool velocity_valid
# Row-major covariance of [x,y,z,vx,vy,vz], units m and m/s.
float64[36] covariance
bool matched_this_frame
# Only meaningful when matched_this_frame is true; never a persistent ID.
uint32 input_index
uint8 color
float32 detection_confidence
bool digit_check_enabled
bool digit_verified
uint8 digit
bool prediction_valid
builtin_interfaces/Time prediction_stamp
geometry_msgs/Point predicted_position
