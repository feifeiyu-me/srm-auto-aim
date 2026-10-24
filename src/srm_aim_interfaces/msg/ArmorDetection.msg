# Color describes the observed armor, not the identity/color of the local robot.
uint8 COLOR_UNKNOWN=0
uint8 COLOR_RED=1
uint8 COLOR_BLUE=2
uint8 color

# Detection confidence: finite and within [0, 1].
float32 confidence

# Semantic corner order: left-top, left-bottom, right-bottom, right-top.
# The publisher must map the model's annotation order to these named fields.
srm_aim_interfaces/Keypoint2D lt
srm_aim_interfaces/Keypoint2D lb
srm_aim_interfaces/Keypoint2D rb
srm_aim_interfaces/Keypoint2D rt
