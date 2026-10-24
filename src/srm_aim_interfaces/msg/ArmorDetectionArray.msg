# Copy the source sensor_msgs/Image.header unchanged, including stamp/frame_id.
# stamp is the source image timestamp, never the inference completion time.
std_msgs/Header header

# Dimensions of that original image, not the network input tensor.
uint32 image_width
uint32 image_height

# All valid detections from this image. Empty means no accepted detections.
# Array position is not a persistent tracking ID.
srm_aim_interfaces/ArmorDetection[] detections
