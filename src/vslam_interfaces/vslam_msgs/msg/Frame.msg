# Contains the data structure of frame

# Header
std_msgs/Header header

# Camera information
sensor_msgs/CameraInfo cam_info

# Frame id
uint32 id

# Image
sensor_msgs/Image image

# Camera pose
geometry_msgs/Pose pose

# Points
Vector2d[] keypoints
bool[] keypoints_has_mp
Vector3d[] mappoints

# Pose constraints
Vector3d[] other_keyframes_pos