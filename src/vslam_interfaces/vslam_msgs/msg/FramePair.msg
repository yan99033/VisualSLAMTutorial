# Constraints to be established in the feature matching and tracking

# Constraint types
uint8 FOR_INIT=0
uint8 FOR_TRACKING=1
uint8 FOR_LOOP_CLOSURE=2
uint8 FOR_RELOCALIZATION=3

# The frame pair
# frame1 is the existing/reference frame
vslam_msgs/Frame frame1
vslam_msgs/Frame frame2

# Relative transformation
geometry_msgs/Pose pose12

# The matched points
vslam_msgs/PointPair[] matches

uint8 constraint_type