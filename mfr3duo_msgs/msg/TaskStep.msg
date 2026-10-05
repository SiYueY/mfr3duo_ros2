# Whole-robot SDK task input. Pose is a navigation target, optional grasp pose,
# or desired resting object pose. Manipulator: 0 auto, 1 left, 2 right.
string kind
string object_id
uint8 manipulator
bool has_pose
geometry_msgs/PoseStamped pose
float64 timeout_s
# scene_joint only: object_id is the simulation fixture joint name.
float64 position
