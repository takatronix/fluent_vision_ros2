std_msgs/Header header
int32 id
geometry_msgs/Point root_camera
geometry_msgs/Point tip_camera
geometry_msgs/Vector3 dir_camera
float32 length_m
float32 confidence

# Optional world-frame outputs (v4.1+, may be NaN if TF unavailable)
geometry_msgs/Point root_world
geometry_msgs/Point tip_world
geometry_msgs/Vector3 dir_world


