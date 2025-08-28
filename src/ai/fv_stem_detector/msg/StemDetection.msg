int32 region_id
int32 x
int32 y
int32 w
int32 h
bool detected

# YOLO class and score
int32 class_id
float32 yolo_score

# 3D camera-frame coordinates (legacy quick fields; root cut-point when available)
float32 camera_x
float32 camera_y
float32 camera_z

# v4.1 additions (backward-compatible)
float32 world_x
float32 world_y
float32 world_z
bool visible
builtin_interfaces/Time observed_at

# source flag (single-topic multiplexing)
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_FIXED=1
uint8 SOURCE_YOLO=2
uint8 source

# Approach (2D/3D) and distances (camera_z based where possible)
int32 approach_px_x
int32 approach_px_y
geometry_msgs/Point approach_camera
float32 approach_distance_m

# Root (cut-point)
int32 root_px_x
int32 root_px_y
geometry_msgs/Point root_camera
float32 root_distance_m

# Tip (optional; NaN/-1 if unavailable in 2D path)
int32 tip_px_x
int32 tip_px_y
geometry_msgs/Point tip_camera
float32 tip_distance_m

# PCA / geometry (optional)
float32 pca_score
float32[3] pca_eigenvalues
geometry_msgs/Vector3 axis_camera
float32 length_m
float32 curvature
float32 diameter_m

# Bones (up to 5 points; unused entries are -1/NaN)
int32[5] bones_px_x
int32[5] bones_px_y
geometry_msgs/Point[5] bones_camera

# ROI stats and performance
int32 point_count
float32 roi_ms
float32 frame_total_ms


