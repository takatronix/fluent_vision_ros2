# 3D object detection result per instance
std_msgs/Header header

int32 id                          # detection tracking ID
int32 class_id
string label

# Shape classification
uint8 SHAPE_UNKNOWN=0
uint8 SHAPE_CUBE=1
uint8 SHAPE_PLATE=2
uint8 SHAPE_CYLINDER=3
uint8 SHAPE_SPHERE=4
uint8 shape

# 3D pose (centroid + orientation)
geometry_msgs/Point centroid       # OBB center in camera frame [m]
geometry_msgs/Quaternion orientation # OBB orientation (PCA eigenvectors -> quaternion)

# OBB extents along principal axes [m]
geometry_msgs/Vector3 extent       # x=axis0, y=axis1, z=axis2

# Distance from camera [m]
float32 distance_m

# Dominant color (HSV-based classification)
uint8 COLOR_UNKNOWN=0
uint8 COLOR_RED=1
uint8 COLOR_ORANGE=2
uint8 COLOR_YELLOW=3
uint8 COLOR_GREEN=4
uint8 COLOR_BLUE=5
uint8 COLOR_PURPLE=6
uint8 COLOR_WHITE=7
uint8 COLOR_BLACK=8
uint8 COLOR_GRAY=9
uint8 color

# Mean RGB
uint8 mean_r
uint8 mean_g
uint8 mean_b

# Confidence & quality
float32 confidence                 # from 2D detection
uint32 num_points                  # point cloud points used

# Eigenvalue ratios for shape analysis
float32 eigen_ratio_01             # eigenvalue[0]/eigenvalue[1]
float32 eigen_ratio_02             # eigenvalue[0]/eigenvalue[2]
