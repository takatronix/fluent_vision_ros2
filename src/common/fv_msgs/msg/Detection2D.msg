std_msgs/Header header

int32 id
# Stable detection / tracking ID. May be greater than 255.
uint8 SOURCE_OBJECT=1
uint8 SOURCE_INSTANCE=2
uint8 SOURCE_SEMANTIC=4
uint8 source_mask

int32 class_id
string label
float32 conf_fused
float32 conf_object
float32 conf_instance
float32 conf_semantic

geometry_msgs/Point32 bbox_min
geometry_msgs/Point32 bbox_max

uint32 mask_instance_id
# Per-frame key into the paired instance mask image.
# For MONO8 mask images this must be 1..255 and equal to the pixel value.
# 0 means no instance-specific mask key.
uint32 mask_semantic_id

float32 depth_hint_m
builtin_interfaces/Time observed_at
