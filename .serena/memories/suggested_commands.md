# FluentVision ROS2 Suggested Commands

## Build Commands
```bash
# Full build
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build specific category
colcon build --packages-select fv_realsense
colcon build --packages-select fv_face_recognizer
colcon build --packages-select fv_mjpeg_server fv_websocket_server fv_image_distributor fv_rtmp_server

# Clean build
rm -rf build/ install/ log/
colcon build

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Environment Setup
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace
source install/setup.bash
```

## Running Nodes
```bash
# RealSense camera
ros2 launch fv_realsense fv_realsense_launch.py

# MJPEG server
ros2 run fv_mjpeg_server fv_mjpeg_server_node

# WebSocket server
ros2 run fv_websocket_server fv_websocket_server_node

# HTTP distributor
ros2 run fv_image_distributor fv_image_distributor_node

# RTMP server
ros2 run fv_rtmp_server fv_rtmp_server_node
```

## Debugging Commands
```bash
# List RealSense cameras
python3 src/fv_realsense/scripts/list_cameras.py

# Check RealSense devices
rs-enumerate-devices

# Check USB devices
lsusb | grep RealSense

# Check library versions
dpkg -l | grep realsense

# Monitor topics
ros2 topic list
ros2 topic echo <topic_name>
ros2 topic hz <topic_name>

# Check node info
ros2 node list
ros2 node info <node_name>

# Network port checking
sudo netstat -tlnp | grep :<port>
```

## Git Commands
```bash
# Check status
git status

# Stage changes
git add .

# Commit
git commit -m "message"

# Push
git push
```

## System Utilities
```bash
# List files
ls -la

# Find files
find . -name "*.cpp"

# Search in files
grep -r "pattern" .

# Process management
ps aux | grep <process>
pkill -f <process_name>
```