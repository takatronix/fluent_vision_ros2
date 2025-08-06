#!/bin/bash

# FV Face Recognizer Node Startup Script
echo "🚀 Starting FV Face Recognizer Node..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the face recognizer node with correct input topic
ros2 launch fv_face_recognizer fv_face_recognizer_launch.py input_topic:=/color/image_raw