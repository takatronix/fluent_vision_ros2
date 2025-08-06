#!/bin/bash

echo "🔍 Camera Identification Test"
echo "="*50
echo ""
echo "This script will flash the IR projector of each camera"
echo "to help you physically identify which is which."
echo ""

# Source ROS2
if [ -f "/home/aspara/seedbox-r1/fluent_vision_ros2/install/setup.bash" ]; then
    source /home/aspara/seedbox-r1/fluent_vision_ros2/install/setup.bash
elif [ -f "/home/aspara/seedbox-r1/fluent_vision_ros2/src/sensors/fv_realsense/install/setup.bash" ]; then
    source /home/aspara/seedbox-r1/fluent_vision_ros2/src/sensors/fv_realsense/install/setup.bash
fi

# Kill any existing processes
pkill -f "fv_realsense_node" || true
sleep 2

echo "📷 Testing D415 (Serial: 237322061056)..."
echo "Look for the camera with ACTIVE IR projector pattern"
echo ""

ros2 run fv_realsense fv_realsense_node \
    --ros-args \
    --params-file /home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_realsense_d415.yaml \
    -r __node:=test_d415 &

PID=$!
sleep 5

echo "✅ D415 should be showing IR pattern now"
echo "⏸️  Press Enter when you've identified D415..."
read

kill $PID 2>/dev/null
sleep 2

echo ""
echo "📷 Testing D405 (Serial: 218622272880)..."
echo "Look for the camera with ACTIVE IR projector pattern"
echo ""

ros2 run fv_realsense fv_realsense_node \
    --ros-args \
    --params-file /home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_realsense_d405.yaml \
    -r __node:=test_d405 &

PID=$!
sleep 5

echo "✅ D405 should be showing IR pattern now"
echo "⏸️  Press Enter when you've identified D405..."
read

kill $PID 2>/dev/null

echo ""
echo "✅ Test complete!"
echo ""
echo "Summary:"
echo "- D415 (Serial: 237322061056) - Depth scale: 0.001 (1mm)"
echo "- D405 (Serial: 218622272880) - Depth scale: 0.0001 (0.1mm)"