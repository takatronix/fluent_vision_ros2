#!/bin/bash

echo "🛑 Stopping FV RealSense System..."

# Kill all fv_ related processes with various patterns
echo "🔧 Killing fv_ processes..."
pkill -9 -f "fv_" || true
pkill -9 -f "fv_realsense" || true
pkill -9 -f "fv_object_detector" || true
pkill -9 -f "fv_object_mask_generator" || true
pkill -9 -f "fv_recorder" || true
pkill -9 -f "depth_image_proc" || true
pkill -9 -f "foxglove_bridge" || true
pkill -9 -f "__node:=depth_image_proc_d405" || true
pkill -9 -f "__node:=depth_image_proc_d415" || true

# Kill topic relay processes
echo "🔧 Killing topic relay processes..."
pkill -9 -f "topic_relay.py" || true

# Also kill ros2 launch processes that might be running fv_ nodes
echo "🔧 Killing ros2 launch processes for fv_ nodes..."
pkill -9 -f "ros2 launch.*fv_" || true
pkill -9 -f "ros2 run.*fv_" || true
pkill -9 -f "ros2 run topic_tools relay" || true

# Wait a moment and check
sleep 1

# Report remaining processes
remaining=$(ps aux | grep -E "fv_|launch.*fv_" | grep -v grep | wc -l)
if [ "$remaining" -gt 0 ]; then
    echo "⚠️  Warning: $remaining processes still running:"
    ps aux | grep -E "fv_|launch.*fv_|depth_image_proc|foxglove_bridge" | grep -v grep
else
    echo "✅ All FV System processes stopped"
fi 

# 念のためノードとトピックを確認
sleep 5
ros2 node list
ros2 topic list