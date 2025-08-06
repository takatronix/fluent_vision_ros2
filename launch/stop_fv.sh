#!/bin/bash

echo "🛑 Stopping FV RealSense System..."

# Kill all fv_ related processes with various patterns
echo "🔧 Killing fv_ processes..."
pkill -9 -f "fv_" || true
pkill -9 -f "fv_realsense" || true
pkill -9 -f "fv_object_detector" || true
pkill -9 -f "fv_object_mask_generator" || true
pkill -9 -f "fv_recorder" || true

# Also kill ros2 launch processes that might be running fv_ nodes
echo "🔧 Killing ros2 launch processes for fv_ nodes..."
pkill -9 -f "ros2 launch.*fv_" || true
pkill -9 -f "ros2 run.*fv_" || true

# Wait a moment and check
sleep 1

# Report remaining processes
remaining=$(ps aux | grep -E "fv_|launch.*fv_" | grep -v grep | wc -l)
if [ "$remaining" -gt 0 ]; then
    echo "⚠️  Warning: $remaining processes still running:"
    ps aux | grep -E "fv_|launch.*fv_" | grep -v grep
else
    echo "✅ All FV System processes stopped"
fi 