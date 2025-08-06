#!/bin/bash

echo "🔍 Detecting RealSense camera serial numbers..."

# Function to get camera info
get_camera_info() {
    # Run a temporary node to detect cameras
    timeout 5 ros2 run realsense2_camera realsense2_camera_node 2>&1 | grep -E "Device.*D4[01]5.*SN:" | head -2
}

# Try using the fv_realsense node instead
echo "Using FV RealSense node to detect cameras..."

# Create a temporary detection script
cat > /tmp/detect_cameras.py << 'EOF'
#!/usr/bin/env python3
import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()

print(f"Found {len(devices)} RealSense device(s):")
for i, device in enumerate(devices):
    name = device.get_info(rs.camera_info.name)
    serial = device.get_info(rs.camera_info.serial_number)
    print(f"Device {i}: {name} (SN: {serial})")
    
    if "D415" in name:
        print(f"  -> D415 serial: {serial}")
    elif "D405" in name:
        print(f"  -> D405 serial: {serial}")
EOF

# Run the detection script
if command -v python3 >/dev/null 2>&1 && python3 -c "import pyrealsense2" 2>/dev/null; then
    echo "Using pyrealsense2 to detect cameras..."
    python3 /tmp/detect_cameras.py
else
    echo "⚠️  pyrealsense2 not available, using ROS node..."
    
    # Run a test launch to get camera info
    timeout 5 ros2 launch fv_realsense fv_realsense_launch.py \
        node_name:=test_detect \
        config_file:=/home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_realsense_d415.yaml 2>&1 | \
        grep -E "Device [0-9]+:.*D4[01]5.*SN:" | head -2
fi

echo ""
echo "📝 To update the configuration files with new serial numbers:"
echo ""
echo "1. Edit fv_realsense_d415.yaml:"
echo "   serial_number: \"<D415_SERIAL_HERE>\""
echo ""
echo "2. Edit fv_realsense_d405.yaml:"
echo "   serial_number: \"<D405_SERIAL_HERE>\""
echo ""
echo "Current serial numbers in config:"
echo "D415: $(grep serial_number /home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_realsense_d415.yaml | awk -F'"' '{print $2}')"
echo "D405: $(grep serial_number /home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_realsense_d405.yaml | awk -F'"' '{print $2}')"

# Clean up
rm -f /tmp/detect_cameras.py