#!/usr/bin/env python3
import pyrealsense2 as rs

def main():
    print("🔍 Checking RealSense camera serial numbers...")
    print("")
    
    ctx = rs.context()
    devices = ctx.query_devices()
    
    print(f"Found {len(devices)} RealSense device(s):")
    print("")
    
    for i, device in enumerate(devices):
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        firmware = device.get_info(rs.camera_info.firmware_version)
        
        print(f"Device {i}:")
        print(f"  Name: {name}")
        print(f"  Serial: {serial}")
        print(f"  Firmware: {firmware}")
        
        # Determine camera type
        if "D415" in name:
            print(f"  ➡️  This is D415, serial should be in fv_realsense_d415.yaml")
        elif "D405" in name:
            print(f"  ➡️  This is D405, serial should be in fv_realsense_d405.yaml")
        print("")
    
    print("=" * 50)
    print("")
    print("Current configuration:")
    print("")
    
    # Check current config
    import yaml
    
    try:
        with open('/home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_realsense_d415.yaml', 'r') as f:
            d415_config = yaml.safe_load(f)
            d415_serial = d415_config['/**']['ros__parameters']['camera_selection']['serial_number']
            print(f"D415 config uses serial: {d415_serial}")
    except Exception as e:
        print(f"Error reading D415 config: {e}")
    
    try:
        with open('/home/aspara/seedbox-r1/fluent_vision_ros2/launch/fv_realsense_d405.yaml', 'r') as f:
            d405_config = yaml.safe_load(f)
            d405_serial = d405_config['/**']['ros__parameters']['camera_selection']['serial_number']
            print(f"D405 config uses serial: {d405_serial}")
    except Exception as e:
        print(f"Error reading D405 config: {e}")

if __name__ == "__main__":
    main()