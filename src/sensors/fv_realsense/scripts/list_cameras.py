#!/usr/bin/env python3

"""
RealSense Camera List Utility
Lists all connected RealSense cameras and their properties
"""

import pyrealsense2 as rs
import sys


def list_cameras():
    """List all connected RealSense cameras"""
    print("🔍 Scanning for RealSense cameras...")
    print("=" * 60)
    
    try:
        # Get context
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("❌ No RealSense cameras found!")
            return
        
        print(f"✅ Found {len(devices)} RealSense camera(s):")
        print()
        
        for i, device in enumerate(devices):
            print(f"📷 Camera {i + 1}:")
            print(f"   Name: {device.get_info(rs.camera_info.name)}")
            print(f"   Serial: {device.get_info(rs.camera_info.serial_number)}")
            print(f"   Firmware: {device.get_info(rs.camera_info.firmware_version)}")
            print(f"   USB Type: {device.get_info(rs.camera_info.usb_type_descriptor)}")
            
            # Get sensors
            sensors = device.query_sensors()
            print(f"   Sensors: {len(sensors)}")
            for sensor in sensors:
                sensor_name = sensor.get_info(rs.camera_info.name)
                print(f"     - {sensor_name}")
            
            print()
            
            # Show example config
            print(f"   Example config for this camera:")
            print(f"   camera_selection:")
            print(f"     selection_method: \"serial\"")
            print(f"     serial_number: \"{device.get_info(rs.camera_info.serial_number)}\"")
            print()
            print("-" * 60)
            print()
    
    except Exception as e:
        print(f"❌ Error: {e}")
        print("Make sure RealSense SDK is installed and cameras are connected.")


if __name__ == "__main__":
    list_cameras() 