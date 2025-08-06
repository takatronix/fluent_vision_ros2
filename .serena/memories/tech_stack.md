# FluentVision ROS2 Tech Stack

## Core Technologies
- **Programming Language**: C++17
- **Framework**: ROS2 Humble Hawksbill
- **Build System**: colcon with ament_cmake
- **OS**: Ubuntu 22.04 LTS

## Key Dependencies

### ROS2 Packages
- rclcpp: ROS2 C++ client library
- sensor_msgs: Message types for sensors
- cv_bridge: OpenCV-ROS2 bridge
- tf2_ros: Transform library
- pcl_conversions, pcl_ros: Point cloud processing
- image_transport: Image transport plugins
- compressed_image_transport: Compressed image transport
- geometry_msgs: Geometry message types
- std_srvs: Standard service types

### External Libraries
- **Intel RealSense SDK**: librealsense2 (2.55.1-0~realsense.12474 recommended)
- **OpenCV**: 4.5.4+ for computer vision
- **WebSocket++**: For WebSocket server implementation
- **Boost**: System libraries
- **FFmpeg**: For RTMP streaming (libavcodec, libavformat, libavutil, libswscale)
- **PCL**: Point Cloud Library for 3D data processing

## Development Tools
- **CMake**: 3.8+ for build configuration
- **pkg-config**: For finding system packages
- **rosdep**: For managing ROS dependencies
- **colcon**: Build tool for ROS2 packages

## Message/Service Generation
- rosidl_default_generators: For generating custom services and messages