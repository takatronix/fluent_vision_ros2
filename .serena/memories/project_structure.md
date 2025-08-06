# FluentVision ROS2 Project Structure

## Root Directory
```
fluent_vision_ros2/
├── README.md           # Main project documentation
├── .gitignore         # Git ignore rules
├── src/               # Source packages
├── build/             # Build artifacts (generated)
├── install/           # Install artifacts (generated)
├── log/               # Build logs (generated)
├── test/              # Test directory (empty currently)
├── tests/             # Tests directory (empty currently)
└── .serena/           # Serena MCP project configuration
```

## Source Organization (`src/`)
```
src/
├── sensors/           # Sensor-related packages
│   ├── fv_realsense/  # Intel RealSense camera support
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/fv_realsense/
│   │   │   └── *.hpp
│   │   ├── src/
│   │   │   ├── fv_realsense_node.cpp
│   │   │   ├── fv_realsense_simple_node.cpp
│   │   │   └── image_saver_node.cpp
│   │   ├── srv/
│   │   │   ├── GetDistance.srv
│   │   │   └── GetCameraInfo.srv
│   │   ├── launch/
│   │   ├── config/
│   │   └── scripts/
│   └── sipeed_tof_ms_a010/  # TOF camera support
│
├── ai/                # AI/ML packages
│   └── fv_face_recognizer/  # Face recognition
│
├── streaming/         # Streaming server packages
│   ├── fv_mjpeg_server/      # MJPEG streaming
│   ├── fv_websocket_server/  # WebSocket streaming
│   ├── fv_image_distributor/ # HTTP distribution
│   ├── fv_rtmp_server/       # RTMP streaming
│   └── fv_recorder/          # Recording functionality
│
├── control/           # Control packages (future)
│
└── utils/             # Utility packages
```

## Package Components
Each package typically contains:
- `CMakeLists.txt`: Build configuration
- `package.xml`: Package manifest and dependencies
- `include/<package>/`: Header files
- `src/`: Source files
- `srv/`: Service definitions (if applicable)
- `msg/`: Message definitions (if applicable)
- `launch/`: Launch files (if applicable)
- `config/`: Configuration files (if applicable)
- `scripts/`: Utility scripts (if applicable)

## Build Artifacts
- `build/`: CMake build files and intermediate objects
- `install/`: Installed executables, libraries, and resources
- `log/`: Build and test logs

## Configuration
- Config files use YAML format
- Located in individual package `config/` directories
- Examples: camera settings, streaming parameters