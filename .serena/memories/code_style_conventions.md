# FluentVision ROS2 Code Style and Conventions

## C++ Style
- **Standard**: C++17
- **Naming Convention**: 
  - Classes: PascalCase (e.g., `FVDepthCameraNode`)
  - Functions/Methods: snake_case
  - Variables: snake_case
  - Constants: UPPER_SNAKE_CASE
  - Namespaces: lowercase

## File Organization
- **Headers**: Located in `include/<package_name>/` directories
- **Source**: Located in `src/` directories
- **File Extensions**: `.cpp` for source, `.hpp` for headers

## Build Configuration
- **CMake Style**: Modern CMake with target-based dependencies
- **Compiler Flags**: `-Wall -Wextra -Wpedantic` for GCC/Clang
- **Package Format**: ROS2 package format 3

## ROS2 Conventions
- **Node Names**: Descriptive names with package prefix (e.g., `fv_realsense_node`)
- **Topic Names**: Use forward slashes, descriptive names (e.g., `/fv_mjpeg_image`)
- **Service Names**: CamelCase for service definitions (e.g., `GetDistance.srv`)
- **Logging**: Use ROS2 logging macros (RCLCPP_INFO, RCLCPP_ERROR, etc.)
- **Emojis in Logs**: Used for visual clarity (🚀 for startup, ✅ for success, ❌ for errors)

## Package Structure
```
package_name/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── package_name/
│       └── *.hpp
├── src/
│   └── *.cpp
├── srv/  (if services)
│   └── *.srv
├── launch/  (if launch files)
│   └── *.launch.py
└── config/  (if configuration)
    └── *.yaml
```

## Documentation
- Inline comments for complex logic
- README.md files for packages
- Service/Message definitions with descriptions