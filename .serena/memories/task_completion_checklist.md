# FluentVision ROS2 Task Completion Checklist

## After Making Code Changes

### 1. Build Verification
```bash
# Rebuild affected packages
colcon build --packages-select <modified_package>

# Or full rebuild if multiple packages affected
colcon build
```

### 2. Dependency Check
```bash
# Ensure all dependencies are satisfied
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Testing
- Currently no automated tests are configured in the project
- Manual testing recommended:
  - Run the modified node
  - Check topic output with `ros2 topic echo`
  - Verify service calls if applicable
  - Monitor logs for errors

### 4. Code Quality
- Since no linting/formatting tools are currently configured:
  - Ensure consistent code style with existing code
  - Follow C++17 standards
  - Use meaningful variable/function names
  - Add appropriate logging statements

### 5. Documentation
- Update README.md if functionality changed
- Update inline comments for complex logic
- Update service/message definitions if modified

### 6. Version Control
```bash
# Check what changed
git status
git diff

# Stage and commit with descriptive message
git add <files>
git commit -m "feat/fix/refactor: description"
```

## Common Issues to Check
- Memory leaks (especially with RealSense cameras)
- Port conflicts (RTMP: 1935, WebSocket: 8765, HTTP: 8080)
- RealSense driver version compatibility (2.55.1 recommended)
- ROS2 topic naming consistency
- Service response handling

## Performance Considerations
- Check CPU usage during runtime
- Monitor memory consumption
- Verify frame rates for camera nodes
- Check network bandwidth for streaming nodes