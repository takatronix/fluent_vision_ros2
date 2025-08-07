# Web Dashboard

FluentVision Web Dashboard - A flexible multi-stream monitoring dashboard for robotics and IoT applications.

## Features

- 🎥 **Multi-Stream Support**: MJPEG, WebSocket, WebRTC, RTMP, ROS2 topics
- 🎯 **Free Layout System**: Drag & drop, resize, and arrange windows freely
- 🎚️ **Individual Opacity Control**: Adjust transparency for each window
- 🎨 **Auto-hiding Title Bars**: Clean interface with overlay controls
- 💾 **Layout Management**: Save and load custom layouts as JSON
- 🌐 **Web Dashboard Integration**: Embed any web page as iframe
- 📱 **Cross-Device Support**: Works on PC, mobile, and AR glasses
- 🤖 **ROS2 Integration**: Connect to ROS2 topics via rosbridge

## Quick Start

### 1. Basic Usage
Simply open `index.html` in a web browser:
```bash
cd web_dashboard
python3 -m http.server 8000
# Open http://localhost:8000 in browser
```

### 2. ROS2 Integration
For ROS2 streaming support:
```bash
# Start rosbridge (if available)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Or use the simple WebSocket server
python3 simple-ros2-websocket.py
```

### 3. Test Stream Server
For testing with demo streams:
```bash
python3 test-stream-server.py
# Provides test MJPEG streams on http://localhost:5000
```

## Project Structure

```
web_dashboard/
├── index.html                    # Main dashboard application
├── app.js                        # Legacy stream manager
├── free-layout.js                # Free layout system
├── dashboard-examples.js         # Dashboard presets and examples
├── pointcloud-viewer.js          # 3D point cloud visualization
├── mobile-config.js              # Mobile device configuration
├── web-dashboard-presets.html    # Web dashboard gallery
├── simple-ros2-websocket.py      # ROS2 WebSocket bridge
├── test-stream-server.py         # Test MJPEG stream server
└── README.md                     # This file
```

## Usage Guide

### Adding Streams
1. Click "⚡ クイック追加" for quick presets
2. Click "+ カスタム" for custom stream URLs
3. Use "🌐 WebページURL入力" to add web dashboards

### Layout Management
- **Drag**: Click and hold title bar to move windows
- **Resize**: Drag the corner handle to resize
- **Opacity**: Use individual sliders in each window
- **Save**: Click "💾 保存" to save current layout
- **Load**: Click "📂 読込" to restore saved layout

#### Storage Location
- Layouts are saved in **browser's LocalStorage** (`fluentvision-layout` key)
- Each browser/device maintains its own saved layouts
- To export/import layouts between devices, use the JSON export feature in settings
- Clear browser data to reset all saved layouts

### Keyboard Shortcuts
- `ESC`: Close settings panel and menus

### Themes
- Dark (default): Best for AR glasses
- Light: For bright environments
- AR: Transparent background for overlay

## ROS2 Topics Support

The dashboard can visualize ROS2 topics:
- Compressed images from camera topics
- Point clouds from depth sensors
- Custom sensor data streams

Configure ROS2 connection in the settings panel.

## Development

### Adding Custom Stream Types
Edit `index.html` and add your stream type in the `createContent()` method:

```javascript
case 'custom_type':
    return `<your-custom-element></your-custom-element>`;
```

### Creating Dashboard Presets
Add presets in `dashboard-examples.js` for quick access to common configurations.

## Requirements

- Modern web browser with ES6 support
- Python 3.x for local server
- ROS2 (optional, for ROS integration)
- rosbridge_suite (optional, for ROS WebSocket bridge)

## License

Part of the FluentVision project for robotics and agricultural applications.

## Support

For issues and feature requests, please contact the FluentVision team.