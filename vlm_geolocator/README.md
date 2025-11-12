# VLM Geolocator - Vision Language Model Geolocation System

![Version](https://img.shields.io/badge/version-2.0.0-blue)
![Python](https://img.shields.io/badge/python-3.8+-green)
![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)

A modular system for detecting targets from drone video streams and estimating their GPS coordinates.

## 📋 Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [API Documentation](#api-documentation)

## Features

- **Video Stream Reception**: Receive H.265-encoded UDP video streams via GStreamer
- **Target Detection**: Detect human targets using Isaac-0.1 VLM model
- **GPS Estimation**: Calculate target GPS coordinates based on camera intrinsics, gimbal pose, and drone position
- **ROS2 Integration**: Subscribe to MAVROS sensor data and publish detection results
- **Modular Design**: Clear module separation, easy to maintain and extend
- **Configuration Management**: YAML-based configuration system

## System Architecture

```
┌─────────────────┐
│  Video Stream   │ (UDP H.265)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Video Receiver  │ (GStreamer)
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌─────────────────┐
│    Detector     │◄─────┤ Trigger Service │
│   (Isaac-0.1)   │      └─────────────────┘
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌─────────────────┐
│ GPS Calculator  │◄─────┤ Sensor Manager  │
└────────┬────────┘      └────────┬────────┘
         │                        ▲
         │                        │
         ▼                        │
┌─────────────────┐      ┌────────────────┐
│  ROS Publisher  │      │ ROS Subscriber │
└─────────────────┘      └────────────────┘
```

## 📦 Installation

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.8+
- CUDA (optional, for GPU acceleration)

### Dependency Installation

```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-gi \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav

# Install Python dependencies
pip install -r requirements.txt
```

### Clone and Setup

```bash
cd /home/triage/vlm_geolocator

# Install the package in editable mode
pip install -e .

# If you have a ROS workspace, source it
source /opt/ros/humble/setup.bash
source /path/to/your/ros_ws/install/setup.bash
```

## ⚙️ Configuration

Configuration files are located in the `config/` directory:

### `camera_config.yaml`
Camera and GStreamer configuration

### `ros_config.yaml`
ROS2 topics and QoS configuration

### `system_config.yaml`
System parameters (timeouts, model path, etc.)

## Usage

### Method 1: Using Launch Scripts (Recommended)

```bash
# Start the complete system
./scripts/start_system.sh

# Stop the system
./scripts/stop_system.sh
```

### Method 2: Manual Launch

```bash
# Set up environment
source /opt/ros/humble/setup.bash

# Start vision inference node
python3 src/vision_inference_node_refactored.py

# In another terminal, trigger capture
ros2 topic pub --once /trigger_capture std_msgs/msg/Bool "{data: true}"
```

### Trigger Detection

There are two ways to trigger target detection:

1. **ROS2 Service**:
```bash
ros2 service call /trigger_capture std_srvs/srv/Trigger
```

2. **ROS2 Topic**:
```bash
ros2 topic pub --once /trigger_capture std_msgs/msg/Bool "{data: true}"
```

## 📁 Project Structure

```
vlm_geolocator/
├── config/                      # Configuration files
│   ├── camera_config.yaml       # Camera configuration
│   ├── ros_config.yaml          # ROS2 configuration
│   └── system_config.yaml       # System configuration
│
├── src/                         # Source code
│   ├── vlm_geolocator/          # Main package
│   │   ├── core/                # Core modules
│   │   │   ├── __init__.py
│   │   │   └── config.py        # Configuration management
│   │   │
│   │   ├── sensors/             # Sensor management
│   │   │   ├── __init__.py
│   │   │   └── sensor_manager.py
│   │   │
│   │   ├── gps/                 # GPS calculation
│   │   │   ├── __init__.py
│   │   │   └── calculator.py
│   │   │
│   │   ├── vision/              # Vision processing
│   │   │   ├── __init__.py
│   │   │   ├── video_receiver.py
│   │   │   ├── detector_interface.py
│   │   │   └── isaac_detector.py
│   │   │
│   │   ├── ros_interface/       # ROS2 interface
│   │   │   ├── __init__.py
│   │   │   ├── publishers.py
│   │   │   └── subscribers.py
│   │   │
│   │   └── utils/               # Utility functions
│   │       └── __init__.py
│   │
│   ├── vision_inference_node_refactored.py  # Main node
│   └── gps_domain_bridge_refactored.py      # GPS bridge node
│
├── scripts/                     # Launch scripts
│   ├── start_system.sh
│   └── stop_system.sh
│
├── tests/                       # Tests
├── docs/                        # Documentation
├── dtc_network_msgs/            # ROS message definitions
└── README.md
```

## 📚 API Documentation

### Core Modules

#### `ConfigManager`
Configuration manager that loads all configurations from YAML files.

```python
from vlm_geolocator.core import ConfigManager

config = ConfigManager()
camera_config = config.get_camera_config()
```

#### `SensorManager`
Sensor data management with data validation and health checking support.

```python
from vlm_geolocator.sensors import SensorManager

sensor_mgr = SensorManager(timeout=0.5)
sensor_mgr.update_gps(latitude, longitude)
snapshot = sensor_mgr.get_snapshot()
```

#### `GPSCalculator`
GPS coordinate estimation based on camera model and sensor data.

```python
from vlm_geolocator.gps import GPSCalculator

calculator = GPSCalculator(intrinsic_matrix)
result = calculator.estimate_from_snapshot(pixel_x, pixel_y, snapshot)
```

### Vision Modules

#### `VideoFrameReceiver`
GStreamer video receiver.

```python
from vlm_geolocator.vision import VideoFrameReceiver

receiver = VideoFrameReceiver(pipeline_str, frame_callback)
frame_data = receiver.get_latest_frame()
```

#### `IsaacDetectorWrapper`
Isaac-0.1 detector wrapper.

```python
from vlm_geolocator.vision import IsaacDetectorWrapper

detector = IsaacDetectorWrapper(model_path)
detections = detector.detect(image)  # [(cx, cy), ...]
```

## 🔧 Development

### Adding a New Detector

1. Implement `DetectorInterface`:
```python
from vlm_geolocator.vision import DetectorInterface

class MyDetector(DetectorInterface):
    def detect(self, image):
        # Your detection logic
        return [(cx1, cy1), (cx2, cy2), ...]
```

2. Use in node:
```python
self.detector = MyDetector(...)
```

### Modifying Configuration

Edit `config/*.yaml` files without modifying code.

## Debugging

### View Logs

```bash
# View node logs in real-time
tail -f /tmp/vision_inference.log

# View inference logs
ls -lh /home/triage/vlm_geolocator/inference_logs/
```

### Check ROS Topics

```bash
# List all topics
ros2 topic list

# View GPS data
ros2 topic echo /dtc_mrsd_/mavros/global_position/global

# View detection results
ros2 topic echo /manual_targets/geolocated
```

## 📝 Changelog

### v2.0.0 (Refactored Version)
- Modular architecture
- Configuration management system
- Improved sensor data management
- Independent GPS calculation module
- Complete documentation

### v1.0.0 (Proof of Concept)
- Basic functionality implementation

## 📄 License

MIT License - See LICENSE file for details

## 👥 Contributors

**LifGorg** - Main developer and maintainer

## 📞 Contact

For questions or issues, please open an issue on GitHub.

