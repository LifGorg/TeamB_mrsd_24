# VLM Geolocator Project Structure

## Directory Tree

```
vlm_geolocator/
│
├── config/                                # Configuration files
│   ├── camera_config.yaml                 # Camera and GStreamer configuration
│   ├── ros_config.yaml                    # ROS2 topics and QoS configuration
│   └── system_config.yaml                 # System parameter configuration
│
├── src/                                   # Source code
│   ├── vlm_geolocator/                    # Main package
│   │   ├── __init__.py
│   │   │
│   │   ├── core/                          # Core functionality
│   │   │   ├── __init__.py
│   │   │   └── config.py                  # Configuration manager
│   │   │
│   │   ├── sensors/                       # Sensor management
│   │   │   ├── __init__.py
│   │   │   └── sensor_manager.py          # Sensor data management
│   │   │
│   │   ├── gps/                           # GPS calculation
│   │   │   ├── __init__.py
│   │   │   └── calculator.py              # GPS coordinate estimation
│   │   │
│   │   ├── vision/                        # Vision processing
│   │   │   ├── __init__.py
│   │   │   ├── video_receiver.py          # GStreamer video reception
│   │   │   ├── detector_interface.py      # Detector abstract interface
│   │   │   └── isaac_detector.py          # Isaac-0.1 detector
│   │   │
│   │   ├── ros_interface/                 # ROS2 interface
│   │   │   ├── __init__.py
│   │   │   ├── publishers.py              # Publisher management
│   │   │   └── subscribers.py             # Subscriber management
│   │   │
│   │   └── utils/                         # Utility functions
│   │       └── __init__.py
│   │
│   ├── vision_inference_node_refactored.py # Main node (refactored)
│   └── gps_domain_bridge_refactored.py     # GPS bridge node (refactored)
│
├── scripts/                               # Scripts
│   ├── start_system.sh                    # System startup script
│   ├── stop_system.sh                     # System stop script
│   └── verify_system.sh                   # System verification script
│
├── tests/                                 # Tests (to be added)
│
├── docs/                                  # Documentation
│   ├── QUICKSTART.md                      # Quick start guide
│   └── REFACTORING.md                     # Refactoring comparison document
│
├── dtc_network_msgs/                      # ROS message definitions
│   ├── msg/
│   │   └── HumanDataMsg.msg
│   ├── srv/
│   │   └── DroneHemo.srv
│   ├── CMakeLists.txt
│   └── package.xml
│
├── inference_logs/                        # Inference logs (generated at runtime)
│
├── README.md                              # Main documentation
├── requirements.txt                       # Python dependencies
│
└── [Old files - kept for reference]
    ├── vision_inference_node.py           # Original node (v1.0)
    ├── isaac_detector.py                  # Original detector (v1.0)
    ├── gps_domain_bridge.py               # Original bridge (v1.0)
    ├── start_complete_system.sh           # Original launch script (v1.0)
    └── ...
```

## Module Descriptions

### 1. Core Module (`core/`)

#### `config.py`
- **ConfigManager**: Configuration manager
- Load configuration from YAML files
- Provide type-safe configuration access
- Support configuration validation

Main classes:
- `CameraConfig`: Camera configuration
- `GStreamerConfig`: GStreamer configuration
- `ROS2Config`: ROS2 configuration
- `SystemConfig`: System configuration

### 2. Sensor Module (`sensors/`)

#### `sensor_manager.py`
- **SensorManager**: Sensor data manager
- Manage GPS, heading, altitude, gimbal attitude data
- Automatically validate data validity
- Data timeout checking
- Sensor health monitoring

Main classes:
- `SensorData`: Sensor data encapsulation
- `SensorManager`: Sensor manager

### 3. GPS Module (`gps/`)

#### `calculator.py`
- **GPSCalculator**: GPS coordinate estimator
- Pixel coordinates to world coordinates conversion
- Camera-gimbal-body coordinate system transformations
- WGS84 geographic coordinate calculation

Main functionality:
- `estimate_target_gps()`: Estimate GPS from pixels
- `estimate_from_snapshot()`: Estimate using sensor snapshot

### 4. Vision Module (`vision/`)

#### `video_receiver.py`
- **VideoFrameReceiver**: GStreamer video receiver
- Manage GStreamer pipeline
- Frame buffering and timestamps
- Thread-safe frame access

#### `detector_interface.py`
- **DetectorInterface**: Detector abstract interface
- Define standard detector interface
- Support pluggable detector implementations

#### `isaac_detector.py`
- **IsaacDetectorWrapper**: Isaac-0.1 detector wrapper
- VLM model inference
- Result parsing and coordinate conversion
- Inference logging

### 5. ROS Interface Module (`ros_interface/`)

#### `publishers.py`
- **ROSPublisherManager**: Publisher manager
- Manage all ROS publishers
- Unified QoS configuration
- Simplified publishing interface

#### `subscribers.py`
- **ROSSubscriberManager**: Subscriber manager
- Manage all ROS subscribers
- Unified callback registration

### 6. Main Nodes

#### `vision_inference_node_refactored.py`
- **VisionInferenceNode**: Main vision inference node
- Integrate all modules
- ROS2 lifecycle management
- Asynchronous processing

Functionality:
- Video stream reception
- Target detection
- GPS estimation
- Result publishing
- Sensor data subscription

#### `gps_domain_bridge_refactored.py`
- **GPSDomainBridge**: GPS domain bridge node
- Domain 100 → Domain 0 bridging
- Forward GPS data

## Data Flow

```
Video Stream (UDP)
    ↓
VideoFrameReceiver
    ↓
Trigger (Service/Topic)
    ↓
IsaacDetectorWrapper
    ↓
Detection Results [(cx, cy), ...]
    ↓
GPSCalculator ← SensorManager (sensor snapshot)
    ↓
GPS Coordinates (lat, lon)
    ↓
ROSPublisherManager
    ↓
Downstream Systems (UDP/ROS)
```

## Configuration Flow

```
config/*.yaml
    ↓
ConfigManager
    ↓
Module initialization
```

## Dependencies

```
VisionInferenceNode
    ├── ConfigManager
    ├── SensorManager
    ├── GPSCalculator
    ├── VideoFrameReceiver
    ├── IsaacDetectorWrapper
    ├── ROSPublisherManager
    └── ROSSubscriberManager
```

## External Dependencies

- **ROS2**: Communication framework
- **GStreamer**: Video processing
- **PyTorch**: Deep learning
- **Transformers**: VLM model
- **OpenCV**: Image processing
- **NumPy/SciPy**: Numerical computation

## Runtime Generated

- `inference_logs/`: Inference logs and input images
- `/tmp/vlm_geolocator/`: Process PID files
- `/tmp/*.log`: Runtime logs

## Extension Points

1. **New Detector**: Implement `DetectorInterface`
2. **New Sensor**: Extend `SensorManager`
3. **New Configuration**: Add YAML configuration
4. **New ROS Topics**: Use Publisher/SubscriberManager
5. **New Coordinate System**: Extend `GPSCalculator`
