# Quick Start Guide

This guide will help you quickly start and use the VLM Geolocator system.

## Prerequisites Check

```bash
# Check ROS2
ros2 --version
# Should display: ros2 cli version: humble

# Check Python
python3 --version
# Should display: Python 3.8 or higher

# Check CUDA (optional)
nvidia-smi
```

## 5-Minute Quick Start

### Step 1: Install Dependencies

```bash
cd /home/triage/vlm_geolocator

# Install the package in editable mode (this also installs dependencies)
pip install -e .

# Set up environment
source /opt/ros/humble/setup.bash
```

### Step 2: Check Configuration

```bash
# View configuration files
cat config/camera_config.yaml
cat config/ros_config.yaml
cat config/system_config.yaml

# If you need to modify, edit the YAML files
nano config/system_config.yaml
```

### Step 3: Start System

```bash
# Start the complete system
./scripts/start_system.sh
```

The system will start the following components:
1. Vision inference node (receives video, detects targets, calculates GPS)
2. Drone GPS UDP publisher
3. Target GPS UDP publisher

### Step 4: Trigger Detection

In another terminal:

```bash
# Method 1: Use ROS service
ros2 service call /trigger_capture std_srvs/srv/Trigger

# Method 2: Use ROS topic
ros2 topic pub --once /trigger_capture std_msgs/msg/Bool "{data: true}"
```

### Step 5: View Results

```bash
# View real-time logs
tail -f /tmp/vision_inference.log

# View detection results
ros2 topic echo /manual_targets/geolocated

# View inference logs
ls -lh inference_logs/
cat inference_logs/20250107_*.json
```

### Step 6: Stop System

```bash
# Graceful shutdown
./scripts/stop_system.sh

# Or press Ctrl+C
```

## Common Issues

### Q: Cannot receive video stream?

Check video source:
```bash
# Test GStreamer pipeline
gst-launch-1.0 udpsrc port=5005 ! fakesink
```

### Q: Not detecting targets?

Check:
1. Model path is correct (`config/system_config.yaml`)
2. Video frames are being received (check logs)
3. Sensor data is normal

```bash
# Check sensor topics
ros2 topic list | grep mavros
ros2 topic echo /dtc_mrsd_/mavros/global_position/global
```

### Q: GPS estimation failed?

Ensure all sensor data is available:
- GPS position
- Compass heading
- Relative altitude
- Gimbal attitude

Check sensor health information in logs.

## Advanced Usage

### Custom Configuration

Edit configuration files to suit your needs:

```yaml
# config/camera_config.yaml
camera:
  name: "Your Camera Name"
  resolution:
    width: 1920  # Change to your resolution
    height: 1080
  intrinsics:
    fx: 1000.0  # Change to your camera intrinsics
    fy: 1000.0
    cx: 960.0
    cy: 540.0
```

### Debug Mode

```bash
# Start with detailed logs
python3 src/vision_inference_node_refactored.py --ros-args --log-level debug
```

### Performance Monitoring

```bash
# Monitor CPU and memory
htop

# Monitor GPU
watch -n 1 nvidia-smi
```

## Next Steps

- Read complete documentation: [README.md](../README.md)
- Understand architecture design: [REFACTORING.md](REFACTORING.md)
- Develop new features: Refer to API documentation

## Getting Help

If you have questions:
1. Check log files
2. Review documentation
3. Contact development team
