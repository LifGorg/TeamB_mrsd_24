# DTC-MRSD Unified Autonomy & Gimbal Control

## Overview

This repository contains the unified software stack for the Team Chiron MRS-D drone system. It consolidates flight autonomy, gimbal control, and vision-based geolocation into a single deployment unit.

**Key Features:**
- **Unified Runtime**: Single ROS 2 Humble Docker container for all onboard systems.
- **Autonomy**: MAVROS interface, behavior governor, and domain bridge.
- **Vision**: Real-time casualty detection and GPS geolocation (VLM Geolocator).
- **Operations**: Integrated mission control and visualization via Foxglove and QGroundControl.

---

## 1. Prerequisites

### Hardware
- **Onboard Computer**: NVIDIA Jetson Orin (or compatible ARM64/x86 platform with NVIDIA GPU).
- **Flight Controller**: PX4-based flight controller (connected via serial/USB).
- **Camera**: USB Camera (mounted as `/dev/video4` by default) or RTSP stream.

### Software
- **OS**: Ubuntu 20.04 or 22.04.
- **Docker**: Installed and configured.
- **NVIDIA Container Toolkit**: Required for GPU acceleration (JetPack).

---

## 2. Installation

1. **Clone the Repository**
   ```bash
   git clone --recursive https://github.com/LifGorg/TeamB_mrsd_24.git georgia_dtc_ops_team_chiron_mrsd
   cd georgia_dtc_ops_team_chiron_mrsd
   ```

2. **Build the Unified Docker Image**
   ```bash
   cd airstack/docker
   docker build -f Dockerfile.unified -t airstack-unified:latest .
   ```
   *Note: This build includes ROS 2 Humble, MAVROS, and all dependency packages.*

---

## 3. Deployment (Field Operation)

The primary method for field operation is the **FVD (Field Validation & Deployment)** script. This script automates the startup of all necessary subsystems in a managed `tmux` session.

### Launch
Run the deployment script from the repository root:

```bash
./scripts/FVD.sh
```

### What Happens
The script initializes a split-screen interface (tmux) with the following active components:

| Pane/Window | Component | Function |
|-------------|-----------|----------|
| **Pane 0** | `Foxglove Bridge` | Bridges ROS 2 topics (Domain 100) for ground station visualization. |
| **Pane 1** | `QGroundControl` | Flight planning and telemetry monitoring. |
| **Pane 2** | `RTSP/Camera` | Handles video capture and streaming. |
| **Pane 3** | `Local Receiver` | Displays the local video feed for verification. |
| **Pane 4** | **VLM Geolocator** | Runs the vision inference node for casualty detection. |
| **Win 2** | `Video Merger` | Combines video streams (if configured). |
| **Win 3** | `HTML Viewer` | Displays Gemini analysis results in real-time. |

### Stopping the System
To stop all processes and close the session:
- Press `Ctrl+C` in the respective panes, or
- Kill the tmux session: `tmux kill-session -t chiron_ops_panes`

---

## 4. Manual Operation (Development)

If you need to run specific nodes manually or debug the system, follow these steps.

### Launch the Container
Start the unified container in background mode:
```bash
./airstack/docker/run_unified.sh
```
*This automatically starts MAVROS, the behavior governor, and the domain bridge.*

### Enter the Container
```bash
docker exec -it airstack-unified bash
source /root/ros_ws/install/setup.bash
```

### Run Gimbal Nodes
Gimbal control nodes are not started automatically in the container. Run them as needed:
```bash
# Status Monitor
ros2 run gimbal_control gimbal_status_node

# Control Interface
ros2 run gimbal_control gimbal_control_node

# Angle Control
ros2 run gimbal_control gimbal_angle_control_node
```

### Rebuilding Code
The workspace is bind-mounted. You can edit code on the host, then rebuild inside the container:
```bash
cd /root/ros_ws
colcon build --symlink-install --merge-install
source install/setup.bash
```

---

## 5. Configuration

### Environment Variables
Set these before running `run_unified.sh` or `FVD.sh` to override defaults:

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_NAME` | `dtc_mrsd` | Logical robot name (used for namespacing). |
| `ROS_DOMAIN_ID` | `70` | DDS domain for flight autonomy. |

### VLM Geolocator Config
Located in `vlm_geolocator/config/`:
- `camera_config.yaml`: Camera resolution, framerate, and source.
- `gps_config.yaml`: GPS offset and coordinate calculations.
- `ros_config.yaml`: Topic names and QoS settings.

---

## 6. Repository Structure

- `airstack/` – **Core Autonomy**. Unified ROS 2 workspace (`ros_ws`) and Docker configuration.
- `operator/` – **Ground Control**. Foxglove Studio extensions and visualization tools.
- `vlm_geolocator/` – **Vision System**. Casualty detection, GPS geolocation, and video analysis.
- `scripts/` – **Utilities**. Deployment scripts (including `FVD.sh`).

---

## 7. Troubleshooting

| Symptom | Check |
|---------|-------|
| **Container exits immediately** | Check `docker logs airstack-unified`. Ensure NVIDIA runtime is available. |
| **MAVROS not connecting** | Verify flight controller USB connection and permissions (`/dev/ttyACM0`). |
| **Camera fail** | Check if `/dev/video4` exists or verify RTSP stream URL in `camera_config.yaml`. |
| **"Casualty Not Found"** | Ensure the VLM Geolocator node (Pane 4) is running and receiving GPS data. |

---
*For legacy documentation (multi-container setup), refer to archived files.*
