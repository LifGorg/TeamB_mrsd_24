# DTC-MRSD Unified Autonomy & Gimbal Control

## Overview

This repository contains the unified software stack for the Team Chiron MRS-D drone system. It consolidates flight autonomy, gimbal control, and vision-based geolocation.

### System Architecture

| System | Hardware | Role | Key Components |
|--------|----------|------|----------------|
| **Onboard (Drone)** | NVIDIA Jetson Orin | Flight Autonomy & Gimbal Control | AirStack (ROS 2), MAVROS, Behavior Governor, RTSP Streamer |
| **Ground Station** | x86_64 Laptop (Ubuntu) | Mission Control & Geolocator | VLM Geolocator, Foxglove Studio, QGroundControl, FVD Dashboard |

**Communication**:
- **AirStack (Onboard)** operates on `ROS_DOMAIN_ID=70`.
- **Ground Station** components operate on `ROS_DOMAIN_ID=100`.
- A **Domain Bridge** (running onboard) connects critical topics between the two domains.

---

## 1. Prerequisites

### Onboard (Drone)
- **Hardware**: NVIDIA Jetson Orin (ARM64).
- **Software**: Ubuntu 20.04/22.04, JetPack 5.x/6.x, Docker + NVIDIA Container Toolkit.
- **Peripherals**: PX4 Flight Controller (USB), Gimbal Camera (RTSP/USB).

### Ground Station
- **Hardware**: x86_64 Laptop with NVIDIA GPU (recommended for VLM).
- **Software**: Ubuntu 22.04, Docker, Python 3.10+.
- **Network**: Reliable WiFi/Radio link to the drone (Static IP configuration recommended).

---

## 2. Installation

Clone this repository on **BOTH** the Onboard Computer and Ground Station.

```bash
git clone --recursive https://github.com/LifGorg/TeamB_mrsd_24.git georgia_dtc_ops_team_chiron_mrsd
cd georgia_dtc_ops_team_chiron_mrsd
```

---

## 3. Deployment (Ground Station)

Run these steps on your **Ground Station Laptop**. This sets up the VLM Geolocator and Mission Control dashboard.

The primary method is the **FVD (Field Validation & Deployment)** script:

```bash
./scripts/FVD.sh
```

### What Happens (Ground Side)
The script initializes a `tmux` session with:

| Pane | Component | Description |
|------|-----------|-------------|
| **0** | `Foxglove Bridge` | Connects local ground tools to the ROS network (Domain 100). |
| **1** | `QGroundControl` | Standard flight planning and telemetry. |
| **2** | `RTSP Viewer` | Displays the live video feed from the drone. |
| **3** | `Local Receiver` | Web video server for browser-based monitoring. |
| **4** | **VLM Geolocator** | **Core Ground Node**. Runs Gemini-based casualty detection and GPS estimation. |

---

## 4. Deployment (Onboard Drone)

Run these steps on the **Jetson Orin (Onboard)**.

### 1. Build the AirStack Image
```bash
cd airstack/docker
docker build -f Dockerfile.unified -t airstack-unified:latest .
```

### 2. Launch Autonomy Stack
Start the unified container in background mode:
```bash
./airstack/docker/run_unified.sh
```
*This automatically starts:*
- **MAVROS**: Communication with PX4.
- **Behavior Governor**: Flight logic.
- **Domain Bridge**: Links Domain 70 (Drone) to Domain 100 (Ground).

### 3. Manual Gimbal Control (Optional)
Enter the container to run specific gimbal nodes if not auto-started:
```bash
docker exec -it airstack-unified bash
source /root/ros_ws/install/setup.bash
ros2 run gimbal_control gimbal_control_node
```

---

## 5. Configuration

### VLM Geolocator (Ground)
Located in `vlm_geolocator/config/`:
- `camera_config.yaml`: RTSP stream URL (should match drone IP).
- `gps_config.yaml`: Target GPS coordinate offset logic.

### Environment Variables
Set these before running scripts to override defaults:

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_NAME` | `dtc_mrsd` | Robot namespace. |
| `ROS_DOMAIN_ID` | `70` (Onboard) / `100` (Ground) | DDS Domain ID. |

---

## 6. Repository Structure

- `airstack/` – **Onboard Code**. ROS 2 workspace for flight autonomy (runs on Jetson).
- `vlm_geolocator/` – **Ground Code**. Casualty detection and geolocation (runs on Laptop).
- `operator/` – **Ground Tools**. Foxglove extensions.
- `scripts/` – **Utilities**. Deployment scripts (e.g., `FVD.sh`).

---

## 7. Troubleshooting

| Symptom | Check |
|---------|-------|
| **No Video on Ground** | Check `camera_config.yaml` RTSP URL. Verify drone IP reachability. |
| **VLM Not Detecting** | Ensure Ground Station has internet access (for Gemini API) or local model loaded. |
| **MAVROS Disconnected** | On drone: Check USB connection to Pixhawk (`/dev/ttyACM0`). |
| **Bridge Not Working** | Verify `domain_bridge` is running in the onboard container. |

---
*For legacy documentation, refer to archived files.*
