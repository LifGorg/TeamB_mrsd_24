#!/bin/bash
# Refactored system startup script

set -e

echo "=========================================="
echo "🚀 Starting VLM Geolocation System v2.0"
echo "=========================================="

# Configuration
PROJECT_DIR="/home/triage/vlm_geolocator"
ROS_WS="/home/triage/georgia_dtc_ops_team_chiron_mrsd/uav_to_ugv/clean_workspace/ros_ws"
UDP_IP="10.3.1.106"
GPS_PORT=12100
HUMAN_PORT=12200

# Set up environment
echo ""
echo "1️⃣  Setting up environment..."
source /opt/ros/humble/setup.bash

if [ -d "$ROS_WS/install" ]; then
    source "$ROS_WS/install/setup.bash"
    echo "   ✅ Loaded ROS2 workspace: $ROS_WS"
else
    echo "   ⚠️  ROS workspace does not exist"
fi

# Note: Package should be installed with 'pip install -e .' before running
echo "   ✅ Environment ready"

# Start vision inference node
echo ""
echo "2️⃣  Starting vision inference node (refactored version)..."
cd "$PROJECT_DIR"
python3 src/vision_inference_node_refactored.py > /tmp/vision_inference.log 2>&1 &
VISION_PID=$!
echo "   ✅ Vision inference node started (PID: $VISION_PID)"

# Wait for node to start
sleep 3

# Start drone GPS UDP publisher
echo ""
echo "3️⃣  Starting drone GPS UDP publisher..."
echo "   Subscribing: /dtc_mrsd_/mavros/global_position/global"
echo "   Target: $UDP_IP:$GPS_PORT"
ros2 run dell_publisher gps_udp_publisher \
    --ros-args \
    -p topic_name:="/dtc_mrsd_/mavros/global_position/global" \
    -p udp_ip:="$UDP_IP" \
    -p udp_port:=$GPS_PORT \
    > /tmp/drone_gps_udp.log 2>&1 &
DRONE_GPS_PID=$!
echo "   ✅ Drone GPS publisher started (PID: $DRONE_GPS_PID)"

sleep 1

# Start target GPS UDP publisher
echo ""
echo "4️⃣  Starting target GPS UDP publisher..."
echo "   Subscribing: /manual_targets/geolocated"
echo "   Target: $UDP_IP:$HUMAN_PORT"
ros2 run dell_publisher rostopic_publisher \
    --ros-args \
    -p topic_name:="/manual_targets/geolocated" \
    -p topic_type:="dtc_network_msgs/msg/HumanDataMsg" \
    -p udp_ip:="$UDP_IP" \
    -p udp_port:=$HUMAN_PORT \
    > /tmp/human_data_udp.log 2>&1 &
HUMAN_GPS_PID=$!
echo "   ✅ Target GPS publisher started (PID: $HUMAN_GPS_PID)"

echo ""
echo "=========================================="
echo "✅ System startup complete (v2.0 refactored version)"
echo "=========================================="
echo ""
echo "📊 System Components:"
echo "  1. Vision Inference Node (PID: $VISION_PID)"
echo "     - Video stream: UDP 5005"
echo "     - MAVROS sensor data subscription"
echo "     - Target detection and GPS estimation"
echo ""
echo "  2. Drone GPS Publisher (PID: $DRONE_GPS_PID)"
echo "     - UDP: $UDP_IP:$GPS_PORT"
echo ""
echo "  3. Target GPS Publisher (PID: $HUMAN_GPS_PID)"
echo "     - UDP: $UDP_IP:$HUMAN_PORT"
echo ""
echo "📂 Log Files:"
echo "  - Vision node: /tmp/vision_inference.log"
echo "  - Drone GPS: /tmp/drone_gps_udp.log"
echo "  - Target GPS: /tmp/human_data_udp.log"
echo "  - Inference logs: $PROJECT_DIR/inference_logs/"
echo ""
echo "🔍 Monitoring Commands:"
echo "  tail -f /tmp/vision_inference.log"
echo ""
echo "⚙️  Trigger Detection:"
echo "  ros2 service call /trigger_capture std_srvs/srv/Trigger"
echo "  ros2 topic pub --once /trigger_capture std_msgs/msg/Bool \"{data: true}\""
echo ""
echo "🛑 Stop System:"
echo "  ./scripts/stop_system.sh"
echo ""
echo "=========================================="

# Save PIDs
mkdir -p /tmp/vlm_geolocator
echo "$VISION_PID" > /tmp/vlm_geolocator/vision.pid
echo "$DRONE_GPS_PID" > /tmp/vlm_geolocator/drone_gps.pid
echo "$HUMAN_GPS_PID" > /tmp/vlm_geolocator/human_gps.pid

# Cleanup function
cleanup() {
    echo ""
    echo "🛑 Stopping all components..."
    kill $VISION_PID $DRONE_GPS_PID $HUMAN_GPS_PID 2>/dev/null || true
    rm -rf /tmp/vlm_geolocator
    echo "✅ System stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "Press Ctrl+C to stop the system..."
echo ""

# Wait
wait
