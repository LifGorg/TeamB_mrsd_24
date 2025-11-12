#!/bin/bash
# System stop script

echo "🛑 Stopping VLM Geolocation System..."

PID_DIR="/tmp/vlm_geolocator"

if [ ! -d "$PID_DIR" ]; then
    echo "⚠️  No running system found"
    exit 1
fi

# Read PIDs
VISION_PID=$(cat $PID_DIR/vision.pid 2>/dev/null)
DRONE_GPS_PID=$(cat $PID_DIR/drone_gps.pid 2>/dev/null)
HUMAN_GPS_PID=$(cat $PID_DIR/human_gps.pid 2>/dev/null)

# Stop processes
echo "Stopping vision inference node (PID: $VISION_PID)..."
kill $VISION_PID 2>/dev/null && echo "  ✅ Stopped" || echo "  ⚠️  Process not found"

echo "Stopping drone GPS publisher (PID: $DRONE_GPS_PID)..."
kill $DRONE_GPS_PID 2>/dev/null && echo "  ✅ Stopped" || echo "  ⚠️  Process not found"

echo "Stopping target GPS publisher (PID: $HUMAN_GPS_PID)..."
kill $HUMAN_GPS_PID 2>/dev/null && echo "  ✅ Stopped" || echo "  ⚠️  Process not found"

# Cleanup
rm -rf $PID_DIR

echo ""
echo "✅ System completely stopped"
