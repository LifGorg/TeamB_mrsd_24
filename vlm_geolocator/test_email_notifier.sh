#!/bin/bash
# Test script for Casualty Email Notifier

echo "=========================================="
echo "Casualty Email Notifier - Test Script"
echo "=========================================="
echo ""

# Set environment
export ROS_DOMAIN_ID=100

# Kill any existing notifier process
echo "1. Cleaning up existing processes..."
pkill -f "casualty_email_notifier.py" 2>/dev/null
sleep 1

# Start the email notifier node
echo "2. Starting email notifier node..."
cd /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator
bash -c "source /opt/ros/humble/setup.bash && python3 src/casualty_email_notifier.py" > /tmp/email_notifier.log 2>&1 &
NOTIFIER_PID=$!
echo "   Node started with PID: $NOTIFIER_PID"
sleep 3

# Check if node is running
if ps -p $NOTIFIER_PID > /dev/null; then
    echo "   ✅ Node is running"
else
    echo "   ❌ Node failed to start"
    echo "   Check log: tail /tmp/email_notifier.log"
    exit 1
fi

# Show node output
echo ""
echo "3. Node startup log:"
echo "-------------------"
head -10 /tmp/email_notifier.log
echo "-------------------"
echo ""

# Publish test GPS message
echo "4. Publishing test GPS message to /casualty_geolocated..."
bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub --once /casualty_geolocated sensor_msgs/msg/NavSatFix '{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: \"world\"},
  status: {status: 0, service: 1},
  latitude: 33.775600,
  longitude: -84.396300,
  altitude: 100.0,
  position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  position_covariance_type: 0
}'" 2>&1 | head -3

echo ""
echo "5. Waiting for email to be sent (10 seconds)..."
sleep 10

# Show email sending result
echo ""
echo "6. Email notifier log (last 15 lines):"
echo "--------------------------------------"
tail -15 /tmp/email_notifier.log
echo "--------------------------------------"
echo ""

echo "=========================================="
echo "Test completed!"
echo ""
echo "Next steps:"
echo "1. Check your email inbox (recipients from config/email_config.yaml)"
echo "2. Check spam folder if email not in inbox"
echo "3. Review full log: tail -f /tmp/email_notifier.log"
echo ""
echo "To stop the notifier:"
echo "  kill $NOTIFIER_PID"
echo "  OR"
echo "  pkill -f casualty_email_notifier.py"
echo "=========================================="

