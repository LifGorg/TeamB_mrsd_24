#!/bin/bash

# Get project root directory (parent of scripts directory)
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# Configuration
SESSION_NAME="chiron_ops_panes"
REMOTE_HOST="10.3.1.32"
REMOTE_USER="dtc"
REMOTE_PASS="passme24"
REMOTE_SCRIPT_PATH="/home/dtc/restart.sh"


# Kill existing processes to prevent duplicates
echo "Cleaning up existing processes..."

# Kill foxglove bridge
echo "  - Stopping foxglove_bridge..."
pkill -f "foxglove_bridge" 2>/dev/null

# Kill QGroundControl
echo "  - Stopping QGroundControl..."
pkill -f "QGroundControl.AppImage" 2>/dev/null

# Kill GStreamer processes
echo "  - Stopping gst-launch..."
pkill -f "gst-launch-1.0" 2>/dev/null

# Kill VLM inference node
echo "  - Stopping vision_inference_node_refactored..."
pkill -f "vision_inference_node_refactored.py" 2>/dev/null

# Kill video merger node
echo "  - Stopping video_merger_node..."
pkill -f "video_merger_node.py" 2>/dev/null

echo "Process cleanup complete."
sleep 2

# Kill existing session if it exists
echo "Clear existing tmux session..."
tmux kill-session -t $SESSION_NAME 2>/dev/null
sleep 1

# Create new tmux session with first pane
tmux new-session -d -s $SESSION_NAME

# Split the window to create 5 panes
# First, split horizontally to create left and right halves
tmux split-window -h

# Split the left pane vertically
tmux select-pane -t 0
tmux split-window -v

# Split the right pane vertically
tmux select-pane -t 2
tmux split-window -v

# Split one of the right panes to create the 5th pane
tmux select-pane -t 3
tmux split-window -v

# Now we have 5 panes, let's assign commands to each
# Pane 0: Foxglove (with clientPublish capability enabled)
tmux select-pane -t 0
tmux send-keys "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=100 && source $PROJECT_ROOT/airstack/ros_ws/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p capabilities:='[clientPublish]'" Enter

# Pane 1: QGroundControl
tmux select-pane -t 1
tmux send-keys "./Downloads/QGroundControl.AppImage" Enter

# Pane 2: RTSP Stream (Remote) - v4l2 camera capture and H264 encode
tmux select-pane -t 2
tmux send-keys "bash dtc.sh" Enter
sleep 5

# Camera capture with hardware encoding to multiple destinations:
# tmux send-keys "gst-launch-1.0 -e v4l2src device=/dev/video4 ! video/x-raw,format=YUY2,width=1280,height=720,framerate=15/1 ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! nvv4l2h264enc maxperf-enable=1 control-rate=1 bitrate=4000000 iframeinterval=15 idrinterval=15 insert-sps-pps=true preset-level=1 ! h264parse ! rtph264pay config-interval=1 pt=96 ! multiudpsink clients=10.3.1.10:5000,10.3.1.10:5600 sync=false async=false" Enter
# tmux send-keys "gst-launch-1.0 -e v4l2src device=/dev/video4 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! nvv4l2h264enc maxperf-enable=1 control-rate=1 bitrate=4000000 iframeinterval=30 idrinterval=30 insert-sps-pps=true preset-level=1 ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=239.255.0.1 port=5000 auto-multicast=true ttl=10 ttl-mc=10 sync=false async=false" Enter

#30fps to reduce motion blur, down to 5fps to reduce bandwidth usage

# EXPERIMENTAL NOTES 1: 30fps to reduce motion blur, down to 5fps to reduce bandwidth usage


# EXPERIMENTAL NOTES 2.1: multicast to 239.255.0.1:5000
# EXPERIMENTAL NOTES 2.2: 
  # For testing 5fps multicast stream
  # set mode="auto", multicast_enabled=true in camera_config.yaml

# Sender command for multicast to 239.255.0.1:5000
# gst-launch-1.0 -e v4l2src device=/dev/video4 ! \
#   video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! \
#   videorate ! video/x-raw,format=YUY2,framerate=5/1 ! \
#   nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! \
#   nvv4l2h264enc maxperf-enable=1 control-rate=1 bitrate=1500000 iframeinterval=5 idrinterval=5 insert-sps-pps=true preset-level=1 ! \
#   h264parse ! rtph264pay config-interval=1 pt=96 ! \
#   udpsink host=239.255.0.1 port=5000 auto-multicast=true ttl=10 ttl-mc=10 sync=false async=false

# EXPERIMENTAL NOTES:
  # For testing 5fps unicast stream
  # set mode="auto", multicast_enabled=false in camera_config.yaml

## command for sender unicast to 10.3.1.10:5000


######### 1920 1080 ###############
tmux send-keys " gst-launch-1.0 -e v4l2src device=/dev/video4 ! \
  video/x-raw,format=YUY2,width=1920,height=1080,framerate=8/1 ! \
  videorate ! video/x-raw,format=YUY2,framerate=4/1 ! \
  nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! \
  nvv4l2h264enc maxperf-enable=1 control-rate=1 bitrate=2500000 iframeinterval=3 idrinterval=1 insert-sps-pps=true preset-level=1 ! \
  h264parse ! rtph264pay config-interval=1 pt=96 ! \
  udpsink host=10.3.1.10 port=5000 sync=false async=false" Enter

####### 1280 720 ###############
####### If see packet loss / congestion, drop back to 1500000 ###############
####### drop frame rate to 4/1 ###############
# tmux send-keys " gst-launch-1.0 -e v4l2src device=/dev/video4 ! \
#   video/x-raw,format=YUY2,width=1280,height=720,framerate=15/1 ! \
#   videorate ! video/x-raw,format=YUY2,framerate=5/1 ! \
#   nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! \
#   nvv4l2h264enc maxperf-enable=1 control-rate=1 bitrate=2000000 iframeinterval=10 idrinterval=10 insert-sps-pps=true preset-level=1 ! \
#   h264parse ! rtph264pay config-interval=1 pt=96 ! \
#   udpsink host=10.3.1.10 port=5000 sync=false async=false" Enter

####### 640 480 ###############
# tmux send-keys " gst-launch-1.0 -e v4l2src device=/dev/video4 ! \
#   video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! \
#   videorate ! video/x-raw,format=YUY2,framerate=5/1 ! \
#   nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! \
#   nvv4l2h264enc maxperf-enable=1 control-rate=1 bitrate=1500000 iframeinterval=10 idrinterval=10 insert-sps-pps=true preset-level=1 ! \
#   h264parse ! rtph264pay config-interval=1 pt=96 ! \
#   udpsink host=10.3.1.10 port=5000 sync=false async=false" Enter


# 
### MOTION BLUR MITIGATION ###
# Higher Framerate 
#tmux send-keys "gst-launch-1.0 -e v4l2src device=/dev/video4 ! video/x-raw,format=YUY2,width=424,height=240,framerate=60/1 ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! nvv4l2h264enc maxperf-enable=1 control-rate=1 bitrate=4000000 iframeinterval=60 idrinterval=60 insert-sps-pps=true preset-level=1 ! h264parse ! rtph264pay config-interval=1 pt=96 ! multiudpsink clients=10.3.1.10:5000,10.3.1.10:5600 sync=false async=false" Enter



# Pane 3: Local Receiver
tmux select-pane -t 3

sleep 5  # Wait for remote stream to start
tmux send-keys "export ROS_DOMAIN_ID=100 && source /opt/ros/humble/setup.bash && ros2 run web_video_server web_video_server" Enter
# http://localhost:8080/stream?topic=/vlm_geolocator/debug/camera_feed
# tmux send-keys "gst-launch-1.0 -v udpsrc port=5000 caps=\"application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,payload=96\" ! rtpjitterbuffer latency=50 ! rtph264depay ! queue max-size-buffers=1 leaky=downstream ! h264parse ! nvh264dec ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! glimagesink sync=false" Enter
# tmux send-keys "gst-launch-1.0 -v \
#   udpsrc address=239.255.0.1 port=5000 multicast-iface=enx00e04c6802a8 buffer-size=4194304 caps=\"application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,payload=96\" auto-multicast=true ! \
#   rtpjitterbuffer latency=200 ! \
#   rtph264depay ! h264parse ! nvh264dec ! queue ! \
#   videoconvert ! fpsdisplaysink video-sink=glimagesink text-overlay=true sync=false" Enter
  
# try2-local-receiver (only in camera_config.yaml of vlm_geolocator)


# Pane 4: VLM Geolocator (Refactored) - Domain 100, publishes to /casualty_geolocated
tmux select-pane -t 4
sleep 5
tmux send-keys "cd $PROJECT_ROOT/vlm_geolocator" Enter
sleep 1
tmux send-keys "export ROS_DOMAIN_ID=100 && source /opt/ros/humble/setup.bash && source $PROJECT_ROOT/airstack/ros_ws/install/setup.bash && python3 src/vision_inference_node_refactored.py" Enter

# Add pane titles (requires tmux 3.0+)
tmux select-pane -t 0 -T "Foxglove"
tmux select-pane -t 1 -T "QGroundControl"
tmux select-pane -t 2 -T "RTSP Stream"
tmux select-pane -t 3 -T "Local Receiver"
tmux select-pane -t 4 -T "VLM (D100→/casualty_geolocated)"

# Enable pane titles display
tmux set -g pane-border-status top
tmux set -g pane-border-format "#{pane_index}: #{pane_title}"

# Create a NEW WINDOW (not pane) for Video Merger Node
tmux new-window -t $SESSION_NAME -n "VideoMerger"

# Single pane: Video Merger Node
tmux send-keys "cd $PROJECT_ROOT/vlm_geolocator" Enter
sleep 1
tmux send-keys "export ROS_DOMAIN_ID=100 && source /opt/ros/humble/setup.bash && source $PROJECT_ROOT/airstack/ros_ws/install/setup.bash && python3 src/video_merger_node.py" Enter

# Add pane title for Window 1 (VideoMerger)
tmux select-pane -t 0 -T "Video Merger"

# Create a NEW WINDOW for HTML Viewer
tmux new-window -t $SESSION_NAME -n "HTMLViewer"

# Single pane: HTML Viewer (Watch Mode)
tmux send-keys "cd $PROJECT_ROOT/vlm_geolocator" Enter
sleep 1
tmux send-keys "python3 gemini_result_viewer_html.py --watch --interval 5" Enter

# Add pane title for Window 2 (HTMLViewer)
tmux select-pane -t 0 -T "HTML Viewer (Watch Mode)"

# Switch back to main window (Window 0) and first pane
tmux select-window -t 0
tmux select-pane -t 0

# Attach to the session
tmux attach-session -t $SESSION_NAME

echo "Tmux session '$SESSION_NAME' started with all processes."
echo ""
echo "📺 WINDOW 0 (Main) - 5 Panes:"
echo "  - Pane 0: Foxglove Bridge"
echo "  - Pane 1: QGroundControl"
echo "  - Pane 2: RTSP Stream (Remote Camera)"
echo "  - Pane 3: Local Receiver"
echo "  - Pane 4: VLM Geolocator"
echo ""
echo "📺 WINDOW 1 (VideoMerger) - 1 Pane:"
echo "  - Pane 0: Video Merger Node"
echo ""
echo "📺 WINDOW 2 (HTMLViewer) - 1 Pane:"
echo "  - Pane 0: HTML Viewer (Watch Mode)"
echo ""
echo "⌨️  Navigation:"
echo "  - 'Ctrl+b' then arrow keys to switch between panes"
echo "  - 'Ctrl+b' then 'w' to show window list"
echo "  - 'Ctrl+b' then '0/1/2' to jump to specific window"
echo "  - 'Ctrl+b' then 'd' to detach from session"
echo "  - 'tmux attach -t $SESSION_NAME' to reattach later"
