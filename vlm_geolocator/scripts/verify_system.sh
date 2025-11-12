#!/bin/bash
# System verification script - Check all dependencies and configuration

set -e

echo "========================================"
echo "🔍 VLM Geolocator System Verification"
echo "========================================"
echo ""

ERRORS=0
WARNINGS=0

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

check_pass() {
    echo -e "${GREEN}✓${NC} $1"
}

check_fail() {
    echo -e "${RED}✗${NC} $1"
    ERRORS=$((ERRORS + 1))
}

check_warn() {
    echo -e "${YELLOW}⚠${NC} $1"
    WARNINGS=$((WARNINGS + 1))
}

# 1. Check ROS2
echo "1️⃣  Checking ROS2..."
if command -v ros2 &> /dev/null; then
    ROS_VERSION=$(ros2 --version 2>&1 | grep -o 'humble' || echo "unknown")
    if [ "$ROS_VERSION" = "humble" ]; then
        check_pass "ROS2 Humble installed"
    else
        check_warn "ROS2 version is not Humble"
    fi
else
    check_fail "ROS2 not installed"
fi
echo ""

# 2. Check Python
echo "2️⃣  Checking Python..."
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version | grep -oP '\d+\.\d+' | head -1)
    MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
    MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)
    if [ "$MAJOR" -ge 3 ] && [ "$MINOR" -ge 8 ]; then
        check_pass "Python ${PYTHON_VERSION} (requires >= 3.8)"
    else
        check_fail "Python version too low: ${PYTHON_VERSION} (requires >= 3.8)"
    fi
else
    check_fail "Python3 not installed"
fi
echo ""

# 3. Check Python dependencies
echo "3️⃣  Checking Python dependencies..."
REQUIRED_PACKAGES=("numpy" "cv2" "scipy" "yaml" "torch" "transformers" "PIL")
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if python3 -c "import $pkg" 2>/dev/null; then
        check_pass "$pkg installed"
    else
        check_fail "$pkg not installed"
    fi
done
echo ""

# 4. Check GStreamer
echo "4️⃣  Checking GStreamer..."
if command -v gst-launch-1.0 &> /dev/null; then
    check_pass "GStreamer installed"
else
    check_fail "GStreamer not installed"
fi
echo ""

# 5. Check CUDA (optional)
echo "5️⃣  Checking CUDA (optional)..."
if command -v nvidia-smi &> /dev/null; then
    CUDA_VERSION=$(nvidia-smi | grep -oP 'CUDA Version: \K[0-9.]+' || echo "unknown")
    check_pass "CUDA ${CUDA_VERSION} available"
else
    check_warn "CUDA not available (will use CPU)"
fi
echo ""

# 6. Check directory structure
echo "6️⃣  Checking directory structure..."
PROJECT_DIR="/home/triage/vlm_geolocator"
cd "$PROJECT_DIR"

REQUIRED_DIRS=("config" "src" "scripts" "docs")
for dir in "${REQUIRED_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        check_pass "Directory exists: $dir/"
    else
        check_fail "Directory missing: $dir/"
    fi
done
echo ""

# 7. Check configuration files
echo "7️⃣  Checking configuration files..."
CONFIG_FILES=("config/camera_config.yaml" "config/ros_config.yaml" "config/system_config.yaml")
for file in "${CONFIG_FILES[@]}"; do
    if [ -f "$file" ]; then
        check_pass "Configuration file exists: $file"
    else
        check_fail "Configuration file missing: $file"
    fi
done
echo ""

# 8. Check main modules
echo "8️⃣  Checking main modules..."
MODULES=(
    "src/vlm_geolocator/core/config.py"
    "src/vlm_geolocator/sensors/sensor_manager.py"
    "src/vlm_geolocator/gps/calculator.py"
    "src/vlm_geolocator/vision/video_receiver.py"
    "src/vlm_geolocator/vision/isaac_detector.py"
)
for module in "${MODULES[@]}"; do
    if [ -f "$module" ]; then
        check_pass "Module exists: $(basename $module)"
    else
        check_fail "Module missing: $module"
    fi
done
echo ""

# 9. Check model path
echo "9️⃣  Checking model path..."
MODEL_PATH=$(grep -A 2 "model:" config/system_config.yaml | grep "path:" | awk '{print $2}' | tr -d '"')
if [ -d "$MODEL_PATH" ]; then
    check_pass "Model directory exists: $MODEL_PATH"
else
    check_warn "Model directory does not exist: $MODEL_PATH"
fi
echo ""

# 10. Check ROS workspace
echo "🔟 Checking ROS workspace..."
ROS_WS="/home/triage/georgia_dtc_ops_team_chiron_mrsd/uav_to_ugv/clean_workspace/ros_ws"
if [ -d "$ROS_WS/install" ]; then
    check_pass "ROS workspace exists: $ROS_WS"
else
    check_warn "ROS workspace does not exist: $ROS_WS"
fi
echo ""

# Summary
echo "========================================"
echo "📊 Verification Summary"
echo "========================================"
if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo -e "${GREEN}✅ All checks passed! System is ready to use.${NC}"
    exit 0
elif [ $ERRORS -eq 0 ]; then
    echo -e "${YELLOW}⚠️  Found ${WARNINGS} warnings, but system should run.${NC}"
    exit 0
else
    echo -e "${RED}❌ Found ${ERRORS} errors and ${WARNINGS} warnings.${NC}"
    echo ""
    echo "Please resolve the issues above before running the system."
    echo ""
    echo "Recommendations:"
    echo "  - Install missing dependencies: pip install -r requirements.txt"
    echo "  - Check if configuration files exist"
    echo "  - Verify model path is correct"
    exit 1
fi
