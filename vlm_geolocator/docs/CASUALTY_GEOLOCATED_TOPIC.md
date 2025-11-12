# NavSatFix 主题发布说明

## 概述

系统现在可以将检测到的伤员位置发布到 `/casualty_geolocated` 主题，消息类型为 `sensor_msgs/msg/NavSatFix`。

## 修改内容

### 1. 发布器管理器 (`src/vlm_geolocator/ros_interface/publishers.py`)

添加了新的方法：
- `create_casualty_geolocated_publisher()`: 创建NavSatFix类型的发布器
- `publish_casualty_geolocated()`: 发布NavSatFix消息

### 2. ROS配置文件 (`config/ros_config.yaml`)

在 `output_topics` 中添加了：
```yaml
casualty_geolocated: "/casualty_geolocated"
```

### 3. 视觉推理节点 (`src/vision_inference_node_refactored.py`)

- 在初始化时创建 `/casualty_geolocated` 发布器
- 在检测到目标并计算GPS坐标后，同时发布到两个主题：
  - `/manual_targets/geolocated` (HumanDataMsg类型 - 包含图像数据)
  - `/casualty_geolocated` (NavSatFix类型 - 仅GPS坐标)

## 消息格式

### NavSatFix 消息结构

```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id: "map"
sensor_msgs/NavSatStatus status
  int8 status: 0 (STATUS_FIX)
  uint16 service: 1 (SERVICE_GPS)
float64 latitude          # 纬度 (度)
float64 longitude         # 经度 (度)
float64 altitude          # 海拔高度 (米)
float64[9] position_covariance
uint8 position_covariance_type: 0
```

## 使用方法

### 启动系统

```bash
cd /home/triage/vlm_geolocator
./scripts/start_system.sh
```

### 触发检测

两种方式：

1. 通过服务调用：
```bash
ros2 service call /trigger_capture std_srvs/srv/Trigger
```

2. 通过主题发布：
```bash
ros2 topic pub --once /trigger_capture std_msgs/msg/Bool "{data: true}"
```

### 监听伤员位置消息

```bash
# 监听NavSatFix格式
ros2 topic echo /casualty_geolocated

# 查看发布频率
ros2 topic hz /casualty_geolocated

# 查看消息详细信息
ros2 topic info /casualty_geolocated -v
```

### 示例输出

```yaml
header:
  stamp:
    sec: 1730846789
    nanosec: 123456789
  frame_id: map
status:
  status: 0
  service: 1
latitude: 40.443345
longitude: -79.943456
altitude: 320.5
position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
position_covariance_type: 0
```

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│         Vision Inference Node (ROS2 Domain 100)            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Video Stream → Detection → GPS Calculation → Publishing   │
│                                                             │
│  Publishes to:                                              │
│  ✓ /manual_targets/geolocated (HumanDataMsg)              │
│  ✓ /casualty_geolocated (NavSatFix)      [新增]           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 日志示例

检测到目标时的日志输出：

```
[INFO] 📸 Capture triggered via service
[INFO] 📷 Captured frame #12345, size: (1920, 1080, 3), age: 0.023s
[INFO] 🔄 Running inference (frame #12345)...
[INFO] ✅ Detected 2 targets
[INFO]   Target 1: (850.3, 540.2)
[INFO]   Target 1: GPS (40.443345, -79.943456), distance: 85.3m
[INFO]   Target 2: (1100.8, 620.5)
[INFO]   Target 2: GPS (40.443298, -79.943501), distance: 92.1m
[INFO] ✅ Successfully processed 2/2 targets
```

## 依赖关系

- ROS2 Humble
- sensor_msgs (标准ROS2包)
- 其他依赖见 `requirements.txt`

## 注意事项

1. `/casualty_geolocated` 主题使用与MAVROS相同的QoS配置（best_effort, volatile）
2. 每检测到一个目标就会发布一条消息
3. `altitude` 字段使用无人机的相对高度（传感器提供）
4. `frame_id` 设置为 "map"
5. GPS坐标系统使用WGS84

## 调试

查看系统日志：
```bash
tail -f /tmp/vision_inference.log
```

查看所有活跃主题：
```bash
ros2 topic list
```

验证主题类型：
```bash
ros2 topic type /casualty_geolocated
# 应输出: sensor_msgs/msg/NavSatFix
```

