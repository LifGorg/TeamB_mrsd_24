# 调试图像查看指南

## 概述

VLM Geolocator 现在通过 ROS2 标准图像话题发布调试视频流，而不是使用 `cv2.imshow()`。这避免了 Qt 线程问题，并提供更好的集成体验。

## 配置

### 启用调试图像发布

编辑 `config/system_config.yaml`:

```yaml
debug_display:
  enabled: true  # 启用图像发布
  display_interval: 6  # 每6帧发布一次（降低带宽）
```

**说明：**
- `enabled: true` - 启动图像发布到 `/vlm_geolocator/debug/camera_feed`
- `display_interval` - 控制发布频率（值越大，带宽越小）
  - `1` = 每帧都发布（30fps，高带宽）
  - `6` = 每6帧发布一次（~5fps，推荐）
  - `30` = 每30帧发布一次（~1fps，低带宽）

## 查看方法

### 方法 1: Foxglove Studio（推荐）

**优点：** 强大的可视化、录制、回放功能

**步骤：**

1. 启动 Foxglove Studio（你们已经在用）

2. 添加图像面板：
   - 点击左侧 `+` 添加面板
   - 选择 `Image`
   - 在面板设置中，选择话题：`/vlm_geolocator/debug/camera_feed`

3. 功能：
   - 实时查看视频流
   - 显示帧信息叠加层
   - 可缩放、暂停
   - 可录制为 `.mcap` 文件

**截图：**
```
┌─────────────────────────────────────┐
│ Foxglove Studio                     │
├─────────────────────────────────────┤
│ ┌─────────────┐  ┌─────────────┐   │
│ │   3D View   │  │  Image View │   │
│ │             │  │             │   │
│ │             │  │ Frame: 1234 │   │
│ │             │  │             │   │
│ └─────────────┘  └─────────────┘   │
│ Topics:                             │
│  • /vlm_geolocator/debug/camera_feed│
│  • /mavros/global_position/global   │
└─────────────────────────────────────┘
```

---

### 方法 2: rqt_image_view

**优点：** 轻量、ROS 原生

**步骤：**

```bash
# 确保节点正在运行
ros2 run vlm_geolocator vision_inference_node_refactored

# 在另一个终端启动 rqt_image_view
rqt_image_view /vlm_geolocator/debug/camera_feed
```

**或使用 rqt 完整工具：**

```bash
rqt
# 然后在菜单: Plugins → Visualization → Image View
# 选择话题: /vlm_geolocator/debug/camera_feed
```

---

### 方法 3: RViz2

**优点：** 可与3D可视化集成

**步骤：**

```bash
rviz2
```

1. 点击 `Add` 按钮
2. 选择 `By topic` → `/vlm_geolocator/debug/camera_feed` → `Image`
3. 调整显示大小和位置

---

### 方法 4: 命令行检查

**检查话题是否在发布：**

```bash
# 列出所有话题
ros2 topic list | grep debug

# 应该看到：
# /vlm_geolocator/debug/camera_feed

# 查看话题信息
ros2 topic info /vlm_geolocator/debug/camera_feed

# 输出：
# Type: sensor_msgs/msg/Image
# Publisher count: 1
# Subscription count: 0 (或更多，取决于查看器)

# 查看实时频率
ros2 topic hz /vlm_geolocator/debug/camera_feed

# 输出：
# average rate: 5.023
#   min: 0.198s max: 0.202s std dev: 0.00143s window: 10
```

**查看单帧：**

```bash
ros2 topic echo /vlm_geolocator/debug/camera_feed --once
```

---

## 性能调优

### 带宽使用估算

```
分辨率: 640x480 RGB (3 channels)
每帧大小: 640 * 480 * 3 = 921,600 bytes ≈ 900 KB

带宽 = 帧大小 * 发布频率

示例:
- 30 fps (interval=1): 900 KB * 30 = 27 MB/s  ❌ 太高
- 5 fps  (interval=6): 900 KB * 5  = 4.5 MB/s ✅ 推荐
- 1 fps  (interval=30): 900 KB * 1  = 0.9 MB/s ✅ 低带宽
```

### 推荐设置

| 场景 | `display_interval` | 发布频率 | 带宽 |
|------|-------------------|---------|------|
| 本地开发 | 3-6 | 5-10 fps | 4-9 MB/s |
| 远程查看 | 10-15 | 2-3 fps | 1.8-2.7 MB/s |
| 低带宽 | 30 | 1 fps | 0.9 MB/s |
| 禁用 | N/A (enabled: false) | 0 fps | 0 MB/s |

---

## 故障排除

### 问题 1: 话题不存在

```bash
$ ros2 topic list | grep debug
# （无输出）
```

**解决：**
1. 检查 `system_config.yaml` 中 `debug_display.enabled: true`
2. 重启节点
3. 检查节点日志：应该看到 `📷 Debug image publisher enabled`

---

### 问题 2: 图像黑屏或不更新

**可能原因：**
- GStreamer 管道未接收到视频
- 相机未连接
- 帧率设置过低

**检查：**
```bash
# 检查帧接收
ros2 topic echo /vlm_geolocator/debug/camera_feed --once

# 查看节点日志
# 应该看到: "✓ Received 1000 frames" 等消息
```

---

### 问题 3: Foxglove 显示延迟

**优化：**
1. 降低 `display_interval`（但会增加带宽）
2. 检查网络延迟（如果远程）
3. 在 Foxglove 中调整缓冲设置

---

### 问题 4: ROS Domain 不匹配

如果在不同的 ROS_DOMAIN_ID 运行：

```bash
# 查看节点运行的 domain
echo $ROS_DOMAIN_ID

# 设置相同的 domain
export ROS_DOMAIN_ID=0  # 或你的节点使用的值

# 然后启动查看器
rqt_image_view /vlm_geolocator/debug/camera_feed
```

---

## 录制和回放

### 使用 ros2 bag 录制

```bash
# 仅录制调试图像
ros2 bag record /vlm_geolocator/debug/camera_feed

# 录制所有话题（包括图像）
ros2 bag record -a

# 录制特定话题组合
ros2 bag record \
  /vlm_geolocator/debug/camera_feed \
  /mavros/global_position/global \
  /casualty_geolocated
```

### 回放

```bash
# 回放录制
ros2 bag play <bag_file>

# 然后在 Foxglove/rqt_image_view 中查看
```

---

## 迁移说明

### 从旧的 cv2.imshow() 迁移

**之前：**
```python
cv2.imshow("窗口名", frame)
cv2.waitKey(1)
```

**现在：**
```python
# 自动完成！只需：
# 1. 启用 debug_display.enabled: true
# 2. 在 Foxglove 或 rqt_image_view 中查看
```

**优点：**
- ✅ 无 Qt 线程错误
- ✅ 可远程查看
- ✅ 可录制回放
- ✅ 更好的 UI（Foxglove）
- ✅ 不影响主线程性能
- ✅ 可在无头服务器运行

---

## 高级用法

### 自定义图像处理

如果需要自定义叠加层，修改 `vision_inference_node_refactored.py`:

```python:296:309:vlm_geolocator/src/vision_inference_node_refactored.py
                try:
                    # 添加帧信息叠加
                    debug_frame = frame.copy()
                    if cv2 is not None:
                        cv2.putText(
                            debug_frame,
                            f"Frame: {frame_count}",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0),
                            2
                        )
```

### 发布多个调试视频流

```python
# 在 VisionInferenceNode.__init__() 中添加
self.detection_image_pub = self.create_publisher(
    Image,
    '/vlm_geolocator/debug/detections',
    10
)

# 在检测后发布带边界框的图像
annotated_frame = self._draw_detections(frame, detections)
self.detection_image_pub.publish(
    self.cv_bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
)
```

---

## 总结

| 特性 | 旧方法 (cv2.imshow) | 新方法 (ROS话题) |
|------|---------------------|------------------|
| 线程安全 | ❌ Qt 错误 | ✅ 完全安全 |
| 远程查看 | ❌ 仅本地 | ✅ 支持 |
| 录制回放 | ❌ 需要额外工具 | ✅ ros2 bag |
| 性能影响 | ⚠️  主线程阻塞 | ✅ 最小影响 |
| UI 质量 | ⚠️  基础 | ✅ Foxglove 强大 |
| 无头服务器 | ❌ 不支持 | ✅ 支持 |

**推荐工作流程：**
1. 开发时：`enabled: true` + Foxglove Studio
2. 生产时：`enabled: false`（节省带宽）
3. 调试时：按需启用，使用 rqt_image_view 快速检查

---

**相关文档：**
- [OpenCV Display Diagnosis](OPENCV_DISPLAY_DIAGNOSIS.md) - 问题诊断详情
- [Foxglove Documentation](https://foxglove.dev/docs)
- [ROS2 Image Transport](https://github.com/ros-perception/image_common/tree/rolling/image_transport)

