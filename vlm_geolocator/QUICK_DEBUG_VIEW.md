# 快速调试视频查看指南

## ✅ 推荐方案：web_video_server

**最佳选择** - 低 CPU、低延迟、浏览器原生支持

### 安装

```bash
sudo apt install ros-humble-web-video-server
```

### 使用（3步）

```bash
# 1. 确保在正确的 ROS Domain
export ROS_DOMAIN_ID=100  # 或你的节点运行的 domain

# 2. 启动 web_video_server
ros2 run web_video_server web_video_server

# 3. 在浏览器打开
```

**URL：** http://localhost:8080/stream?topic=/vlm_geolocator/debug/camera_feed

### 性能特性

| 特性 | 性能 |
|------|------|
| CPU 占用 | 很低（~5-10% 单核） |
| 延迟 | 低（~100-200ms） |
| 带宽 | MJPEG 自动压缩 |
| 画质 | 可调（默认良好） |

---

## ❌ 不推荐的方案

### Foxglove Studio
- ❌ **问题**：高 CPU 占用（14 核 80%）
- ❌ **原因**：rosbridge + 多层解码 + Electron 渲染
- ⚠️ **仅适合**：远程查看（不同机器）

### OpenCV GUI (cv2.imshow / image_view)
- ❌ **问题**：Qt 线程冲突，窗口无法显示
- ❌ **症状**：黑屏、冻结、QBasicTimer 错误
- ⚠️ **不可用**：在 GStreamer + ROS2 架构下

---

## 配置调整

编辑 `config/system_config.yaml`:

```yaml
debug_display:
  enabled: true
  display_interval: 1   # 1=全速30fps, 10=3fps, 30=1fps
```

**推荐值：**
- 本地调试：`1` 或 `3`（流畅）
- 持续监控：`10`（节省 CPU）
- 低带宽：`30`（最低开销）

---

## 故障排除

### 浏览器显示"Waiting for image"

```bash
# 检查话题
export ROS_DOMAIN_ID=100
ros2 topic list | grep camera_feed
ros2 topic hz /vlm_geolocator/debug/camera_feed

# 应该看到：
# /vlm_geolocator/debug/camera_feed
# average rate: ~3.0 (取决于 display_interval)
```

### Domain 不匹配

```bash
# 确认节点运行的 domain
ps aux | grep vision_inference_node

# 设置相同的 domain
export ROS_DOMAIN_ID=<节点的domain>
ros2 run web_video_server web_video_server
```

### 端口被占用

```bash
# 检查端口
lsof -i :8080

# 使用其他端口
ros2 run web_video_server web_video_server --ros-args -p port:=8081

# 访问：http://localhost:8081/stream?topic=/vlm_geolocator/debug/camera_feed
```

---

## 高级选项

### 调整压缩质量

在浏览器 URL 中添加参数：

```
http://localhost:8080/stream?topic=/vlm_geolocator/debug/camera_feed&quality=80
```

质量范围：1-100（默认 80）
- `quality=50`：低画质，低带宽
- `quality=90`：高画质，高带宽

### 查看其他格式

```
# MJPEG（推荐）
http://localhost:8080/stream?topic=/vlm_geolocator/debug/camera_feed&type=mjpeg

# VP8（更高压缩）
http://localhost:8080/stream?topic=/vlm_geolocator/debug/camera_feed&type=vp8

# PNG（无损，但很慢）
http://localhost:8080/snapshot?topic=/vlm_geolocator/debug/camera_feed
```

---

## 对比总结

| 工具 | CPU | 延迟 | 易用性 | 推荐度 |
|------|-----|------|--------|--------|
| **web_video_server** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ✅ **推荐** |
| Foxglove | ⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⚠️ 远程用 |
| image_view | N/A | N/A | N/A | ❌ 不可用 |
| cv2.imshow | N/A | N/A | N/A | ❌ 不可用 |

---

**相关文档：**
- [完整诊断报告](OPENCV_DISPLAY_DIAGNOSIS.md)
- [详细查看指南](DEBUG_IMAGE_VIEWING_GUIDE.md)




