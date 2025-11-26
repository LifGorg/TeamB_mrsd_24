# 视频帧显示功能使用指南

## 概述
系统现在支持可选的实时视频帧显示功能，用于调试和监控。该功能专为**单播（unicast）**场景设计，不会与主节点竞争 UDP 流。

## 为什么这个方案适合单播？

在单播场景下：
- ❌ **方案2（独立脚本）**: 无法工作，因为 UDP 单播端口只能被一个进程监听
- ✅ **方案1（共享 pipeline）**: 完美适配，在同一进程内共享 GStreamer 管道，无需重复接收/解码

## 功能特点

1. **默认关闭**: 不影响生产环境性能
2. **可配置间隔**: 只显示部分帧，降低 CPU 占用
3. **优雅降级**: 如果 OpenCV 不可用，自动禁用显示
4. **帧计数叠加**: 显示当前接收到的帧编号
5. **共享资源**: 使用同一个视频流，不会重复解码

## 如何启用

### 1. 编辑配置文件

打开 `vlm_geolocator/config/system_config.yaml`，找到 `debug_display` 部分：

```yaml
debug_display:
  enabled: true  # 设置为 true 启用显示
  display_interval: 30  # 每 30 帧显示一次（调整以控制性能影响）
  window_name: "VLM Geolocator - Video Feed"
```

### 2. 性能建议

根据您的需求选择 `display_interval`:

| 场景 | 推荐值 | CPU 影响 |
|------|--------|----------|
| 快速调试 | 1 | 高 |
| 常规监控 | 15-30 | 低 |
| 偶尔检查 | 60+ | 极低 |

### 3. 运行节点

正常启动 ROS2 节点：

```bash
ros2 run vlm_geolocator vision_inference_node
```

如果启用了显示，您会看到：
```
[VideoReceiver] Display enabled: showing every 30 frame(s)
```

一个 OpenCV 窗口会打开显示视频流。

## 性能影响分析

### 单播模式（当前配置）
- **方案1（实现的）**: 
  - ✅ 共享同一个 GStreamer pipeline
  - ✅ 只解码一次视频
  - ✅ 显示操作非常轻量（仅 cv2.imshow + waitKey）
  - ✅ 通过 display_interval 控制 CPU 占用
  
- **方案2（未实现，不适用）**:
  - ❌ 需要单独的 UDP 接收器
  - ❌ 无法监听同一个单播端口
  - ❌ 会与主节点冲突

### 资源占用估算

假设 30fps 视频流，display_interval=30：
- 显示频率: 1 FPS
- 每帧开销: ~2-5ms (cv2.imshow + putText)
- 总 CPU 影响: < 0.5%

## 故障排除

### 显示窗口未出现
- 检查 `enabled: true` 是否设置
- 确认 OpenCV 已安装: `pip install opencv-python`
- 查看日志是否有 "Display enabled" 消息

### 显示影响性能
- 增加 `display_interval` 值（如 60 或更高）
- 或直接设置 `enabled: false` 关闭显示

### 窗口卡住不响应
- 这是正常的，cv2.waitKey(1) 只处理事件 1ms
- 主处理循环不受影响

## 使用建议

1. **开发/调试时**: `enabled: true`, `display_interval: 1-15`
2. **现场监控时**: `enabled: true`, `display_interval: 30-60`
3. **生产运行时**: `enabled: false`

## 技术细节

实现位置:
- 配置: `vlm_geolocator/config/system_config.yaml`
- 配置解析: `vlm_geolocator/src/vlm_geolocator/core/config.py`
- 显示逻辑: `vlm_geolocator/src/vlm_geolocator/vision/video_receiver.py`
- 主节点集成: `vlm_geolocator/src/vision_inference_node_refactored.py`

显示在 `_on_new_frame()` 回调中执行，与主处理流程串行，不会产生竞态条件。


