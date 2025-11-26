#!/bin/bash
# 测试 /trigger_record 主题功能的脚本

echo "🎥 测试视频录制触发器..."
echo ""
echo "发送触发信号到 /trigger_record..."

# 发布触发消息（true = 开始录制）
ros2 topic pub --once /trigger_record std_msgs/msg/Bool "{data: true}"

echo ""
echo "✅ 触发信号已发送！"
echo ""
echo "检查节点日志，应该能看到："
echo "  - 🎥 Video recording triggered (duration=5s, min_fps=2.0)"
echo "  - 📹 Recording 5s video ..."
echo ""
echo "录制的视频将保存在："
echo "  /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator/inference_logs/recordings/"
echo ""




