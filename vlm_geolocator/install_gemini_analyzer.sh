#!/bin/bash

# Gemini 视频分析器快速安装脚本

echo "========================================"
echo "Gemini 视频分析器 - 安装脚本"
echo "========================================"
echo ""

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误: 未找到 Python3"
    exit 1
fi

echo "✅ Python3 已安装: $(python3 --version)"

# 安装依赖
echo ""
echo "正在安装依赖..."
pip install google-genai --quiet

if [ $? -eq 0 ]; then
    echo "✅ google-genai 安装成功"
else
    echo "❌ google-genai 安装失败"
    exit 1
fi

# 检查 API 密钥
echo ""
if [ -z "$GEMINI_API_KEY" ]; then
    echo "⚠️  警告: GEMINI_API_KEY 环境变量未设置"
    echo ""
    echo "请执行以下命令设置 API 密钥:"
    echo "  export GEMINI_API_KEY='your-api-key-here'"
    echo ""
    echo "获取 API 密钥: https://aistudio.google.com/app/apikey"
else
    echo "✅ GEMINI_API_KEY 已设置 (${GEMINI_API_KEY:0:10}...)"
fi

echo ""
echo "========================================"
echo "安装完成!"
echo "========================================"
echo ""
echo "使用方法:"
echo ""
echo "1. 设置 API 密钥 (如果还没设置):"
echo "   export GEMINI_API_KEY='your-api-key-here'"
echo ""
echo "2. 运行测试脚本:"
echo "   cd /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator"
echo "   python test_gemini_analyzer.py"
echo ""
echo "3. 或直接使用分析器:"
echo "   python src/vlm_geolocator/vision/gemini_video_analyzer.py /path/to/video.mp4"
echo ""
echo "详细文档: GEMINI_VIDEO_ANALYZER_README.md"
echo ""


