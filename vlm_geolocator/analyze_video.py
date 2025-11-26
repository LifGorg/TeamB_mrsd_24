#!/usr/bin/env python3
"""
简单的命令行工具 - 分析任意视频
使用方法: python3 analyze_video.py <视频路径>
"""

import os
import sys
import json

# 添加src到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from vlm_geolocator.vision.gemini_video_analyzer import GeminiVideoAnalyzer


def get_api_key():
    """获取API密钥"""
    # 尝试从环境变量获取
    api_key = os.environ.get("GEMINI_API_KEY")
    
    # 如果环境变量没有，尝试从文件读取
    if not api_key:
        key_file = os.path.join(os.path.dirname(__file__), "gemini_api_key.txt")
        if os.path.exists(key_file):
            with open(key_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and line != 'PASTE_YOUR_API_KEY_HERE':
                        api_key = line
                        break
    
    return api_key


def main():
    """主函数"""
    # 检查命令行参数
    if len(sys.argv) < 2:
        print("用法: python3 analyze_video.py <视频路径> [fps]")
        print()
        print("示例:")
        print("  python3 analyze_video.py /path/to/video.mp4")
        print("  python3 analyze_video.py /path/to/video.mp4 2.0  # 使用2.0 FPS（默认）")
        print()
        sys.exit(1)
    
    video_path = sys.argv[1]
    fps = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0  # Changed default from 1.0 to 2.0
    
    # 检查视频是否存在
    if not os.path.exists(video_path):
        print(f"❌ 错误: 视频文件不存在: {video_path}")
        sys.exit(1)
    
    # 获取API密钥
    api_key = get_api_key()
    if not api_key or api_key == 'PASTE_YOUR_API_KEY_HERE':
        print("❌ 错误: 请设置 API 密钥")
        print()
        print("方法1: 设置环境变量")
        print("  export GEMINI_API_KEY='your-api-key-here'")
        print()
        print("方法2: 编辑 gemini_api_key.txt 文件")
        print(f"  编辑文件: {os.path.join(os.path.dirname(__file__), 'gemini_api_key.txt')}")
        print()
        sys.exit(1)
    
    print("=" * 60)
    print("Gemini 视频分析器 - 伤员检测")
    print("=" * 60)
    print(f"视频: {video_path}")
    print(f"FPS: {fps}")
    print()
    
    try:
        # 创建分析器
        analyzer = GeminiVideoAnalyzer(api_key=api_key)
        
        # 分析视频
        analysis = analyzer.analyze_video(video_path=video_path, fps=fps)
        
        # 打印结果
        analyzer.print_analysis(analysis)
        
        # 保存结果
        output_path = video_path.replace('.mp4', '_gemini_analysis.json')
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump({
                'video_path': video_path,
                'is_casualty': analysis.is_casualty,
                'confidence': analysis.confidence,
                'trauma_locations': [loc.value for loc in analysis.trauma_locations],
                'hemorrhage_severity': analysis.hemorrhage_severity.value,
                'detailed_findings': analysis.detailed_findings,
                'timestamps': analysis.timestamps,
                'raw_response': analysis.raw_response
            }, f, ensure_ascii=False, indent=2)
        
        print(f"\n✅ 分析结果已保存到: {output_path}\n")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

