#!/usr/bin/env python3
"""
测试 Gemini 视频分析器
用于分析示例无人机视频
"""

import os
import sys

# 添加src到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from vlm_geolocator.vision.gemini_video_analyzer import (
    GeminiVideoAnalyzer,
    TraumaLocation,
    HemorrhageSeverity
)


def test_drone_video():
    """测试无人机视频分析"""
    
    # 示例视频路径
    video_path = "/home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator/inference_logs/recordings/recording_20251113_164742.mp4"
    
    # 检查视频是否存在
    if not os.path.exists(video_path):
        print(f"错误: 视频文件不存在: {video_path}")
        return
    
    # 尝试从多个来源获取API密钥
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
    
    if not api_key or api_key == 'PASTE_YOUR_API_KEY_HERE':
        print("错误: 请设置 API 密钥")
        print()
        print("方法1: 设置环境变量")
        print("  export GEMINI_API_KEY='your-api-key-here'")
        print()
        print("方法2: 编辑 gemini_api_key.txt 文件")
        print(f"  编辑文件: {os.path.join(os.path.dirname(__file__), 'gemini_api_key.txt')}")
        print("  将您的API密钥粘贴进去替换 PASTE_YOUR_API_KEY_HERE")
        print()
        print("获取API密钥: https://aistudio.google.com/app/apikey")
        return
    
    print("=" * 60)
    print("Gemini 视频分析器 - 伤员检测测试")
    print("=" * 60)
    print(f"视频路径: {video_path}")
    print(f"API密钥: {'已设置 (' + api_key[:10] + '...)' if api_key else '未设置'}")
    print()
    
    try:
        # 创建分析器
        print("初始化 Gemini 分析器...")
        analyzer = GeminiVideoAnalyzer(api_key=api_key)
        
        # 分析视频
        print("开始分析无人机视频...")
        print("(这可能需要几分钟时间，请耐心等待...)\n")
        
        analysis = analyzer.analyze_video(
            video_path=video_path,
            fps=1.0  # 每秒采样1帧 (对于抖动视频已经足够)
        )
        
        # 打印格式化结果
        analyzer.print_analysis(analysis)
        
        # 额外的决策支持信息
        print("\n" + "=" * 60)
        print("紧急响应建议")
        print("=" * 60)
        
        if analysis.is_casualty:
            print("\n⚠️  检测到伤员！")
            print(f"   置信度: {analysis.confidence:.0%}")
            
            # 创伤评估
            if TraumaLocation.HEAD in analysis.trauma_locations:
                print("\n🔴 头部创伤 - 优先级：极高")
                print("   建议: 立即派遣医疗团队，准备头部创伤处理设备")
            
            if TraumaLocation.TRUNK in analysis.trauma_locations:
                print("\n🔴 躯干创伤 - 优先级：高")
                print("   建议: 准备胸腹部创伤处理设备")
            
            if TraumaLocation.LIMBS in analysis.trauma_locations:
                print("\n🟡 四肢创伤 - 优先级：中")
                print("   建议: 准备夹板和止血带")
            
            # 出血评估
            if analysis.hemorrhage_severity == HemorrhageSeverity.CRITICAL:
                print("\n🚨 危急出血！")
                print("   建议: 立即派遣快速反应医疗队，准备大量止血物资和输血设备")
            elif analysis.hemorrhage_severity == HemorrhageSeverity.SEVERE:
                print("\n🔴 严重出血")
                print("   建议: 快速派遣医疗团队，准备止血物资")
            elif analysis.hemorrhage_severity == HemorrhageSeverity.MODERATE:
                print("\n🟡 中度出血")
                print("   建议: 派遣医疗团队进行止血处理")
            elif analysis.hemorrhage_severity == HemorrhageSeverity.MINOR:
                print("\n🟢 轻微出血")
                print("   建议: 准备基本急救包")
            
            # 时间戳信息
            if analysis.timestamps:
                print("\n关键时刻:")
                for ts in analysis.timestamps:
                    print(f"   ⏱️  {ts}")
        
        else:
            print("\n✅ 未检测到伤员")
            print("   当前区域可能安全，继续监控其他区域")
        
        print("\n" + "=" * 60 + "\n")
        
        # 保存分析结果
        import json
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
        
        print(f"✅ 分析结果已保存到: {output_path}\n")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    test_drone_video()

