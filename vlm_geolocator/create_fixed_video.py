#!/usr/bin/env python3
"""
从多个视频段提取帧并创建统一帧率的新视频
"""

import cv2
import os
import sys
from pathlib import Path
import numpy as np

def extract_all_frames(video_paths):
    """从多个视频中提取所有有效帧"""
    all_frames = []
    
    for video_path in video_paths:
        print(f"读取: {Path(video_path).name}")
        cap = cv2.VideoCapture(video_path)
        
        if not cap.isOpened():
            print(f"  ⚠️  无法打开视频")
            continue
        
        frame_count = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            all_frames.append(frame)
            frame_count += 1
        
        cap.release()
        print(f"  提取了 {frame_count} 帧")
    
    return all_frames

def create_video_fixed_fps(frames, output_path, fps=5.0):
    """用固定帧率创建视频"""
    if not frames:
        print("❌ 没有可用的帧")
        return False
    
    # 获取帧大小
    height, width = frames[0].shape[:2]
    
    # 创建视频写入器（使用H.264编码）
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))
    
    if not writer.isOpened():
        print("❌ 无法创建视频写入器")
        return False
    
    # 写入所有帧
    for frame in frames:
        writer.write(frame)
    
    writer.release()
    
    duration = len(frames) / fps
    print(f"✅ 创建视频成功:")
    print(f"   文件: {output_path}")
    print(f"   帧数: {len(frames)}")
    print(f"   帧率: {fps} FPS")
    print(f"   时长: {duration:.2f} 秒")
    
    return True

def main():
    # 输入视频文件夹
    casualty_folder = Path("/home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator/inference_logs/recordings/casualty_40.425428_-79.954430")
    
    # 找到所有segment视频（按时间排序）
    segments = sorted(casualty_folder.glob("segment_*.mp4"))
    
    if not segments:
        print("❌ 没有找到segment视频")
        return 1
    
    print(f"找到 {len(segments)} 个视频段\n")
    
    # 提取所有帧
    print("=" * 60)
    all_frames = extract_all_frames(segments)
    print("=" * 60)
    print(f"\n总共提取了 {len(all_frames)} 帧\n")
    
    # 创建输出路径
    output_path = casualty_folder.parent / "casualty_40.425428_-79.954430_merged_20251117_094237.mp4"
    
    # 创建新视频（固定5 FPS）
    success = create_video_fixed_fps(all_frames, output_path, fps=5.0)
    
    if success:
        # 使用ffmpeg重新编码为H.264（更好的兼容性）
        print("\n正在用 H.264 重新编码...")
        temp_path = str(output_path).replace('.mp4', '_h264.mp4')
        
        import subprocess
        cmd = [
            'ffmpeg', '-i', str(output_path),
            '-c:v', 'libx264', '-preset', 'fast', '-crf', '23',
            '-movflags', '+faststart',
            '-y', temp_path
        ]
        
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        if result.returncode == 0:
            os.replace(temp_path, output_path)
            print(f"✅ H.264重编码完成")
        
        return 0
    else:
        return 1

if __name__ == '__main__':
    sys.exit(main())


