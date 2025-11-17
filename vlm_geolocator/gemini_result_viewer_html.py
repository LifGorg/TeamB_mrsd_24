#!/usr/bin/env python3
"""
Gemini Analysis Result Viewer (HTML version)
Generates HTML page to view in browser
Auto-analyzes videos without JSON results
"""

import os
import sys
import json
import glob
import base64
import cv2
import numpy as np
import webbrowser
import tempfile
import time

# Import Gemini analyzer
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
from vlm_geolocator.vision.gemini_video_analyzer import GeminiVideoAnalyzer


def extract_frames(video_path, num_frames=6):
    """从视频中提取帧（增强版 - 处理损坏的视频）"""
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print(f"❌ 无法打开视频: {video_path}")
        return []
    
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    if total_frames <= 0:
        print(f"⚠️  无法获取视频帧数，尝试顺序读取...")
        # Fallback: read frames sequentially
        frames = []
        frame_count = 0
        while len(frames) < num_frames and frame_count < 1000:  # Safety limit
            ret, frame = cap.read()
            if not ret:
                break
            # Sample every N frames
            if frame_count % max(1, frame_count // num_frames + 1) == 0:
                frames.append(frame)
            frame_count += 1
        cap.release()
        return frames
    
    if total_frames < num_frames:
        num_frames = total_frames
    
    # 均匀采样
    frame_indices = np.linspace(0, total_frames - 1, num_frames, dtype=int)
    
    frames = []
    failed_reads = 0
    
    for idx in frame_indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        
        if ret and frame is not None:
            frames.append(frame)
        else:
            failed_reads += 1
            # Try reading next frame if this one failed
            for retry in range(5):
                ret, frame = cap.read()
                if ret and frame is not None:
                    frames.append(frame)
                    break
    
    cap.release()
    
    if failed_reads > 0:
        print(f"⚠️  {failed_reads} 帧读取失败，成功提取 {len(frames)} 帧")
    
    return frames


def frame_to_base64(frame):
    """将帧转换为 base64 编码的 JPEG"""
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return base64.b64encode(buffer).decode('utf-8')


def calculate_triage_level(is_casualty, hemorrhage_severity, trauma_locations, findings, amputation_status, respiratory_status):
    """Calculate triage level based on casualty conditions"""
    if not is_casualty:
        return "GREEN", "No immediate medical attention required"
    
    # RED - Immediate life-saving intervention required
    # Check for critical respiratory issues
    if any(keyword in respiratory_status for keyword in ["No Breathing", "Gasping", "Critical"]):
        return "RED", "Immediate - Critical respiratory distress"
    
    # Check for critical hemorrhage
    if "Critical" in hemorrhage_severity or "Severe" in hemorrhage_severity:
        return "RED", "Immediate - Life-threatening hemorrhage"
    
    # Check for amputation (typically RED due to severe bleeding risk)
    if "⚠️" in amputation_status and amputation_status != "None detected":
        return "RED", "Immediate - Traumatic amputation"
    
    # YELLOW - Delayed medical attention required
    # Check for moderate hemorrhage or respiratory distress
    if "Moderate" in hemorrhage_severity:
        return "YELLOW", "Delayed - Moderate hemorrhage"
    
    if "Labored" in respiratory_status:
        return "YELLOW", "Delayed - Respiratory distress"
    
    # Check for multiple trauma locations
    if len(trauma_locations) >= 2:
        return "YELLOW", "Delayed - Multiple trauma sites"
    
    # GREEN - Minor injuries or ambulatory
    if "Minor" in hemorrhage_severity or len(trauma_locations) == 1:
        return "YELLOW", "Delayed - Minor to moderate injuries"
    
    # Default to GREEN if ambulatory/minor
    return "GREEN", "Minor - Walking wounded"


def generate_html(results):
    """Generate HTML page"""
    
    # Calculate current time for latency calculation
    html_generation_time = time.time()
    
    html = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Gemini Casualty Analysis Results Viewer</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #1e1e1e 0%, #2d2d2d 100%);
            color: #ffffff;
            padding: 20px;
        }
        
        .header {
            text-align: center;
            padding: 30px 0;
            background: linear-gradient(135deg, #0078d4 0%, #005a9e 100%);
            border-radius: 10px;
            margin-bottom: 30px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
        }
        
        .header h1 {
            font-size: 2.5em;
            margin-bottom: 10px;
        }
        
        .header p {
            font-size: 1.2em;
            opacity: 0.9;
        }
        
        .video-container {
            background: #2b2b2b;
            border-radius: 10px;
            padding: 30px;
            margin-bottom: 40px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
        }
        
        .video-title {
            font-size: 1.8em;
            margin-bottom: 20px;
            padding-bottom: 15px;
            border-bottom: 2px solid #0078d4;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        
        .video-number {
            background: #0078d4;
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 0.7em;
        }
        
        .content {
            display: grid;
            grid-template-columns: 1fr 2fr;
            gap: 30px;
        }
        
        .frames {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
        }
        
        .frame {
            background: #1e1e1e;
            border-radius: 8px;
            overflow: hidden;
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.3);
            max-height: 200px;
        }
        
        .frame img {
            width: 100%;
            height: 180px;
            object-fit: cover;
            display: block;
        }
        
        .frame-label {
            text-align: center;
            padding: 10px;
            font-size: 0.9em;
            color: #888;
        }
        
        .analysis {
            background: #1e1e1e;
            border-radius: 8px;
            padding: 25px;
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.3);
        }
        
        .analysis h3 {
            font-size: 1.4em;
            margin-bottom: 15px;
            color: #0078d4;
        }
        
        .status {
            padding: 10px 15px;
            border-radius: 8px;
            margin-bottom: 15px;
            font-weight: bold;
            font-size: 1.1em;
            border-left: 4px solid;
        }
        
        .status.casualty {
            background: #3d1a1a;
            border-color: #ff4444;
            color: #ff4444;
        }
        
        .status.safe {
            background: #1a3d1a;
            border-color: #44ff44;
            color: #44ff44;
        }
        
        .detail-section {
            margin-bottom: 20px;
        }
        
        .detail-section h4 {
            color: #0078d4;
            margin-bottom: 10px;
            font-size: 1.1em;
        }
        
        .detail-section p, .detail-section ul {
            line-height: 1.6;
            color: #cccccc;
        }
        
        .detail-section ul {
            list-style: none;
            padding-left: 20px;
        }
        
        .detail-section li:before {
            content: "• ";
            color: #0078d4;
            font-weight: bold;
        }
        
        .module-card {
            background: linear-gradient(135deg, #2a2a2a 0%, #1e1e1e 100%);
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
            border-left: 4px solid #0078d4;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
        }
        
        .module-card h4 {
            color: #0078d4;
            margin-bottom: 15px;
            font-size: 1.2em;
            padding-bottom: 10px;
            border-bottom: 2px solid #0078d4;
        }
        
        .module-card p {
            line-height: 1.8;
            color: #e0e0e0;
            margin: 8px 0;
        }
        
        .module-card.trauma {
            border-left-color: #ff6b6b;
        }
        
        .module-card.trauma h4 {
            color: #ff6b6b;
            border-bottom-color: #ff6b6b;
        }
        
        .module-card.hemorrhage {
            border-left-color: #ff4444;
        }
        
        .module-card.hemorrhage h4 {
            color: #ff4444;
            border-bottom-color: #ff4444;
        }
        
        .module-card.movement {
            border-left-color: #44aaff;
        }
        
        .module-card.movement h4 {
            color: #44aaff;
            border-bottom-color: #44aaff;
        }
        
        .json-container {
            background: #1a1a1a;
            padding: 15px;
            border-radius: 8px;
            margin-top: 20px;
            max-height: 400px;
            overflow-y: auto;
        }
        
        .json-content {
            font-family: 'Courier New', monospace;
            font-size: 11px;
            color: #00ff00;
            white-space: pre-wrap;
            word-wrap: break-word;
        }
        
        .confidence {
            font-size: 1.3em;
            color: #ffaa00;
            margin: 15px 0;
        }
        
        .latency-info {
            background: #1a1a1a;
            padding: 15px;
            border-radius: 8px;
            margin: 15px 0;
            border-left: 4px solid #ffaa00;
        }
        
        .latency-info p {
            margin: 5px 0;
            color: #ffffff;
        }
        
        .latency-value {
            font-size: 1.4em;
            font-weight: bold;
            color: #ffaa00;
        }
        
        /* New Compact Medical Info Styles */
        .medical-summary {
            background: #1a1a1a;
            border-radius: 8px;
            padding: 20px;
            margin-bottom: 20px;
        }
        
        .triage-banner {
            padding: 15px 20px;
            border-radius: 8px;
            margin-bottom: 20px;
            font-weight: bold;
            font-size: 1.3em;
            text-align: center;
            border: 3px solid;
        }
        
        .triage-banner.RED {
            background: #8b0000;
            border-color: #ff0000;
            color: #ffffff;
        }
        
        .triage-banner.YELLOW {
            background: #8b8b00;
            border-color: #ffff00;
            color: #ffffff;
        }
        
        .triage-banner.GREEN {
            background: #006400;
            border-color: #00ff00;
            color: #ffffff;
        }
        
        .triage-banner.BLACK {
            background: #000000;
            border-color: #666666;
            color: #ffffff;
        }
        
        .info-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin-top: 15px;
        }
        
        .info-card {
            background: #2a2a2a;
            border-radius: 6px;
            padding: 12px 15px;
            border-left: 4px solid;
            min-width: 0;
            overflow: hidden;
        }
        
        .info-card.gps {
            border-color: #4444ff;
            grid-column: span 2;  /* Make GPS card span 2 columns for more space */
        }
        
        .info-card.casualty-status {
            border-color: #ff4444;
        }
        
        .info-card.trauma {
            border-color: #ff6b6b;
        }
        
        .info-card.hemorrhage {
            border-color: #ff0000;
        }
        
        .info-card.amputation {
            border-color: #ff8800;
        }
        
        .info-card.respiratory {
            border-color: #00aaff;
        }
        
        .info-card-title {
            font-size: 0.85em;
            color: #888;
            margin-bottom: 5px;
            text-transform: uppercase;
            font-weight: 600;
        }
        
        .info-card-value {
            font-size: 1.1em;
            color: #ffffff;
            font-weight: bold;
            word-wrap: break-word;
            overflow-wrap: break-word;
        }
        
        .info-card-detail {
            font-size: 0.9em;
            color: #cccccc;
            margin-top: 5px;
            line-height: 1.4;
        }
        
        @media (max-width: 1200px) {
            .content {
                grid-template-columns: 1fr;
            }
            .info-grid {
                grid-template-columns: 1fr;
            }
        }
        
        @media (max-width: 768px) {
            .frames {
                grid-template-columns: repeat(2, 1fr);
            }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🔬 Gemini Casualty Analysis Results Viewer</h1>
        <p>Total """ + str(len(results)) + """ Video Analysis Results</p>
    </div>
"""
    
    for idx, result in enumerate(results):
        try:
            analysis_data = result['analysis']
            video_name = analysis_data.get('video_name', os.path.basename(result['video_path']))
            
            # 处理两种JSON结构：直接或嵌套在analysis下
            if 'analysis' in analysis_data:
                # 新格式：有嵌套的 analysis 字段
                is_casualty = analysis_data['analysis']['is_casualty']
                confidence = analysis_data['analysis']['confidence']
                trauma_locations = analysis_data['analysis']['trauma_locations']
                hemorrhage_severity = analysis_data['analysis']['hemorrhage_severity']
            else:
                # 旧格式：直接在顶层
                is_casualty = analysis_data.get('is_casualty', False)
                confidence = analysis_data.get('confidence', 0.0)
                trauma_locations = analysis_data.get('trauma_locations', [])
                hemorrhage_severity = analysis_data.get('hemorrhage_severity', '未知')
            
            overall_assessment = analysis_data.get('overall_assessment', '')
            recommendations = analysis_data.get('recommendations', '')
        except Exception as e:
            print(f"⚠️  跳过视频 {idx + 1}: {e}")
            continue
        
        # 提取视频帧
        print(f"处理视频 {idx + 1}/{len(results)}: {video_name}")
        frames = extract_frames(result['video_path'])
        
        # Get detailed findings
        findings = None
        if 'analysis' in analysis_data and 'detailed_findings' in analysis_data['analysis']:
            findings = analysis_data['analysis']['detailed_findings']
        elif 'detailed_findings' in analysis_data:
            findings = analysis_data['detailed_findings']
        
        # Extract GPS from metadata (preferred) or video name fallback
        gps_coords = "Unknown"
        
        # Try to get GPS from analysis metadata first
        if 'gps' in analysis_data and isinstance(analysis_data['gps'], dict):
            lat = analysis_data['gps'].get('latitude', None)
            lon = analysis_data['gps'].get('longitude', None)
            if lat is not None and lon is not None:
                gps_coords = f"{lat:.6f}, {lon:.6f}"
        
        # Fallback: extract from video name (casualty_LAT_LON format)
        # Format: casualty_33.7756_-84.3963_merged_... or casualty_33.7756_-84.3963.mp4
        if gps_coords == "Unknown" and "casualty_" in video_name:
            try:
                # Split by underscore
                parts = video_name.split("_")
                if len(parts) >= 3:
                    lat = parts[1]  # 33.7756
                    lon = parts[2]  # -84.3963 (full value, not split further)
                    gps_coords = f"{lat}, {lon}"
            except:
                pass
        
        # Extract amputation status from trauma assessment
        amputation_status = "None detected"
        if findings and 'trauma' in findings:
            # Check if trauma is a dict with 'amputation' field (new format)
            if isinstance(findings['trauma'], dict) and 'amputation' in findings['trauma']:
                amp_desc = findings['trauma']['amputation'].lower()
                if amp_desc != 'none' and amp_desc != '':
                    amputation_status = f"⚠️ {findings['trauma']['amputation']}"
            else:
                # Fallback: check description for keywords
                trauma_desc = str(findings['trauma']).lower()
                if 'amput' in trauma_desc or 'missing limb' in trauma_desc or 'severed' in trauma_desc:
                    amputation_status = "⚠️ Possible amputation detected"
        
        # Extract respiratory status
        respiratory_status = "Normal Breathing"  # Default to normal instead of unable to assess
        respiratory_detail = ""
        
        # Check for respiratory_status field in casualty data (new format)
        if 'respiratory_status' in analysis_data:
            resp_data = analysis_data['respiratory_status']
            if isinstance(resp_data, dict):
                respiratory_status = resp_data.get('status', 'Normal Breathing')
                respiratory_detail = resp_data.get('observations', '')
        elif findings and 'respiratory_status' in findings:
            resp_data = findings['respiratory_status']
            if isinstance(resp_data, dict):
                respiratory_status = resp_data.get('status', 'Normal Breathing')
                respiratory_detail = resp_data.get('observations', '')
        
        # Fallback: check observations for respiratory keywords (for old JSON format)
        if respiratory_status == "Normal Breathing" and findings and 'observations' in findings:
            obs = findings['observations'].lower()
            if 'no breathing' in obs or 'not breathing' in obs or 'no chest movement' in obs:
                respiratory_status = "⚠️ No Breathing"
            elif 'gasping' in obs or 'gasp' in obs:
                respiratory_status = "⚠️ Gasping"
            elif 'labored breathing' in obs or 'difficulty breathing' in obs or 'respiratory distress' in obs or 'chest heaving' in obs:
                respiratory_status = "⚠️ Labored Breathing"
            elif 'abnormal breathing' in obs or 'irregular breathing' in obs:
                respiratory_status = "⚠️ Check Airway"
        
        # If no casualty detected, show as N/A instead of Normal
        if not is_casualty:
            respiratory_status = "N/A (No casualty)"
        
        # Calculate triage level (now with amputation and respiratory status)
        triage_level, triage_description = calculate_triage_level(
            is_casualty, hemorrhage_severity, trauma_locations, findings, 
            amputation_status, respiratory_status
        )
        
        html += f"""
    <div class="video-container" id="video-{idx}">
        <div class="video-title">
            <span>📹 {video_name}</span>
            <span class="video-number">#{idx + 1}</span>
        </div>
        
        <div class="content">
            <div class="frames">
"""
        
        # 添加视频帧
        for frame_idx, frame in enumerate(frames[:6]):
            img_data = frame_to_base64(frame)
            html += f"""
                <div class="frame">
                    <img src="data:image/jpeg;base64,{img_data}" alt="Frame {frame_idx + 1}">
                    <div class="frame-label">Frame {frame_idx + 1}</div>
                </div>
"""
        
        html += f"""
            </div>
            
            <div class="analysis">
                <h3>🏥 Medical Triage Report</h3>
                
                <!-- Triage Level Banner -->
                <div class="triage-banner {triage_level}">
                    TRIAGE LEVEL: {triage_level}
                </div>
                
                <!-- Compact Medical Info Grid -->
                <div class="medical-summary">
                    <div class="info-grid">
                        <!-- Casualty Status -->
                        <div class="info-card casualty-status">
                            <div class="info-card-title">Casualty Status</div>
                            <div class="info-card-value">{'YES' if is_casualty else 'NO'}</div>
                            <div class="info-card-detail">Confidence: {confidence:.0%}</div>
                        </div>
                        
                        <!-- GPS Location -->
                        <div class="info-card gps">
                            <div class="info-card-title">GPS Coordinates</div>
                            <div class="info-card-value">{gps_coords}</div>
                        </div>
                        
                        <!-- Trauma Condition -->
                        <div class="info-card trauma">
                            <div class="info-card-title">Trauma Assessment</div>
                            <div class="info-card-value">
                                {
                                    'No trauma' if not trauma_locations or len(trauma_locations) == 0 or trauma_locations[0] == 'None' 
                                    else f'{len(trauma_locations)} location(s)'
                                }
                            </div>
                            <div class="info-card-detail">{'None' if not trauma_locations or trauma_locations[0] == 'None' else ', '.join(trauma_locations)}</div>
                        </div>
                        
                        <!-- Hemorrhage -->
                        <div class="info-card hemorrhage">
                            <div class="info-card-title">Hemorrhage Severity</div>
                            <div class="info-card-value">{hemorrhage_severity}</div>
                        </div>
                        
                        <!-- Amputation -->
                        <div class="info-card amputation">
                            <div class="info-card-title">Amputation Status</div>
                            <div class="info-card-value">{amputation_status}</div>
                        </div>
                        
                        <!-- Respiratory Distress -->
                        <div class="info-card respiratory">
                            <div class="info-card-title">Respiratory Status</div>
                            <div class="info-card-value">{respiratory_status}</div>
                        </div>
                    </div>
                </div>
"""
        
        # Add compact detailed observations
        if findings:
            if 'observations' in findings and findings['observations']:
                html += f"""
                <div class="detail-section">
                    <h4>📋 Detailed Observations</h4>
                    <p>{findings['observations']}</p>
                </div>
"""
            
            if 'overall' in findings and findings['overall']:
                html += f"""
                <div class="detail-section">
                    <h4>📊 Overall Assessment</h4>
                    <p>{findings['overall']}</p>
                </div>
"""
            
            if 'recommendations' in findings and findings['recommendations']:
                html += f"""
                <div class="detail-section">
                    <h4>💡 Recommended Actions</h4>
                    <p>{findings['recommendations']}</p>
                </div>
"""
        
        # Close the analysis and content divs
        html += """
            </div>
        </div>
    </div>
"""
    
    html += """
</body>
</html>
"""
    
    return html


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Gemini Analysis Results Viewer')
    parser.add_argument('--watch', action='store_true', help='Continuously monitor for new videos')
    parser.add_argument('--interval', type=int, default=30, help='Check interval in seconds (default: 30)')
    args = parser.parse_args()
    
    recording_dir = '/home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator/inference_logs/recordings'
    
    if not os.path.exists(recording_dir):
        print(f"❌ Directory does not exist: {recording_dir}")
        sys.exit(1)
    
    if args.watch:
        print("👁️  Watch mode enabled - monitoring for new videos...")
        print(f"📂 Directory: {recording_dir}")
        print(f"⏱️  Check interval: {args.interval} seconds")
        print("Press Ctrl+C to stop\n")
        
        last_modified_times = {}
        iteration = 0
        
        try:
            while True:
                iteration += 1
                print(f"\n{'='*60}")
                print(f"🔄 Check #{iteration} - {time.strftime('%Y-%m-%d %H:%M:%S')}")
                print('='*60)
                
                # Find all video files (including merged casualty videos)
                video_files = []
                video_files.extend(glob.glob(os.path.join(recording_dir, 'recording_*.mp4')))
                video_files.extend(glob.glob(os.path.join(recording_dir, 'casualty_*_merged_*.mp4')))
                
                # Check for new or modified videos
                new_or_modified = False
                for video_path in video_files:
                    mtime = os.path.getmtime(video_path)
                    if video_path not in last_modified_times or last_modified_times[video_path] != mtime:
                        new_or_modified = True
                        last_modified_times[video_path] = mtime
                
                if new_or_modified:
                    print(f"🎬 Detected changes! Regenerating HTML...")
                    process_and_generate_html(recording_dir)
                else:
                    print("✅ No changes - HTML is up to date")
                
                print(f"\n💤 Waiting {args.interval} seconds before next check...")
                time.sleep(args.interval)
                
        except KeyboardInterrupt:
            print("\n\n⏹️  Watch mode stopped by user")
            print("✅ Exiting gracefully...")
            sys.exit(0)
    else:
        # Single run mode (original behavior)
        process_and_generate_html(recording_dir)


def process_and_generate_html(recording_dir):
    """Process videos and generate HTML"""
    recording_dir = '/home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator/inference_logs/recordings'
    
    if not os.path.exists(recording_dir):
        print(f"❌ 目录不存在: {recording_dir}")
        sys.exit(1)
    
    # 加载所有分析结果
    print("🔍 正在扫描分析结果...")
    analysis_files = glob.glob(os.path.join(recording_dir, '*_gemini_analysis.json'))
    
    results = []
    for json_path in analysis_files:
        video_path = json_path.replace('_gemini_analysis.json', '.mp4')
        if os.path.exists(video_path):
            with open(json_path, 'r', encoding='utf-8') as f:
                analysis = json.load(f)
            
            # 获取文件修改时间
            mtime = os.path.getmtime(json_path)
            
            results.append({
                'video_path': video_path,
                'json_path': json_path,
                'analysis': analysis,
                'mtime': mtime
            })
    
    if not results:
        print("⚠️  No analysis results found")
        print("🔬 Will analyze latest 4 videos...")
        
        # Find all video files (including merged casualty videos)
        video_files = []
        video_files.extend(glob.glob(os.path.join(recording_dir, 'recording_*.mp4')))
        video_files.extend(glob.glob(os.path.join(recording_dir, 'casualty_*_merged_*.mp4')))
        
        if not video_files:
            print("❌ No video files found")
            sys.exit(1)
        
        # Sort by modification time and take latest 4
        video_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)
        latest_videos = video_files[:4]
        
        print(f"📹 Found {len(video_files)} videos, will analyze latest {len(latest_videos)}")
        
        # Initialize analyzer
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            key_file = os.path.join(os.path.dirname(__file__), "gemini_api_key.txt")
            if os.path.exists(key_file):
                with open(key_file, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line and not line.startswith('#') and line != 'PASTE_YOUR_API_KEY_HERE':
                            api_key = line
                            break
        
        if not api_key:
            print("❌ No GEMINI_API_KEY found")
            sys.exit(1)
        
        analyzer = GeminiVideoAnalyzer(api_key=api_key)
        
        # Analyze each video
        for idx, video_path in enumerate(latest_videos, 1):
            print(f"\n{'='*60}")
            print(f"Analyzing video {idx}/{len(latest_videos)}: {os.path.basename(video_path)}")
            print('='*60)
            
            json_path = video_path.replace('.mp4', '_gemini_analysis.json')
            
            try:
                # Analyze video
                analysis = analyzer.analyze_video(video_path=video_path, fps=2.0)
                
                # Build result message
                result_msg = {
                    'video_path': video_path,
                    'video_name': os.path.basename(video_path),
                    'timestamp': time.time(),
                    'analysis': {
                        'is_casualty': analysis.is_casualty,
                        'confidence': analysis.confidence,
                        'trauma_locations': [loc.value for loc in analysis.trauma_locations],
                        'hemorrhage_severity': analysis.hemorrhage_severity.value,
                        'timestamps': analysis.timestamps,
                        'detailed_findings': analysis.detailed_findings
                    },
                    'recommendations': analysis.detailed_findings.get('recommendations', ''),
                    'overall_assessment': analysis.detailed_findings.get('overall', '')
                }
                
                # Save to JSON
                with open(json_path, 'w', encoding='utf-8') as f:
                    json.dump(result_msg, f, ensure_ascii=False, indent=2)
                
                print(f"✅ Analysis complete and saved: {json_path}")
                
                # Add to results
                results.append({
                    'video_path': video_path,
                    'json_path': json_path,
                    'analysis': result_msg,
                    'mtime': os.path.getmtime(json_path)
                })
                
            except Exception as e:
                print(f"❌ Analysis failed: {e}")
                import traceback
                traceback.print_exc()
        
        if not results:
            print("❌ No successful analysis")
            sys.exit(1)
    
    # 按修改时间排序，取最新的4个
    results.sort(key=lambda x: x['mtime'], reverse=True)
    total_found = len(results)
    results = results[:4]
    
    print(f"✅ Found {total_found} analysis results, showing latest {len(results)}")
    
    # Check for missing analysis and run if needed
    analyzer = None
    # Find all video files (including merged casualty videos)
    video_files = []
    video_files.extend(glob.glob(os.path.join(recording_dir, 'recording_*.mp4')))
    video_files.extend(glob.glob(os.path.join(recording_dir, 'casualty_*_merged_*.mp4')))
    video_files.sort(key=lambda x: os.path.getmtime(x), reverse=True)
    latest_videos = video_files[:4]
    
    for video_path in latest_videos:
        json_path = video_path.replace('.mp4', '_gemini_analysis.json')
        if not os.path.exists(json_path):
            print(f"\n⚠️  No analysis found for: {os.path.basename(video_path)}")
            print(f"🔬 Running Gemini analysis...")
            
            # Initialize analyzer if needed
            if analyzer is None:
                api_key = os.environ.get("GEMINI_API_KEY")
                if not api_key:
                    key_file = os.path.join(os.path.dirname(__file__), "gemini_api_key.txt")
                    if os.path.exists(key_file):
                        with open(key_file, 'r') as f:
                            for line in f:
                                line = line.strip()
                                if line and not line.startswith('#') and line != 'PASTE_YOUR_API_KEY_HERE':
                                    api_key = line
                                    break
                
                if not api_key:
                    print("❌ No GEMINI_API_KEY found, skipping auto-analysis")
                    continue
                
                analyzer = GeminiVideoAnalyzer(api_key=api_key)
            
            try:
                # Analyze video
                analysis = analyzer.analyze_video(video_path=video_path, fps=2.0)
                
                # Build result message
                result_msg = {
                    'video_path': video_path,
                    'video_name': os.path.basename(video_path),
                    'timestamp': time.time(),
                    'analysis': {
                        'is_casualty': analysis.is_casualty,
                        'confidence': analysis.confidence,
                        'trauma_locations': [loc.value for loc in analysis.trauma_locations],
                        'hemorrhage_severity': analysis.hemorrhage_severity.value,
                        'timestamps': analysis.timestamps,
                        'detailed_findings': analysis.detailed_findings
                    },
                    'recommendations': analysis.detailed_findings.get('recommendations', ''),
                    'overall_assessment': analysis.detailed_findings.get('overall', '')
                }
                
                # Save to JSON
                with open(json_path, 'w', encoding='utf-8') as f:
                    json.dump(result_msg, f, ensure_ascii=False, indent=2)
                
                print(f"✅ Analysis complete and saved: {json_path}")
                
                # Add to results
                results.append({
                    'video_path': video_path,
                    'json_path': json_path,
                    'analysis': result_msg,
                    'mtime': os.path.getmtime(json_path)
                })
                
            except Exception as e:
                print(f"❌ Analysis failed: {e}")
    
    # Re-sort after adding new analyses
    results.sort(key=lambda x: x['mtime'], reverse=True)
    results = results[:4]
    print("📝 正在生成 HTML...")
    
    # 生成 HTML
    html_content = generate_html(results)
    
    # 保存到临时文件
    output_file = os.path.join(recording_dir, 'gemini_analysis_results.html')
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(html_content)
    
    print(f"✅ HTML 已生成: {output_file}")
    print("🌐 正在浏览器中打开...")
    
    # 在浏览器中打开
    webbrowser.open('file://' + output_file)
    
    print("✅ 完成！")
    print(f"\n如需重新查看，请打开: {output_file}")


if __name__ == '__main__':
    main()

