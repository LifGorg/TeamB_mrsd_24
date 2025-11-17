#!/usr/bin/env python3
"""
ROS2 Node: Listens to /generate_report topic and automatically merges video segments from casualty folders
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import os
import json
import subprocess
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional
import tempfile


class VideoMergerNode(Node):
    """Video Merger Node"""
    
    def __init__(self):
        super().__init__('video_merger_node')
        
        # Subscribe to /generate_report topic
        self.subscription = self.create_subscription(
            Int32,
            '/generate_report',
            self.generate_report_callback,
            10
        )
        
        # Configure paths
        self.recordings_base = Path(__file__).parent.parent / 'inference_logs' / 'recordings'
        self.get_logger().info(f'Video merger node started')
        self.get_logger().info(f'Recordings base directory: {self.recordings_base}')
        
        # Track if we're currently processing
        self.is_processing = False
        
    def generate_report_callback(self, msg: Int32):
        """
        Callback function: triggers merge whenever value is 1
        """
        if msg.data == 1 and not self.is_processing:
            # Trigger merge if value is 1 and not already processing
            self.get_logger().info('Received generate report signal, starting processing...')
            self.is_processing = True
            self.merge_all_casualties()
            self.is_processing = False
            self.get_logger().info('Processing complete, ready for next trigger')
    
    def find_casualty_folders(self) -> List[Path]:
        """
        Find all casualty folders (starting with 'casualty_')
        """
        if not self.recordings_base.exists():
            self.get_logger().warn(f'Recordings directory does not exist: {self.recordings_base}')
            return []
        
        casualty_folders = []
        for item in self.recordings_base.iterdir():
            if item.is_dir() and item.name.startswith('casualty_'):
                casualty_folders.append(item)
        
        self.get_logger().info(f'Found {len(casualty_folders)} casualty folder(s)')
        return casualty_folders
    
    def get_video_segments(self, casualty_folder: Path) -> List[Dict]:
        """
        Get all video segments in a casualty folder, sorted by timestamp
        Returns: [{'video': Path, 'metadata': dict, 'timestamp': datetime}, ...]
        """
        segments = []
        
        # Find all .mp4 files
        for video_file in casualty_folder.glob('segment_*.mp4'):
            metadata_file = video_file.with_suffix('.mp4').parent / f"{video_file.stem}_metadata.json"
            
            # Read metadata
            metadata = {}
            timestamp = None
            if metadata_file.exists():
                try:
                    with open(metadata_file, 'r') as f:
                        metadata = json.load(f)
                    
                    # Extract timestamp from filename or metadata
                    if 'start_time' in metadata:
                        timestamp = datetime.fromisoformat(metadata['start_time'])
                    else:
                        # Extract timestamp from filename (segment_20251115_211930.mp4)
                        timestamp_str = video_file.stem.replace('segment_', '')
                        timestamp = datetime.strptime(timestamp_str, '%Y%m%d_%H%M%S')
                except Exception as e:
                    self.get_logger().warn(f'Cannot read metadata {metadata_file}: {e}')
            
            if timestamp is None:
                # Fallback to file modification time
                timestamp = datetime.fromtimestamp(video_file.stat().st_mtime)
            
            segments.append({
                'video': video_file,
                'metadata': metadata,
                'timestamp': timestamp
            })
        
        # Sort by timestamp
        segments.sort(key=lambda x: x['timestamp'])
        
        self.get_logger().info(f'{casualty_folder.name}: Found {len(segments)} video segment(s)')
        return segments
    
    def check_merged_video_exists(self, casualty_name: str) -> Optional[Path]:
        """
        Check if a merged video already exists
        Returns: Path if exists, None otherwise
        """
        # Search pattern: casualty_33.7756_-84.3963_merged_YYYYMMDD_HHMMSS.mp4
        pattern = f"{casualty_name}_merged_*.mp4"
        existing = list(self.recordings_base.glob(pattern))
        
        if existing:
            # Return the most recent merged video
            existing.sort(key=lambda x: x.stat().st_mtime, reverse=True)
            self.get_logger().info(f'Merged video already exists: {existing[0].name}')
            return existing[0]
        
        return None
    
    def merge_videos_ffmpeg(self, segments: List[Dict], output_path: Path) -> bool:
        """
        Merge videos using ffmpeg
        """
        if not segments:
            self.get_logger().warn('No video segments to merge')
            return False
        
        # Create temporary file list
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            concat_file = f.name
            for seg in segments:
                # ffmpeg concat format: file 'path/to/file.mp4'
                f.write(f"file '{seg['video'].absolute()}'\n")
        
        try:
            # ffmpeg command
            cmd = [
                'ffmpeg',
                '-f', 'concat',
                '-safe', '0',
                '-i', concat_file,
                '-c', 'copy',  # Copy streams directly without re-encoding (fast)
                '-y',  # Overwrite output file
                str(output_path)
            ]
            
            self.get_logger().info(f'Executing merge command: {" ".join(cmd)}')
            
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=300  # 5 minute timeout
            )
            
            if result.returncode == 0:
                self.get_logger().info(f'✓ Successfully merged video: {output_path}')
                return True
            else:
                self.get_logger().error(f'✗ ffmpeg merge failed: {result.stderr.decode()}')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('ffmpeg merge timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Error during video merge: {e}')
            return False
        finally:
            # Clean up temporary file
            try:
                os.remove(concat_file)
            except:
                pass
    
    def merge_casualty_videos(self, casualty_folder: Path) -> bool:
        """
        Merge all videos from a single casualty folder
        """
        casualty_name = casualty_folder.name
        
        # Check if merged video already exists
        existing_merged = self.check_merged_video_exists(casualty_name)
        if existing_merged:
            self.get_logger().info(f'Skipping {casualty_name}, merged video already exists')
            return True
        
        # Get all video segments
        segments = self.get_video_segments(casualty_folder)
        
        if not segments:
            self.get_logger().warn(f'{casualty_name}: No video segments found')
            return False
        
        # Use earliest segment timestamp as merged video timestamp
        earliest_timestamp = segments[0]['timestamp']  # segments already sorted by time
        timestamp_str = earliest_timestamp.strftime('%Y%m%d_%H%M%S')
        output_filename = f"{casualty_name}_merged_{timestamp_str}.mp4"
        output_path = self.recordings_base / output_filename
        
        self.get_logger().info(f'Starting merge of {len(segments)} segment(s) for {casualty_name}...')
        
        # Execute merge
        success = self.merge_videos_ffmpeg(segments, output_path)
        
        if success:
            # Create metadata for merged video
            metadata = {
                'casualty_folder': casualty_name,
                'total_segments': len(segments),
                'first_detection_time': earliest_timestamp.isoformat(),  # First detection time
                'segments': [
                    {
                        'filename': seg['video'].name,
                        'timestamp': seg['timestamp'].isoformat(),
                        'gps': seg['metadata'].get('gps_at_start')
                    }
                    for seg in segments
                ],
                'merged_at': datetime.now().isoformat(),  # Actual merge time
                'output_file': output_filename
            }
            
            metadata_path = output_path.with_suffix('.json')
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            self.get_logger().info(f'✓ Merge complete: {output_filename}')
            return True
        else:
            return False
    
    def merge_all_casualties(self):
        """
        Merge videos from all casualty folders
        """
        casualty_folders = self.find_casualty_folders()
        
        if not casualty_folders:
            self.get_logger().info('No casualty folders found')
            return
        
        success_count = 0
        skip_count = 0
        fail_count = 0
        
        for folder in casualty_folders:
            try:
                # Check if merged video already exists
                if self.check_merged_video_exists(folder.name):
                    skip_count += 1
                    continue
                
                if self.merge_casualty_videos(folder):
                    success_count += 1
                else:
                    fail_count += 1
            except Exception as e:
                self.get_logger().error(f'Error processing {folder.name}: {e}')
                fail_count += 1
        
        self.get_logger().info(
            f'Video merge completed - Success: {success_count}, Skipped: {skip_count}, Failed: {fail_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VideoMergerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

