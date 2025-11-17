"""视频录制模块"""
import threading
import time
import queue
import json
import hashlib
import datetime
from pathlib import Path
from typing import Optional
import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("[VideoRecorder] Warning: OpenCV not available, video recording disabled")

try:
    from sensor_msgs.msg import NavSatFix
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[VideoRecorder] Warning: ROS2 sensor_msgs not available, GPS tracking disabled")


class VideoRecorder:
    """视频录制器 - 录制固定时长的视频片段，支持按伤员GPS分文件夹"""
    
    def __init__(self, output_dir: str, fps: int = 30, ros_node=None):
        """
        初始化录制器
        
        Args:
            output_dir: 视频输出目录（基础目录）
            fps: 帧率 (默认30fps)
            ros_node: ROS2节点实例（用于订阅GPS话题）
        """
        if not CV2_AVAILABLE:
            raise RuntimeError("OpenCV is required for video recording")
        
        self.base_output_dir = Path(output_dir)
        self.base_output_dir.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.ros_node = ros_node
        
        # 录制状态
        self.is_recording = False
        self.recording_lock = threading.Lock()
        
        # 帧缓冲队列
        self.frame_queue = queue.Queue(maxsize=300)  # 最多缓冲10秒(30fps)
        
        # 录制线程
        self.recording_thread = None
        self.stop_event = threading.Event()
        
        # GPS追踪
        self.current_casualty_gps = None
        self.gps_timestamp = None
        self.gps_lock = threading.Lock()
        self.casualty_gps_subscription = None
        
        # 当前录制信息
        self.current_output_dir = None
        self.recording_metadata = {}
        
        # 订阅伤员GPS话题
        if ros_node is not None and ROS_AVAILABLE:
            try:
                self.casualty_gps_subscription = ros_node.create_subscription(
                    NavSatFix,
                    '/casualty_geolocated',
                    self._casualty_gps_callback,
                    10
                )
                ros_node.get_logger().info('[VideoRecorder] Subscribed to /casualty_geolocated')
            except Exception as e:
                if ros_node:
                    ros_node.get_logger().warn(f'[VideoRecorder] Failed to subscribe to GPS: {e}')
        else:
            if ros_node:
                ros_node.get_logger().warn('[VideoRecorder] GPS tracking disabled (ROS messages not available)')
        
    def add_frame(self, frame: np.ndarray):
        """
        添加帧到录制缓冲
        
        Args:
            frame: 视频帧 (numpy array)
        """
        if self.is_recording:
            try:
                # 非阻塞方式添加,如果队列满则丢弃最老的帧
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                self.frame_queue.put_nowait((frame.copy(), time.time()))
            except queue.Full:
                pass  # 队列满则跳过此帧
    
    def _casualty_gps_callback(self, msg: 'NavSatFix'):
        """
        接收新的伤员GPS位置
        
        Args:
            msg: NavSatFix消息
        """
        with self.gps_lock:
            self.current_casualty_gps = msg
            self.gps_timestamp = time.time()
        
        if self.ros_node:
            self.ros_node.get_logger().info(
                f'[VideoRecorder] Updated casualty GPS: '
                f'lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
            )
    
    def _copy_gps(self, gps: Optional['NavSatFix']) -> Optional['NavSatFix']:
        """
        深拷贝GPS消息（线程安全）
        
        Args:
            gps: NavSatFix消息
            
        Returns:
            GPS消息的副本
        """
        if gps is None or not ROS_AVAILABLE:
            return None
        
        try:
            new_gps = NavSatFix()
            new_gps.header = gps.header
            new_gps.status = gps.status
            new_gps.latitude = gps.latitude
            new_gps.longitude = gps.longitude
            new_gps.altitude = gps.altitude
            new_gps.position_covariance = gps.position_covariance
            new_gps.position_covariance_type = gps.position_covariance_type
            return new_gps
        except Exception:
            return None
    
    def _get_gps_age(self) -> float:
        """
        获取GPS数据的年龄（秒）
        
        Returns:
            GPS数据年龄（秒），如果无GPS则返回无穷大
        """
        if self.gps_timestamp is None:
            return float('inf')
        
        return time.time() - self.gps_timestamp
    
    def _get_casualty_folder_name(self, gps: Optional['NavSatFix']) -> str:
        """
        根据GPS坐标生成文件夹名
        四舍五入到6位小数（约0.11米精度），相近位置会得到相同文件夹
        
        Args:
            gps: NavSatFix消息
            
        Returns:
            文件夹名称
        """
        if gps is None:
            return "no_gps"
        
        # 四舍五入GPS坐标到6位小数（约0.11米精度）
        lat = round(gps.latitude, 6)
        lon = round(gps.longitude, 6)
        
        # 格式化文件夹名: casualty_纬度_经度
        folder_name = f"casualty_{lat:.6f}_{lon:.6f}"
        return folder_name
    
    def start_recording(self, duration: float = 5.0, min_fps: float = 2.0) -> Optional[str]:
        """
        开始录制指定时长的视频（自动根据伤员GPS分配文件夹）
        
        Args:
            duration: 录制时长(秒)
            min_fps: 最小要求帧率 (默认2.0fps)
            
        Returns:
            视频文件路径,如果已在录制则返回None
        """
        with self.recording_lock:
            if self.is_recording:
                return None
            
            self.is_recording = True
            
            # 获取当前伤员GPS快照
            with self.gps_lock:
                gps_snapshot = self._copy_gps(self.current_casualty_gps)
                gps_age = self._get_gps_age()
            
            # 确定保存目录（按GPS分文件夹）
            folder_name = self._get_casualty_folder_name(gps_snapshot)
            self.current_output_dir = self.base_output_dir / folder_name
            self.current_output_dir.mkdir(parents=True, exist_ok=True)
            
            # 生成输出文件名
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"segment_{timestamp}.mp4"
            output_path = self.current_output_dir / filename
            
            # 记录元数据
            self.recording_metadata = {
                "start_time": timestamp,
                "video_file": filename,
                "folder": folder_name,
                "has_gps": gps_snapshot is not None
            }
            
            if gps_snapshot is not None:
                self.recording_metadata["gps"] = {
                    "latitude": round(gps_snapshot.latitude, 6),
                    "longitude": round(gps_snapshot.longitude, 6),
                    "altitude": round(gps_snapshot.altitude, 2),
                    "status": gps_snapshot.status.status if hasattr(gps_snapshot.status, 'status') else 0,
                }
                self.recording_metadata["gps_age_seconds"] = gps_age
                
                if self.ros_node:
                    self.ros_node.get_logger().info(
                        f'[VideoRecorder] Recording casualty at GPS: '
                        f'lat={gps_snapshot.latitude:.6f}, lon={gps_snapshot.longitude:.6f}\n'
                        f'[VideoRecorder] Saving to folder: {folder_name}'
                    )
                    if gps_age > 30.0:
                        self.ros_node.get_logger().warn(
                            f'[VideoRecorder] ⚠️  GPS data is stale ({gps_age:.1f}s old)'
                        )
            else:
                if self.ros_node:
                    self.ros_node.get_logger().warn(
                        f'[VideoRecorder] ⚠️  No GPS available! Saving to: {folder_name}'
                    )
            
            # 清空队列
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    break
            
            # 启动录制线程
            self.stop_event.clear()
            self.recording_thread = threading.Thread(
                target=self._recording_worker,
                args=(output_path, duration, min_fps),
                daemon=True
            )
            self.recording_thread.start()
            
            return str(output_path)
    
    def _recording_worker(self, output_path: Path, duration: float, min_fps: float):
        """
        录制工作线程
        
        Args:
            output_path: 输出文件路径
            duration: 录制时长(秒)
            min_fps: 最小要求帧率
        """
        writer = None
        frame_size = None
        buffered_frames = []
        buffered_times = []
        selected_fps = self.fps
        start_time = time.time()
        frames_written = 0
        min_required_frames = int(duration * min_fps)
        
        try:
            print(f"[VideoRecorder] 开始录制: {output_path} (时长: {duration}秒, 最小要求: {min_required_frames}帧)")
            
            while (time.time() - start_time) < duration and not self.stop_event.is_set():
                try:
                    # 获取帧 (超时时间稍长以容忍帧率波动)
                    frame, frame_time = self.frame_queue.get(timeout=0.5)
                    
                    # 初始化writer时,先缓冲帧以估算实际fps
                    if writer is None:
                        if frame_size is None:
                            frame_size = (frame.shape[1], frame.shape[0])
                        buffered_frames.append(frame)
                        buffered_times.append(frame_time)
                        
                        # 需要至少两帧估算fps
                        if len(buffered_frames) < 2:
                            continue
                        
                        # Use fixed 5 FPS instead of adaptive estimation
                        # This ensures consistent frame rate across all segments for proper merging
                        selected_fps = 5.0
                        
                        # Log estimated FPS for debugging (but don't use it)
                        elapsed_buffer = buffered_times[-1] - buffered_times[0]
                        if elapsed_buffer > 0:
                            estimated_fps = (len(buffered_frames) - 1) / elapsed_buffer
                            if estimated_fps < 3.0:
                                print(f"[VideoRecorder] ⚠️  Warning: Low frame rate detected ({estimated_fps:.1f} FPS), but using fixed 5 FPS for consistency")
                        
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        writer = cv2.VideoWriter(
                            str(output_path),
                            fourcc,
                            selected_fps,
                            frame_size
                        )
                        
                        if not writer.isOpened():
                            print(f"[VideoRecorder] 错误: 无法打开视频写入器")
                            break
                        
                        for buffered_frame in buffered_frames:
                            writer.write(buffered_frame)
                            frames_written += 1
                        buffered_frames.clear()
                        buffered_times.clear()
                        continue
                    
                    # 写入帧
                    writer.write(frame)
                    frames_written += 1
                    
                except queue.Empty:
                    # 队列为空,继续等待
                    continue
                except Exception as e:
                    print(f"[VideoRecorder] 帧写入错误: {e}")
                    break
            
            if writer is None and buffered_frames and frame_size is not None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                writer = cv2.VideoWriter(
                    str(output_path),
                    fourcc,
                    selected_fps,
                    frame_size
                )
                if writer.isOpened():
                    for buffered_frame in buffered_frames:
                        writer.write(buffered_frame)
                        frames_written += 1
                else:
                    print(f"[VideoRecorder] 错误: 无法打开视频写入器")
                buffered_frames.clear()
                buffered_times.clear()
            
            elapsed = time.time() - start_time
            actual_fps = frames_written / elapsed if elapsed > 0 else 0
            
            # 更新元数据
            self.recording_metadata["end_time"] = time.strftime("%Y%m%d_%H%M%S")
            self.recording_metadata["duration_seconds"] = elapsed
            self.recording_metadata["frame_count"] = frames_written
            self.recording_metadata["actual_fps"] = actual_fps
            
            # 保存元数据到同一文件夹
            metadata_filename = output_path.stem + "_metadata.json"
            metadata_path = self.current_output_dir / metadata_filename
            try:
                with open(metadata_path, 'w') as f:
                    json.dump(self.recording_metadata, f, indent=2)
                print(f"[VideoRecorder] 元数据已保存: {metadata_path}")
            except Exception as e:
                print(f"[VideoRecorder] 保存元数据失败: {e}")
            
            print(f"[VideoRecorder] 录制完成: {output_path}")
            print(f"[VideoRecorder] 录制时长: {elapsed:.2f}秒, 帧数: {frames_written}, 实际fps: {actual_fps:.1f}")
            
            # 检查是否满足最小帧率要求
            if frames_written < min_required_frames:
                print(f"[VideoRecorder] ⚠️  警告: 录制帧数不足! 要求至少{min_required_frames}帧(≥{min_fps}fps), 实际录制{frames_written}帧({actual_fps:.1f}fps)")
            else:
                print(f"[VideoRecorder] ✓ 录制帧数满足要求 ({frames_written} >= {min_required_frames})")
            
        except Exception as e:
            print(f"[VideoRecorder] 录制异常: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # 清理资源
            if writer is not None:
                writer.release()
            
            with self.recording_lock:
                self.is_recording = False
            
            # 清空队列
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    break
    
    def stop(self):
        """停止录制"""
        self.stop_event.set()
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=2.0)
    
    def is_currently_recording(self) -> bool:
        """检查是否正在录制"""
        with self.recording_lock:
            return self.is_recording

