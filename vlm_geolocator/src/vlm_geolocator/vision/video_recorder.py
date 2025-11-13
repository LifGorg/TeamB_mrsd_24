"""视频录制模块"""
import threading
import time
import queue
from pathlib import Path
from typing import Optional
import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("[VideoRecorder] Warning: OpenCV not available, video recording disabled")


class VideoRecorder:
    """视频录制器 - 录制固定时长的视频片段"""
    
    def __init__(self, output_dir: str, fps: int = 30):
        """
        初始化录制器
        
        Args:
            output_dir: 视频输出目录
            fps: 帧率 (默认30fps)
        """
        if not CV2_AVAILABLE:
            raise RuntimeError("OpenCV is required for video recording")
        
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        
        # 录制状态
        self.is_recording = False
        self.recording_lock = threading.Lock()
        
        # 帧缓冲队列
        self.frame_queue = queue.Queue(maxsize=300)  # 最多缓冲10秒(30fps)
        
        # 录制线程
        self.recording_thread = None
        self.stop_event = threading.Event()
        
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
    
    def start_recording(self, duration: float = 5.0) -> Optional[str]:
        """
        开始录制指定时长的视频
        
        Args:
            duration: 录制时长(秒)
            
        Returns:
            视频文件路径,如果已在录制则返回None
        """
        with self.recording_lock:
            if self.is_recording:
                return None
            
            self.is_recording = True
            
            # 生成输出文件名
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = self.output_dir / f"recording_{timestamp}.mp4"
            
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
                args=(output_path, duration),
                daemon=True
            )
            self.recording_thread.start()
            
            return str(output_path)
    
    def _recording_worker(self, output_path: Path, duration: float):
        """
        录制工作线程
        
        Args:
            output_path: 输出文件路径
            duration: 录制时长(秒)
        """
        writer = None
        frame_size = None
        start_time = time.time()
        frames_written = 0
        
        try:
            print(f"[VideoRecorder] 开始录制: {output_path} (时长: {duration}秒)")
            
            while (time.time() - start_time) < duration and not self.stop_event.is_set():
                try:
                    # 获取帧 (超时时间稍长以容忍帧率波动)
                    frame, frame_time = self.frame_queue.get(timeout=0.5)
                    
                    # 初始化writer
                    if writer is None:
                        frame_size = (frame.shape[1], frame.shape[0])
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        writer = cv2.VideoWriter(
                            str(output_path),
                            fourcc,
                            self.fps,
                            frame_size
                        )
                        
                        if not writer.isOpened():
                            print(f"[VideoRecorder] 错误: 无法打开视频写入器")
                            break
                    
                    # 写入帧
                    writer.write(frame)
                    frames_written += 1
                    
                except queue.Empty:
                    # 队列为空,继续等待
                    continue
                except Exception as e:
                    print(f"[VideoRecorder] 帧写入错误: {e}")
                    break
            
            elapsed = time.time() - start_time
            print(f"[VideoRecorder] 录制完成: {output_path}")
            print(f"[VideoRecorder] 录制时长: {elapsed:.2f}秒, 帧数: {frames_written}, 实际fps: {frames_written/elapsed:.1f}")
            
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

