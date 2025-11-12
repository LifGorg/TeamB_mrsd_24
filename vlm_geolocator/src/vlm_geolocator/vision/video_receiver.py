"""GStreamer视频处理模块"""
import threading
import numpy as np
import time
from typing import Optional, Callable
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


class VideoFrameReceiver:
    """GStreamer视频帧接收器"""
    
    def __init__(self, pipeline_str: str, frame_callback: Optional[Callable] = None):
        """
        Args:
            pipeline_str: GStreamer管道字符串
            frame_callback: 帧回调函数 callback(frame: np.ndarray, timestamp: float)
        """
        Gst.init(None)
        
        self.pipeline_str = pipeline_str
        self.frame_callback = frame_callback
        
        self.pipeline = None
        self.appsink = None
        self.glib_loop = None
        self.glib_thread = None
        
        self.latest_frame = None
        self.latest_frame_timestamp = None
        self.frame_lock = threading.Lock()
        self.frame_count = 0
        
        self._setup_pipeline()
    
    def _setup_pipeline(self):
        """创建GStreamer管道"""
        print(f'[VideoReceiver] Pipeline: {self.pipeline_str}')
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        
        # 尝试查找appsink (先尝试appsink0，然后是sink)
        self.appsink = self.pipeline.get_by_name('appsink0')
        if self.appsink is None:
            self.appsink = self.pipeline.get_by_name('sink')
        
        # 如果还是找不到，遍历所有元素
        if self.appsink is None:
            print('[VideoReceiver] Searching for appsink element...')
            iterator = self.pipeline.iterate_elements()
            while True:
                result, element = iterator.next()
                if result != Gst.IteratorResult.OK:
                    break
                factory = element.get_factory()
                if factory and factory.get_name() == 'appsink':
                    print(f'[VideoReceiver] Found appsink: {element.get_name()}')
                    self.appsink = element
                    break
        
        if self.appsink is None:
            raise RuntimeError('No appsink element found in GStreamer pipeline')
        
        print(f'[VideoReceiver] Using appsink: {self.appsink.get_name()}')
        self.appsink.connect('new-sample', self._on_new_frame)
        
        # 启动管道
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # 在单独线程运行GLib主循环
        self.glib_loop = GLib.MainLoop()
        self.glib_thread = threading.Thread(target=self.glib_loop.run, daemon=True)
        self.glib_thread.start()
    
    def _on_new_frame(self, sink):
        """每帧回调"""
        sample = sink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.OK
        
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        
        # 获取尺寸信息
        struct = caps.get_structure(0)
        width = struct.get_value('width')
        height = struct.get_value('height')
        
        # 转换为numpy array
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if success:
            frame = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            )
            
            current_time = time.time()
            with self.frame_lock:
                self.latest_frame = frame.copy()
                self.latest_frame_timestamp = current_time
                self.frame_count += 1
            
            # 调用回调函数
            if self.frame_callback:
                self.frame_callback(frame.copy(), current_time)
            
            buffer.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def get_latest_frame(self) -> Optional[tuple]:
        """
        获取最新帧
        
        Returns:
            (frame, timestamp, frame_number) 或 None
        """
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return (
                self.latest_frame.copy(),
                self.latest_frame_timestamp,
                self.frame_count
            )
    
    def get_frame_count(self) -> int:
        """获取接收到的帧数"""
        with self.frame_lock:
            return self.frame_count
    
    def stop(self):
        """停止接收"""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.glib_loop:
            self.glib_loop.quit()

