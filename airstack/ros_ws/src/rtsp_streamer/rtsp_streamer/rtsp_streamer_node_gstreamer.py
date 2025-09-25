import os
import time
import threading
from typing import Optional
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

# Try to import GStreamer
GSTREAMER_AVAILABLE = False
try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstApp', '1.0')
    from gi.repository import Gst, GLib, GstApp
    GSTREAMER_AVAILABLE = True
except Exception as e:
    print(f"GStreamer import failed: {e}")
    GSTREAMER_AVAILABLE = False

class GStreamerStreaming:
    def __init__(self, rtsp_url, width=640, height=512, fps=2):
        self.rtsp_url = rtsp_url
        self.WIDTH = width
        self.HEIGHT = height
        self.fps = fps
        self.is_running = True
        self.frame_count = 0
        self.last_log_time = time.time()
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Create pipeline
        self.pipeline = None
        self.appsink = None
        
        # Create main loop and thread
        self.loop = GLib.MainLoop()
        self.loop_thread = None
        
        # Frame buffer
        self.frame_buffer = deque(maxlen=10)
        self.buffer_lock = threading.Lock()
        
    def get_pipeline_string(self):
        """Get GStreamer pipeline string with hardware acceleration only"""
        # Only try hardware accelerated decoders - fail if none available
        try:
            # First try Jetson-specific hardware decoder (Orin NX)
            jetson_pipeline = (
                f"rtspsrc location={self.rtsp_url} latency=200 buffer-mode=auto protocols=tcp "
                "user-id= user-pw= ! queue ! "
                "rtph265depay ! queue ! h265parse ! queue ! "
                "nvv4l2decoder enable-max-performance=1 ! queue ! "
                "nvvidconv ! "
                f"video/x-raw(memory:NVMM),width={self.WIDTH},height={self.HEIGHT} ! queue ! "
                f"videorate ! video/x-raw(memory:NVMM),framerate={int(self.fps)}/1 ! queue ! "
                "nvvidconv ! video/x-raw ! queue ! "
                "videoconvert ! video/x-raw,format=BGR ! queue ! "
                "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
            )
            
            # Try to create Jetson Pipeline
            test_pipeline = Gst.parse_launch(jetson_pipeline)
            test_pipeline.set_state(Gst.State.NULL)  # Release resources immediately
            print(f"[GStreamer] ✅ Using Jetson Orin NX hardware accelerated decoder for RTSP: {self.rtsp_url}")
            return jetson_pipeline
            
        except Exception as e:
            print(f"[GStreamer] ❌ Jetson nvv4l2decoder failed: {e}")
            print(f"[GStreamer] ❌ Hardware acceleration is required - node will fail")
            raise RuntimeError(f"Jetson hardware acceleration required but not available: {e}")
        
    def start_pipeline(self):
        """Start GStreamer pipeline"""
        print("[DEBUG-GST] start_pipeline called")
        try:
            pipeline_str = self.get_pipeline_string()
            print(f"[DEBUG-GST] Using GStreamer pipeline: {pipeline_str}")
            
            # Create Pipeline
            print("[DEBUG-GST] Creating pipeline from string...")
            self.pipeline = Gst.parse_launch(pipeline_str)
            print(f"[DEBUG-GST] Pipeline created: {self.pipeline}")
            
            # Get appsink element
            print("[DEBUG-GST] Getting appsink element...")
            self.appsink = self.pipeline.get_by_name("sink")
            if not self.appsink:
                print("[DEBUG-GST] Error: Could not get appsink element")
                return False
            print(f"[DEBUG-GST] Got appsink: {self.appsink}")
                
            # Set new frame callback
            print("[DEBUG-GST] Connecting new-sample callback...")
            self.appsink.connect("new-sample", self.on_new_sample, None)
            print("[DEBUG-GST] Callback connected")
            
            # Start pipeline
            print("[DEBUG-GST] Setting pipeline state to PLAYING...")
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            print(f"[DEBUG-GST] Pipeline state change result: {ret}")
            if ret == Gst.StateChangeReturn.FAILURE:
                print("[DEBUG-GST] Error: Could not start GStreamer pipeline")
                return False
                
            # Run GLib main loop in a separate thread
            print("[DEBUG-GST] Starting GLib main loop thread...")
            self.loop_thread = threading.Thread(target=self.run_loop)
            self.loop_thread.daemon = True
            self.loop_thread.start()
            print("[DEBUG-GST] GLib main loop thread started")
            
            print("[DEBUG-GST] GStreamer streaming started successfully")
            return True
            
        except Exception as e:
            print(f"[DEBUG-GST] Exception in start_pipeline: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
            
    def on_new_sample(self, sink, data):
        """Process new GStreamer sample"""
        print("[DEBUG-SAMPLE] on_new_sample callback called!")
        try:
            # Get sample
            sample = sink.emit("pull-sample")
            if not sample:
                print("[DEBUG-SAMPLE] ERROR: No sample received from sink.emit")
                return Gst.FlowReturn.ERROR
            print("[DEBUG-SAMPLE] Got sample from sink")
                
            # Get buffer from sample
            buffer = sample.get_buffer()
            if not buffer:
                print("[GStreamer] Warning: No buffer in sample")
                return Gst.FlowReturn.ERROR
            
            # Get timestamp
            timestamp = time.time()
            
            # Get memory from buffer
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                print("[GStreamer] Error: Failed to map buffer")
                return Gst.FlowReturn.ERROR
            
            try:
                # Get buffer size and validate
                buffer_size = map_info.size
                expected_size = self.HEIGHT * self.WIDTH * 3
                if buffer_size != expected_size:
                    print(f"[GStreamer] Warning: Buffer size mismatch. Got {buffer_size}, expected {expected_size}")
                    # Try to proceed anyway if buffer is large enough
                    if buffer_size < expected_size:
                        return Gst.FlowReturn.ERROR
                
                # Copy data and convert to NumPy array
                frame_data = np.ndarray(
                    shape=(self.HEIGHT, self.WIDTH, 3),
                    dtype=np.uint8,
                    buffer=map_info.data
                ).copy()
                
                # Add frame and timestamp to buffer
                with self.buffer_lock:
                    self.frame_buffer.append((frame_data, timestamp))
                    self.frame_count += 1
                    
                # Periodically log frame rate
                current_time = time.time()
                if current_time - self.last_log_time > 5.0:
                    elapsed = current_time - self.last_log_time
                    fps = self.frame_count / elapsed if elapsed > 0 else 0
                    print(f"[GStreamer] Frame rate: {fps:.2f} FPS, buffer size: {len(self.frame_buffer)}")
                    self.frame_count = 0
                    self.last_log_time = current_time
            finally:
                # Always release buffer
                buffer.unmap(map_info)
                
            return Gst.FlowReturn.OK
            
        except Exception as e:
            print(f"Error processing frame: {str(e)}")
            return Gst.FlowReturn.ERROR
            
    def run_loop(self):
        """Run GLib main loop"""
        try:
            self.loop.run()
        except Exception as e:
            print(f"GStreamer loop error: {str(e)}")
            
    def retrieve_image(self):
        """Get the latest image frame and timestamp"""
        with self.buffer_lock:
            print(f"[DEBUG-RETRIEVE] Buffer size: {len(self.frame_buffer)}")
            if not self.frame_buffer:
                print("[DEBUG-RETRIEVE] Buffer is empty!")
                return None, 0
            # Get the latest frame without clearing the entire buffer
            # This matches the behavior of the working streaming.py
            frame, timestamp = self.frame_buffer[-1]
            print(f"[DEBUG-RETRIEVE] Retrieved frame shape: {frame.shape}")
            return frame, timestamp
            
    def shutdown(self):
        """Close GStreamer resources"""
        self.is_running = False
        
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            
        if self.loop and self.loop.is_running():
            self.loop.quit()
            
        if self.loop_thread and self.loop_thread.is_alive():
            self.loop_thread.join(timeout=3)

class RtspStreamerGStreamer(Node):
    def __init__(self):
        # 强制使用系统时间，忽略use_sim_time参数
        # 这是必要的，因为视频流需要实时处理
        super().__init__('rtsp_streamer_node')
        
        print("[DEBUG] RtspStreamerGStreamer.__init__ started")

        # 声明参数
        self.declare_parameter('rtsp_url', 'rtsp://10.3.1.124:8556/ghadron')
        self.declare_parameter('topic', '/image_raw_compressed')
        self.declare_parameter('raw_topic', '/image_raw')
        self.declare_parameter('fps', 2.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 512)
        self.declare_parameter('jpeg_quality', 30)

        # 获取参数
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.raw_topic = self.get_parameter('raw_topic').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value or 2.0
        self.width = self.get_parameter('width').get_parameter_value().integer_value or 640
        self.height = self.get_parameter('height').get_parameter_value().integer_value or 512
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value or 30
        
        print(f"[DEBUG] Parameters loaded:")
        print(f"  - rtsp_url: {self.rtsp_url}")
        print(f"  - topic: {self.topic}")
        print(f"  - raw_topic: {self.raw_topic}")
        print(f"  - fps: {self.fps}")
        print(f"  - width: {self.width}x{self.height}")
        print(f"  - jpeg_quality: {self.jpeg_quality}")

        # 创建发布器
        print(f"[DEBUG] Creating compressed image publisher on topic: {self.topic}")
        self.publisher = self.create_publisher(CompressedImage, self.topic, 2)
        print(f"[DEBUG] Creating raw image publisher on topic: {self.raw_topic}")
        self.raw_publisher = self.create_publisher(Image, self.raw_topic, 2)
        print("[DEBUG] Publishers created successfully")
        
        # 创建CV Bridge
        self.bridge = CvBridge()

        # 尝试使用GStreamer
        print(f"[DEBUG] GSTREAMER_AVAILABLE = {GSTREAMER_AVAILABLE}")
        self.use_gstreamer = GSTREAMER_AVAILABLE
        self.streamer = None
        self.cap = None

        if self.use_gstreamer:
            print('[DEBUG] Attempting to use GStreamer backend')
            self.get_logger().info('✅ 使用GStreamer后端')
            try:
                print(f"[DEBUG] Creating GStreamerStreaming with url={self.rtsp_url}, width={self.width}, height={self.height}, fps={self.fps}")
                self.streamer = GStreamerStreaming(self.rtsp_url, self.width, self.height, self.fps)
                print("[DEBUG] GStreamerStreaming object created, starting pipeline...")
                if not self.streamer.start_pipeline():
                    print("[DEBUG] GStreamer pipeline start failed")
                    self.get_logger().warning('GStreamer启动失败，降级到OpenCV')
                    self.use_gstreamer = False
                    self._setup_opencv()
                else:
                    print("[DEBUG] GStreamer pipeline started successfully")
            except Exception as e:
                print(f"[DEBUG] GStreamer initialization exception: {e}")
                self.get_logger().warning(f'GStreamer初始化失败: {e}，降级到OpenCV')
                self.use_gstreamer = False
                self._setup_opencv()
        else:
            print('[DEBUG] GStreamer not available, using OpenCV backend')
            self.get_logger().info('⚠️ 使用OpenCV后端')
            self._setup_opencv()

        # 定时器
        self.period = 1.0 / max(self.fps, 0.1) # we want 2-3fps
        print(f"[DEBUG] Creating timer with period {self.period:.3f}s (FPS: {self.fps})")
        self.get_logger().info(f'Creating timer with period {self.period:.3f}s (FPS: {self.fps})')
        self.timer = self.create_timer(self.period, self._on_timer)
        self.frame_count = 0
        self.timer_call_count = 0
        print("[DEBUG] Timer created successfully")
        self.get_logger().info('Timer created successfully')

    def _setup_opencv(self):
        """设置OpenCV后端"""
        try:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if self.width > 0:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
            if self.height > 0:
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        except Exception as e:
            self.get_logger().error(f'OpenCV设置失败: {e}')

    def _on_timer(self):
        """定时器回调"""
        self.timer_call_count += 1
        if self.timer_call_count % 10 == 1:  # Print every 10 calls
            print(f"[DEBUG] _on_timer called (count: {self.timer_call_count})")
        
        try:
            if self.use_gstreamer and self.streamer:
                print(f"[DEBUG] Retrieving image from GStreamer (call {self.timer_call_count})")
                frame, timestamp = self.streamer.retrieve_image()
                if frame is not None:
                    print(f"[DEBUG] Got frame from GStreamer, shape: {frame.shape}, publishing...")
                    self._publish_frame(frame)
                else:
                    if self.timer_call_count % 10 == 1:
                        print(f"[DEBUG] No frame from GStreamer (call {self.timer_call_count})")
            elif self.cap:
                print(f"[DEBUG] Reading frame from OpenCV (call {self.timer_call_count})")
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    print(f"[DEBUG] Got frame from OpenCV, shape: {frame.shape}, publishing...")
                    self._publish_frame(frame)
                else:
                    if self.timer_call_count % 10 == 1:
                        print(f"[DEBUG] No frame from OpenCV (call {self.timer_call_count})")
            else:
                if self.timer_call_count % 10 == 1:
                    print(f"[DEBUG] No streamer available (call {self.timer_call_count})")
        except Exception as e:
            print(f"[DEBUG] Exception in _on_timer: {e}")
            self.get_logger().error(f'Error in timer callback: {e}')

    def _publish_frame(self, frame):
        """发布帧"""
        print(f"[DEBUG] _publish_frame called with frame shape: {frame.shape}")
        try:
            # 编码为JPEG
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(max(1, min(self.jpeg_quality, 100)))]
            ok, buf = cv2.imencode('.jpg', frame, encode_params)
            if not ok or buf is None:
                print("[DEBUG] JPEG encoding failed!")
                return
            print(f"[DEBUG] JPEG encoded successfully, size: {len(buf)} bytes")

            # 创建时间戳
            timestamp = self.get_clock().now().to_msg()
            
            # 发布压缩图像
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = timestamp
            compressed_msg.header.frame_id = 'camera'
            compressed_msg.format = 'jpeg'
            compressed_msg.data = buf.tobytes()
            
            print(f"[DEBUG] Publishing compressed message on topic: {self.topic}")
            self.publisher.publish(compressed_msg)
            
            # 发布未压缩图像
            try:
                raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                raw_msg.header.stamp = timestamp
                raw_msg.header.frame_id = 'camera'
                
                print(f"[DEBUG] Publishing raw message on topic: {self.raw_topic}")
                self.raw_publisher.publish(raw_msg)
                print("[DEBUG] Both messages published successfully")
            except Exception as e:
                print(f"[DEBUG] Failed to publish raw image: {e}")
                print("[DEBUG] Compressed message published successfully")
            
            # 定期日志
            self.frame_count += 1
            if self.frame_count % 5 == 0:  # More frequent logging
                backend = "GStreamer" if self.use_gstreamer else "OpenCV"
                print(f"[DEBUG] Published {self.frame_count} frames using {backend}")
                self.get_logger().info(f'Published {self.frame_count} frames using {backend}')
                
        except Exception as e:
            print(f"[DEBUG] Exception in _publish_frame: {e}")
            self.get_logger().error(f'Frame publishing error: {e}')

    def destroy_node(self):
        """清理资源"""
        try:
            if self.streamer:
                self.streamer.shutdown()
            if self.cap:
                self.cap.release()
        finally:
            return super().destroy_node()

def main():
    rclpy.init()
    node = RtspStreamerGStreamer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
