"""
Refactored Vision Inference Node

Features:
- GStreamer video reception
- Button/topic triggered capture
- Target detection inference
- GPS coordinate estimation
- ROS2 topic publishing
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import os
import json
from typing import Any, Dict
import numpy as np
try:
    import cv2
except Exception:
    cv2 = None

# Import refactored modules
from vlm_geolocator.core import ConfigManager
from vlm_geolocator.sensors import SensorManager
from vlm_geolocator.gps import GPSCalculator
from vlm_geolocator.vision import VideoFrameReceiver, IsaacDetectorWrapper
from vlm_geolocator.ros_interface import ROSPublisherManager, ROSSubscriberManager


class VisionInferenceNode(Node):
    """Vision Inference Node"""
    
    def __init__(self, config_dir: str = None, context=None):
        super().__init__('vision_inference_node', context=context)
        
        # Record startup time
        self.node_start_time = time.time()
        
        # Load configuration
        self.get_logger().info('Loading configuration...')
        self.config = ConfigManager(config_dir)
        
        # Initialize sensor manager
        self.get_logger().info('Initializing sensor manager...')
        self.sensor_manager = SensorManager(
            timeout=self.config.system.sensor_timeout,
            stale_warning=self.config.system.sensor_stale_warning,
            use_gimbal=False,
            default_gimbal_attitude=(0.0, -np.pi / 2.0, 0.0)
        )
        
        # Initialize GPS calculator
        self.get_logger().info('Initializing GPS calculator...')
        self.gps_calculator = GPSCalculator(
            intrinsic_matrix=self.config.camera.intrinsic_matrix,
            earth_radius_lat=self.config.system.gps_earth_radius_lat
        )
        
        # Initialize video receiver
        self.get_logger().info('Initializing video receiver...')
        self.pipeline_str = self.config.gstreamer.get_pipeline_string()
        self.video_receiver = VideoFrameReceiver(
            pipeline_str=self.pipeline_str,
            frame_callback=self._on_frame_received
        )
        
        # Initialize detector
        self.get_logger().info('Loading detection model...')
        self.detector = IsaacDetectorWrapper(
            model_path=self.config.system.model_path,
            device=self.config.system.model_device,
            save_dir=self.config.system.log_dir
        )
        
        # Initialize ROS2 publishers
        self.get_logger().info('Initializing ROS2 publishers...')
        self.publisher_manager = ROSPublisherManager(self, self.config.ros2.qos)
        self.publisher_manager.create_drone_gps_publisher(
            self.config.ros2.output_topics['drone_gps']
        )
        # Note: HumanDataMsg (casualties_gps) is not used - only NavSatFix (casualty_geolocated)
        # self.publisher_manager.create_casualties_gps_publisher(
        #     self.config.ros2.output_topics['casualties_gps']
        # )
        # Create NavSatFix publisher for casualty geolocated data
        self.publisher_manager.create_casualty_geolocated_publisher(
            self.config.ros2.output_topics['casualty_geolocated']
        )
        
        # Initialize ROS2 subscribers
        self.get_logger().info('Initializing ROS2 subscribers...')
        qos_profile = self.publisher_manager._create_qos_profile(self.config.ros2.qos['mavros'])
        self.subscriber_manager = ROSSubscriberManager(self, qos_profile)
        
        # Subscribe to sensor topics
        self.subscriber_manager.subscribe_gps(
            self.config.ros2.input_topics['gps'],
            self._gps_callback
        )
        self.subscriber_manager.subscribe_compass(
            self.config.ros2.input_topics['compass'],
            self._compass_callback
        )
        self.subscriber_manager.subscribe_altitude(
            self.config.ros2.input_topics['altitude'],
            self._altitude_callback
        )
        # self.subscriber_manager.subscribe_gimbal(
        #     self.config.ros2.input_topics['gimbal'],
        #     self._gimbal_callback
        # )
        self.subscriber_manager.subscribe_trigger(
            '/trigger_capture',
            self._trigger_callback
        )
        
        # Create capture service
        self.capture_service = self.create_service(
            Trigger,
            self.config.ros2.services['trigger_capture'],
            self._handle_capture_service
        )
        
        # Thread pool
        self.thread_pool = ThreadPoolExecutor(
            max_workers=self.config.system.thread_pool_max_workers
        )
        
        # Timers
        # Sensor health check
        self.health_timer = self.create_timer(1.0, self._check_sensor_health)
        # Drone GPS relay
        self.gps_relay_timer = self.create_timer(1.0, self._relay_drone_gps)
        
        # Store latest GPS data for relay
        self._latest_gps_data = None
        
        self._setup_logging()
        self.get_logger().info('✅ Vision inference node initialization complete')
        self._print_system_info()
    
    def _print_system_info(self):
        """Print system information"""
        self.get_logger().info('='*60)
        self.get_logger().info('System Configuration:')
        self.get_logger().info(f'  Camera: {self.config.camera.name}')
        self.get_logger().info(f'  Resolution: {self.config.camera.width}x{self.config.camera.height}')
        self.get_logger().info(f'  Model path: {self.config.system.model_path}')
        self.get_logger().info(f'  Device: {self.config.system.model_device}')
        self.get_logger().info(f'  Log directory: {self.config.system.log_dir}')
        self.get_logger().info('='*60)

    def _setup_logging(self):
        """Initialize session-level logging directory and metadata."""
        try:
            base_dir = Path(self.config.system.log_dir)
        except Exception:
            base_dir = Path.cwd() / 'logs'

        session_tag = time.strftime('%Y%m%d_%H%M%S', time.localtime(self.node_start_time))
        self.log_session_dir = base_dir / 'vision_inference' / session_tag
        self.log_session_dir.mkdir(parents=True, exist_ok=True)

        # Write session metadata
        meta = {
            'session_id': session_tag,
            'node_start_time_epoch': self.node_start_time,
            'node_start_time_iso': time.strftime('%Y-%m-%dT%H:%M:%S%z', time.localtime(self.node_start_time)),
            'camera': {
                'name': getattr(self.config.camera, 'name', None),
                'width': getattr(self.config.camera, 'width', None),
                'height': getattr(self.config.camera, 'height', None),
            },
            'model': {
                'path': getattr(self.config.system, 'model_path', None),
                'device': getattr(self.config.system, 'model_device', None),
            },
            'ros2_topics': {
                'inputs': getattr(self.config.ros2, 'input_topics', {}),
                'outputs': getattr(self.config.ros2, 'output_topics', {}),
            },
            'qos': getattr(self.config.ros2, 'qos', {}),
        }
        try:
            self._write_json_file(self.log_session_dir / 'session_meta.json', meta)
        except Exception as e:
            self.get_logger().warn(f'Logging session metadata failed: {e}')

        try:
            if hasattr(self, 'pipeline_str') and self.pipeline_str:
                with open(self.log_session_dir / 'gstreamer_pipeline.txt', 'w', encoding='utf-8') as f:
                    f.write(self.pipeline_str)
        except Exception as e:
            self.get_logger().warn(f'GStreamer pipeline write failed: {e}')

    def _json_default(self, obj: Any):
        """JSON fallback serializer for numpy and unknown types."""
        try:
            if isinstance(obj, (np.integer, np.floating)):
                return obj.item()
            if isinstance(obj, np.ndarray):
                return obj.tolist()
        except Exception:
            pass
        try:
            return str(obj)
        except Exception:
            return None

    def _write_json_file(self, path: Path, data: Dict[str, Any]):
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False, default=self._json_default)

    def _save_frame_image(self, frame, path: Path) -> Path:
        """Save frame to disk. Prefer JPEG/PNG via OpenCV; fallback to .npy if OpenCV unavailable."""
        try:
            if cv2 is not None:
                # Ensure directory exists
                path.parent.mkdir(parents=True, exist_ok=True)
                # Try writing jpg; if path suffix is not image, default to .jpg
                img_path = path if path.suffix.lower() in {'.jpg', '.jpeg', '.png'} else path.with_suffix('.jpg')
                ok = cv2.imwrite(str(img_path), frame)
                if ok:
                    return img_path
                # fallback to numpy if imwrite fails
            # Fallback: save as .npy
            npy_path = path.with_suffix('.npy')
            np.save(npy_path, frame)
            return npy_path
        except Exception:
            # Last resort: attempt numpy save
            npy_path = path.with_suffix('.npy')
            try:
                np.save(npy_path, frame)
                return npy_path
            except Exception:
                return path
    
    def _on_frame_received(self, frame, timestamp):
        """Video frame reception callback"""
        frame_count = self.video_receiver.get_frame_count()
        if frame_count % 1000 == 0:
            self.get_logger().info(f'✓ Received {frame_count} frames')
    
    def _gps_callback(self, msg):
        """GPS callback"""
        self.sensor_manager.update_gps(msg.latitude, msg.longitude)
        self._latest_gps_data = msg
    
    def _compass_callback(self, msg):
        """Compass callback"""
        self.sensor_manager.update_heading(msg.data)
    
    def _altitude_callback(self, msg):
        """Altitude callback"""
        self.sensor_manager.update_altitude(msg.data)
    
    # def _gimbal_callback(self, msg):
    #     """Gimbal attitude callback"""
    #     self.sensor_manager.update_gimbal(msg.y, msg.x, msg.z)  # roll, pitch, yaw
    
    def _trigger_callback(self, msg):
        """Trigger topic callback"""
        if msg.data:
            self.get_logger().info('📸 Capture triggered from Foxglove')
            self._capture_and_process()
    
    def _handle_capture_service(self, request, response):
        """Capture service callback"""
        self.get_logger().info('📸 Capture triggered via service')
        
        frame_data = self.video_receiver.get_latest_frame()
        if frame_data is None:
            response.success = False
            response.message = 'No video frame available'
            self.get_logger().warn('⚠️  No video frame available')
            return response
        
        frame, timestamp, frame_number = frame_data
        frame_age = time.time() - timestamp
        
        response.success = True
        response.message = f'Capture successful, frame#{frame_number}, size: {frame.shape}, age: {frame_age:.3f}s'
        
        # Async processing
        self._capture_and_process()
        
        return response
    
    def _capture_and_process(self):
        """Capture and process"""
        # Get latest frame
        frame_data = self.video_receiver.get_latest_frame()
        if frame_data is None:
            self.get_logger().warn('⚠️  No video frame available')
            return
        
        frame, timestamp, frame_number = frame_data
        frame_age = time.time() - timestamp
        
        self.get_logger().info(
            f'📷 Captured frame #{frame_number}, size: {frame.shape}, age: {frame_age:.3f}s'
        )
        
        # Get sensor snapshot
        capture_time = time.time()
        sensor_snapshot = self.sensor_manager.get_snapshot(capture_time)
        
        # Print sensor data
        snapshot_str = self.sensor_manager.format_snapshot(sensor_snapshot)
        for line in snapshot_str.split('\n'):
            self.get_logger().info(line)

        # Prepare per-capture logging directory and artifacts
        try:
            capture_ts_ms = int(capture_time * 1000)
            capture_id = f"{capture_ts_ms}_f{frame_number:06d}"
            capture_dir = self.log_session_dir / capture_id
            capture_dir.mkdir(parents=True, exist_ok=True)

            # Save frame info
            frame_info = {
                'frame_number': int(frame_number),
                'frame_timestamp_epoch': float(timestamp),
                'captured_epoch': float(capture_time),
                'frame_age_sec': float(frame_age),
                'frame_shape': list(frame.shape) if hasattr(frame, 'shape') else None,
            }
            self._write_json_file(capture_dir / 'frame_info.json', frame_info)

            # Save sensor snapshot
            self._write_json_file(capture_dir / 'sensor_snapshot.json', sensor_snapshot)

            # Save image
            saved_path = self._save_frame_image(frame, capture_dir / 'frame.jpg')
            self.get_logger().info(f'📝 Captured image and sensor snapshot saved to: {capture_dir} (image: {saved_path.name})')
        except Exception as e:
            self.get_logger().warn(f'Capture stage logging write failed: {e}')

        # Async processing with capture_dir
        self.thread_pool.submit(
            self._process_frame,
            frame, frame_number, sensor_snapshot, capture_dir
        )
    
    def _process_frame(self, frame, frame_number, sensor_snapshot, capture_dir: Path):
        """Process frame (inference + GPS estimation + publishing) with detailed logging."""
        timeline = {
            'process_start_epoch': time.time(),
        }
        try:
            # 1. Run detection
            self.get_logger().info(f'🔄 Running inference (frame #{frame_number})...')
            t0 = time.time()
            detections = self.detector.detect(frame)
            t1 = time.time()
            timeline['inference_start_epoch'] = t0
            timeline['inference_end_epoch'] = t1
            infer_ms = int((t1 - t0) * 1000)

            self.get_logger().info(f'✅ Detected {len(detections)} targets')
            for i, (cx, cy) in enumerate(detections, 1):
                self.get_logger().info(f'  Target {i}: ({cx:.1f}, {cy:.1f})')

            # Write detections log
            try:
                det_payload = {
                    'count': int(len(detections)),
                    'centers': [{'cx': float(cx), 'cy': float(cy)} for (cx, cy) in detections],
                    'inference_ms': infer_ms,
                }
                self._write_json_file(capture_dir / 'detections.json', det_payload)
            except Exception as e:
                self.get_logger().warn(f'Detection results write failed: {e}')

            # 2. GPS estimation
            timeline['geolocation_start_epoch'] = time.time()
            successful_count = 0
            geo_results = []
            for i, (cx, cy) in enumerate(detections, 1):
                per_result = {
                    'index': int(i),
                    'input_center': {'cx': float(cx), 'cy': float(cy)},
                }
                try:
                    gps_result = self.gps_calculator.estimate_from_snapshot(
                        cx, cy, sensor_snapshot
                    )

                    lat = float(gps_result['estimated_latitude'])
                    lon = float(gps_result['estimated_longitude'])
                    dist = float(gps_result['lateral_distance'])

                    self.get_logger().info(
                        f'  Target {i}: GPS ({lat:.6f}, {lon:.6f}), distance: {dist:.1f}m'
                    )

                    # 3. Publish results
                    altitude = sensor_snapshot['altitude'] if sensor_snapshot['altitude'] is not None else 0.0
                    altitude = float(altitude)
                    self.publisher_manager.publish_casualty_geolocated(lat, lon, altitude)

                    successful_count += 1
                    per_result.update({
                        'estimate': {
                            'latitude': lat,
                            'longitude': lon,
                            'altitude': altitude,
                        },
                        'lateral_distance_m': dist,
                        'published_topic': self.config.ros2.output_topics['casualty_geolocated'],
                        'published_at_epoch': time.time(),
                        'status': 'published',
                    })
                except Exception as e:
                    err_msg = str(e)
                    self.get_logger().error(f'  Target {i} GPS estimation failed: {err_msg}')
                    per_result.update({
                        'status': 'error',
                        'error': err_msg,
                    })
                geo_results.append(per_result)

            timeline['geolocation_end_epoch'] = time.time()
            summary = {
                'successful_count': int(successful_count),
                'total_detections': int(len(detections)),
            }

            try:
                self._write_json_file(capture_dir / 'geolocations.json', {
                    'results': geo_results,
                    'summary': summary,
                })
                self._write_json_file(capture_dir / 'timeline.json', timeline)
            except Exception as e:
                self.get_logger().warn(f'Geolocation logging write failed: {e}')

            self.get_logger().info(
                f'✅ Successfully processed {successful_count}/{len(detections)} targets (logs at {capture_dir.name})'
            )

        except Exception as e:
            self.get_logger().error(f'Frame processing failed: {e}')
            import traceback
            tb = traceback.format_exc()
            self.get_logger().error(tb)
            try:
                with open((capture_dir / 'error.txt'), 'w', encoding='utf-8') as f:
                    f.write(str(e))
                    f.write('\n')
                    f.write(tb)
            except Exception:
                pass
    
    def _check_sensor_health(self):
        """Check sensor health status"""
        uptime = time.time() - self.node_start_time
        warnings = self.sensor_manager.check_health(
            uptime,
            self.config.system.warning_interval
        )
        
        for sensor, message in warnings.items():
            self.get_logger().warn(f'⚠️  {message}')
    
    def _relay_drone_gps(self):
        """Relay drone GPS"""
        if self._latest_gps_data is not None:
            self.publisher_manager.publish_drone_gps(self._latest_gps_data)
            self.get_logger().debug(
                f'Relay drone GPS: lat={self._latest_gps_data.latitude:.6f}, '
                f'lon={self._latest_gps_data.longitude:.6f}'
            )
    
    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info('🛑 Shutting down node...')
        self.video_receiver.stop()
        self.thread_pool.shutdown(wait=True)
        super().destroy_node()


def main(args=None):
    """Main function"""
    # Initialize ROS2 (domain controlled by ROS_DOMAIN_ID env var)
    context = rclpy.Context()
    rclpy.init(args=args, context=context)
    
    # Optional: get config directory from command line args
    config_dir = None
    if args and '--config-dir' in args:
        idx = args.index('--config-dir')
        if idx + 1 < len(args):
            config_dir = args[idx + 1]
    
    node = VisionInferenceNode(config_dir, context=context)
    
    # Print domain info
    import os
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
    node.get_logger().info('='*60)
    node.get_logger().info(f'🌐 Running on ROS2 Domain {domain_id}')
    node.get_logger().info('='*60)
    
    try:
        rclpy.spin(node, executor=rclpy.executors.SingleThreadedExecutor(context=context))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        context.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
