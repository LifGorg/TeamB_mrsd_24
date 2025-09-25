#!/usr/bin/env python3
"""
中心像素GPS估计节点 - 简化版本
==============================

假设目标始终位于图像平面中心，持续进行GPS估计并发布结果。
不使用YOLO检测，直接对图像中心像素进行GPS坐标估计。

简化逻辑：
- 不使用聚类算法
- 始终发布估计的GPS位置
- 实时处理，无缓存或历史数据管理
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import cv2
import math
import time
import threading
import numpy as np
from collections import deque
from typing import Dict, List, Tuple, Optional, Any
import logging

# ROS2 message types
from sensor_msgs.msg import Image, NavSatFix, CompressedImage, CameraInfo
from geometry_msgs.msg import Point, PoseStamped, Vector3, PointStamped
from std_msgs.msg import Bool, Float32, String, Float64, Header
from cv_bridge import CvBridge

# Local modules
from .gps_manager import GPSManagerFixed

logger = logging.getLogger(__name__)


class CenterPixelGPSNode(Node):
    """中心像素GPS估计节点 - 简化版本，无聚类，始终发布GPS估计"""
    
    def __init__(self):
        super().__init__('center_pixel_gps_estimator')
        
        # Initialize logging
        self._setup_logging()
        
        # Load parameters
        self._load_parameters()
        
        # Initialize components
        self._initialize_components()
        
        # Setup ROS2 interfaces
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_timers()
        
        # State management
        self._initialize_state()
        
        # Start processing threads
        self._start_processing_threads()
        
        self.get_logger().info("CenterPixelGPSNode 初始化成功")

    def _setup_logging(self):
        """配置日志记录"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

    def _load_parameters(self):
        """加载和验证ROS2参数"""
        # 声明所有参数
        self.declare_parameters(
            namespace='',
            parameters=[
                # 节点标识和诊断
                ('node_name', 'center_pixel_gps_estimator'),
                ('publish_diagnostics', True),
                ('diagnostics_period', 10.0),
                
                # 估计频率参数
                ('estimation.frequency', 10.0),  # Hz - GPS估计频率
                ('estimation.enable_continuous', True),  # 是否持续估计
                
                # 图像参数
                ('image.width', 640),
                ('image.height', 512),
                
                # 同步参数
                ('sync.max_sync_time_diff', 0.1),
                ('sync.buffer_size', 20),
                
                # 输出参数
                ('output.jpeg_quality', 10),
                ('output.publish_local_enu', False),
                ('output.publish_camera_info', False),
                ('output.sigma_lat_lon_m', 5.0),
                ('output.publish_center_marker', False),  # 是否发布中心标记
                
                # 相机参数
                ('camera.mode', 'EO'),
                ('camera.auto_scale_with_resolution', True),
                ('camera.digital_zoom_factor', 1.0),
                
                # 相机内参 - EO
                ('camera_intrinsics.EO.fx', 515.1042901585477),
                ('camera_intrinsics.EO.fy', 515.1042901585475),
                ('camera_intrinsics.EO.cx', 320.0),
                ('camera_intrinsics.EO.cy', 256.0),
                ('camera_intrinsics.EO.distortion', [0.0, 0.0, 0.0, 0.0, 0.0]),
                ('camera_intrinsics.EO.calibration_resolution', [640, 512]),
                
                # 相机内参 - IR
                ('camera_intrinsics.IR.fx', 2267.0),
                ('camera_intrinsics.IR.fy', 1593.0),
                ('camera_intrinsics.IR.cx', 640.0),
                ('camera_intrinsics.IR.cy', 360.0),
                ('camera_intrinsics.IR.distortion', [0.0, 0.0, 0.0, 0.0, 0.0]),
                ('camera_intrinsics.IR.calibration_resolution', [1280, 720]),
                
                # GPS估计 - 简化版本，无聚类
                ('gps_estimation.simple_mode', True),
                
                # 话题名称
                ('topics.mavros_gps', '/dtc_mrsd/mavros/global_position/global'),
                ('topics.mavros_altitude', '/dtc_mrsd/mavros/global_position/rel_alt'),
                ('topics.mavros_heading', '/dtc_mrsd/mavros/global_position/compass_hdg'),
                ('topics.gimbal_attitude', '/gimbal_attitude'),
                ('topics.camera_mode', '/camera_mode'),
                ('topics.image_compressed', '/center_gps/image_compressed'),
                ('topics.camera_info', '/center_gps/camera_info'),
                ('topics.target_gps', '/center_gps/target_gps'),
                ('topics.target_local_enu', '/center_gps/target_local_enu'),
                ('topics.center_marker', '/center_gps/center_marker'),
                
                # QoS设置 - 传感器数据
                ('qos.sensor_data.reliability', 'best_effort'),
                ('qos.sensor_data.durability', 'volatile'),
                ('qos.sensor_data.depth', 1),
                
                # QoS设置 - 状态数据
                ('qos.state_data.reliability', 'reliable'),
                ('qos.state_data.durability', 'volatile'),
                ('qos.state_data.depth', 10),
                
                # QoS设置 - 目标数据
                ('qos.target_data.reliability', 'reliable'),
                ('qos.target_data.durability', 'volatile'),
                ('qos.target_data.depth', 5),
                
                # 性能监控
                ('performance.enable_profiling', False),
                ('performance.profiling_report_interval', 300.0),
                ('performance.log_performance_stats', True),
                ('performance.stats_log_interval', 30.0),
                
                # 日志配置
                ('logging.level', 'INFO'),
                ('logging.log_to_file', False),
                ('logging.log_file_path', '/tmp/center_pixel_gps.log'),
            ]
        )
        
        # 提取参数值
        self.config = self._extract_config()
        
        # 基于参数配置日志
        self._configure_logging()
        
        self.get_logger().info(f"已加载 {self.config['camera']['mode']} 相机配置")

    def _extract_config(self) -> Dict[str, Any]:
        """将参数提取到有组织的配置字典中"""
        config = {}
        
        # 参数组
        param_groups = ['diagnostics', 'estimation', 'image', 'sync', 'output', 
                       'camera', 'gps_estimation', 'topics', 'qos', 'performance', 'logging']
        
        for group in param_groups:
            config[group] = {}
            
        # 添加节点级参数
        config['node_name'] = self.get_parameter('node_name').value
        config['publish_diagnostics'] = self.get_parameter('publish_diagnostics').value
        config['diagnostics_period'] = self.get_parameter('diagnostics_period').value
        
        # 提取参数
        for param_descriptor in self._parameters.values():
            param_name = param_descriptor.name
            value = self.get_parameter(param_name).value
            
            # 跳过已处理的节点级参数
            if param_name in ['node_name', 'publish_diagnostics', 'diagnostics_period']:
                continue
            
            # 分组参数
            if param_name.startswith('estimation.'):
                config['estimation'][param_name[11:]] = value
            elif param_name.startswith('image.'):
                config['image'][param_name[6:]] = value
            elif param_name.startswith('sync.'):
                config['sync'][param_name[5:]] = value
            elif param_name.startswith('output.'):
                config['output'][param_name[7:]] = value
            elif param_name.startswith('camera.') and not param_name.startswith('camera_intrinsics.'):
                config['camera'][param_name[7:]] = value
            elif param_name.startswith('gps_estimation.'):
                config['gps_estimation'][param_name[15:]] = value
            elif param_name.startswith('topics.'):
                config['topics'][param_name[7:]] = value
            elif param_name.startswith('qos.'):
                # 解析QoS层次结构: qos.sensor_data.reliability
                parts = param_name.split('.')
                if len(parts) >= 3:
                    qos_category = parts[1]  # sensor_data, state_data, target_data
                    qos_param = parts[2]     # reliability, durability, depth
                    if qos_category not in config['qos']:
                        config['qos'][qos_category] = {}
                    config['qos'][qos_category][qos_param] = value
            elif param_name.startswith('performance.'):
                config['performance'][param_name[12:]] = value
            elif param_name.startswith('logging.'):
                config['logging'][param_name[8:]] = value
        
        # 添加相机内参
        config['camera_intrinsics'] = {
            'EO': {
                'fx': self.get_parameter('camera_intrinsics.EO.fx').value,
                'fy': self.get_parameter('camera_intrinsics.EO.fy').value,
                'cx': self.get_parameter('camera_intrinsics.EO.cx').value,
                'cy': self.get_parameter('camera_intrinsics.EO.cy').value,
                'distortion': self.get_parameter('camera_intrinsics.EO.distortion').value,
                'calibration_resolution': self.get_parameter('camera_intrinsics.EO.calibration_resolution').value,
            },
            'IR': {
                'fx': self.get_parameter('camera_intrinsics.IR.fx').value,
                'fy': self.get_parameter('camera_intrinsics.IR.fy').value,
                'cx': self.get_parameter('camera_intrinsics.IR.cx').value,
                'cy': self.get_parameter('camera_intrinsics.IR.cy').value,
                'distortion': self.get_parameter('camera_intrinsics.IR.distortion').value,
                'calibration_resolution': self.get_parameter('camera_intrinsics.IR.calibration_resolution').value,
            }
        }
        
        return config

    def _configure_logging(self):
        """基于参数设置配置日志"""
        try:
            # 映射日志级别字符串到日志常量
            level_map = {
                'DEBUG': logging.DEBUG,
                'INFO': logging.INFO,
                'WARN': logging.WARNING,
                'WARNING': logging.WARNING,
                'ERROR': logging.ERROR,
                'CRITICAL': logging.CRITICAL
            }
            
            log_level = level_map.get(self.config['logging']['level'], logging.INFO)
            
            # 配置根日志记录器
            logger = logging.getLogger(__name__)
            logger.setLevel(log_level)
            
            # 如果需要，添加文件处理器
            if self.config['logging']['log_to_file']:
                file_handler = logging.FileHandler(self.config['logging']['log_file_path'])
                file_handler.setLevel(log_level)
                formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
                file_handler.setFormatter(formatter)
                logger.addHandler(file_handler)
                
        except Exception as e:
            self.get_logger().warn(f"配置日志失败: {e}")

    def _create_qos_profile(self, qos_type: str) -> QoSProfile:
        """从配置创建QoS配置文件"""
        qos_config = self.config['qos'].get(qos_type, {})
        
        # 映射字符串值到ROS2枚举
        reliability_map = {
            'reliable': QoSReliabilityPolicy.RELIABLE,
            'best_effort': QoSReliabilityPolicy.BEST_EFFORT
        }
        
        durability_map = {
            'volatile': QoSDurabilityPolicy.VOLATILE,
            'transient_local': QoSDurabilityPolicy.TRANSIENT_LOCAL
        }
        
        return QoSProfile(
            reliability=reliability_map.get(qos_config.get('reliability', 'reliable'), QoSReliabilityPolicy.RELIABLE),
            durability=durability_map.get(qos_config.get('durability', 'volatile'), QoSDurabilityPolicy.VOLATILE),
            depth=qos_config.get('depth', 10)
        )

    def _initialize_components(self):
        """初始化核心处理组件"""
        # 初始化GPS管理器 - 简化版本
        gps_config = self.config['gps_estimation']
        self.gps_manager = GPSManagerFixed(config=gps_config)
        
        # 初始化CV桥接器
        self.cv_bridge = CvBridge()
        
        # 加载相机内参
        self._setup_camera_intrinsics()

    def _setup_camera_intrinsics(self):
        """基于当前模式和分辨率设置相机内参"""
        camera_mode = self.config['camera']['mode']
        width = self.config['image']['width']
        height = self.config['image']['height']
        
        # 从配置加载内参，包括数字变焦因子
        digital_zoom = self.config['camera']['digital_zoom_factor']
        self.gps_manager.load_intrinsics_from_config(
            {
                'mode': camera_mode,
                'intrinsics': self.config['camera_intrinsics']
            },
            width, height, digital_zoom
        )

    def _setup_subscriptions(self):
        """使用适当的QoS设置ROS2订阅"""
        # 从配置创建QoS配置文件
        sensor_qos = self._create_qos_profile('sensor_data')
        state_qos = self._create_qos_profile('state_data')
        
        # 为并行处理创建回调组
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        
        # MAVROS订阅
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.config['topics']['mavros_gps'],
            self._gps_callback,
            state_qos,
            callback_group=self.sensor_callback_group
        )
        
        self.altitude_sub = self.create_subscription(
            Float64,
            self.config['topics']['mavros_altitude'],
            self._altitude_callback,
            state_qos,
            callback_group=self.sensor_callback_group
        )
        
        self.heading_sub = self.create_subscription(
            Float64,
            self.config['topics']['mavros_heading'],
            self._heading_callback,
            state_qos,
            callback_group=self.sensor_callback_group
        )
        
        # 云台和相机订阅
        self.gimbal_attitude_sub = self.create_subscription(
            Vector3,
            self.config['topics']['gimbal_attitude'],
            self._gimbal_attitude_callback,
            sensor_qos,
            callback_group=self.sensor_callback_group
        )
        
        self.camera_mode_sub = self.create_subscription(
            String,
            self.config['topics']['camera_mode'],
            self._camera_mode_callback,
            state_qos
        )
        
        # 图像订阅 - 直接订阅压缩图像话题
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw_compressed',
            self._image_callback,
            sensor_qos,
            callback_group=self.sensor_callback_group
        )

    def _setup_publishers(self):
        """使用适当的QoS设置ROS2发布器"""
        # 从配置创建QoS配置文件
        target_qos = self._create_qos_profile('target_data')
        
        # 只发布GPS目标
        self.target_gps_pub = self.create_publisher(
            NavSatFix,
            self.config['topics']['target_gps'],
            target_qos
        )

    def _setup_timers(self):
        """设置维护任务的定期计时器"""
        # 性能统计计时器
        if self.config['performance']['log_performance_stats']:
            self.stats_timer = self.create_timer(
                self.config['performance']['stats_log_interval'],
                self._log_performance_stats
            )
        
        # 注意：移除了GPS目标列表发布器和集群清理计时器，因为不再使用聚类

    def _initialize_state(self):
        """初始化节点状态变量"""
        # 带时间戳的传感器数据缓冲区
        buffer_size = self.config['sync']['buffer_size']
        self.gps_readings = deque(maxlen=buffer_size)
        self.altitude_readings = deque(maxlen=buffer_size)
        self.heading_readings = deque(maxlen=buffer_size)
        self.gimbal_attitude_readings = deque(maxlen=buffer_size)
        
        # 当前状态（为了兼容性）
        self.current_gps = None
        self.current_altitude = None
        self.current_heading = None
        self.current_gimbal_attitude = Vector3()
        self.current_camera_mode = self.config['camera']['mode']
        
        # 帧处理
        self.latest_frame_info = None
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.current_frame_timestamp = None
        
        # 性能跟踪
        self.stats = {
            'frames_processed': 0,
            'gps_estimates': 0,
            'sync_failures': 0,
            'last_frame_time': 0,
            'average_processing_time': 0.0
        }

    def _start_processing_threads(self):
        """启动后台处理线程"""
        # 启动GPS估计处理线程
        self.estimation_thread = threading.Thread(
            target=self._estimation_loop,
            daemon=True
        )
        self.estimation_thread.start()

    def _estimation_loop(self):
        """主GPS估计循环"""
        target_fps = self.config['estimation']['frequency']
        frame_interval = 1.0 / target_fps
        last_estimation_time = 0
        
        while rclpy.ok():
            try:
                current_time = time.time()
                
                # 频率限制
                if current_time - last_estimation_time < frame_interval:
                    time.sleep(0.01)
                    continue
                
                # 获取最新帧
                frame = None
                frame_timestamp = None
                with self.frame_lock:
                    if self.current_frame is not None:
                        frame = self.current_frame.copy()
                        frame_timestamp = self.current_frame_timestamp
                
                if frame is None:
                    continue
                
                # 查找同步的传感器读数
                sync_data = self._get_synchronized_sensor_data(frame_timestamp)
                
                last_estimation_time = current_time
                processing_start = time.time()
                
                # 处理中心像素GPS估计
                frame_info = {
                    'frame': frame,
                    'timestamp': frame_timestamp,
                    'sync_data': sync_data
                }
                self._process_center_pixel_estimation(frame_info)
                
                # 更新性能统计
                processing_time = time.time() - processing_start
                self._update_processing_stats(processing_time)
                
            except Exception as e:
                self.get_logger().error(f"估计循环错误: {e}")
                time.sleep(1.0)

    def _get_synchronized_sensor_data(self, frame_timestamp: float) -> Optional[Dict[str, Any]]:
        """获取与帧时间戳同步的传感器数据"""
        max_time_diff = self.config['sync']['max_sync_time_diff']
        
        # 查找最接近的读数
        gps_reading = self._find_closest_reading(self.gps_readings, frame_timestamp)
        altitude_reading = self._find_closest_reading(self.altitude_readings, frame_timestamp)
        heading_reading = self._find_closest_reading(self.heading_readings, frame_timestamp)
        gimbal_reading = self._find_closest_reading(self.gimbal_attitude_readings, frame_timestamp)
        
        # 检查同步质量
        if not all([gps_reading, altitude_reading, heading_reading, gimbal_reading]):
            return None
        
        # 检查时间差
        time_diffs = [
            abs(frame_timestamp - gps_reading[0]),
            abs(frame_timestamp - altitude_reading[0]),
            abs(frame_timestamp - heading_reading[0]),
            abs(frame_timestamp - gimbal_reading[0])
        ]
        
        if max(time_diffs) > max_time_diff:
            self.stats['sync_failures'] += 1
            return None
        
        return {
            'gps': gps_reading[1],
            'altitude': altitude_reading[1],
            'heading': heading_reading[1],
            'gimbal_attitude': gimbal_reading[1],
            'max_time_diff': max(time_diffs)
        }

    def _find_closest_reading(self, readings_deque: deque, target_timestamp: float) -> Optional[Tuple[float, Any]]:
        """查找最接近目标时间戳的传感器读数"""
        if not readings_deque:
            return None
        
        return min(readings_deque, key=lambda x: abs(x[0] - target_timestamp))

    def _process_center_pixel_estimation(self, frame_info: Dict[str, Any]):
        """处理中心像素GPS估计"""
        frame_timestamp = frame_info['timestamp']
        sync_data = frame_info['sync_data']
        frame = frame_info['frame']
        
        if sync_data is None:
            return
        
        # 计算图像中心坐标
        height, width = frame.shape[:2]
        center_x = width / 2.0
        center_y = height / 2.0
        
        # 设置GPS管理器状态
        gps_lat, gps_lon = sync_data['gps']
        heading_rad = math.radians(sync_data['heading'])  # 将度转换为弧度
        gimbal_vec = sync_data['gimbal_attitude']
        gimbal_attitude = (gimbal_vec.x, gimbal_vec.y, gimbal_vec.z)  # 已经是弧度
        
        try:
            self.gps_manager.set_current_state(
                gps=(gps_lat, gps_lon),
                height=sync_data['altitude'],
                heading=heading_rad,
                gimbal_attitude=gimbal_attitude
            )
            
            # 对中心像素进行GPS估计
            gps_result = self.gps_manager.estimate_target_gps(center_x, center_y)
            
            # 创建GPS消息
            gps_msg = self._create_gps_message(gps_result, frame_timestamp)
            
            # 发布估计的GPS位置
            self.target_gps_pub.publish(gps_msg)
            
            self.stats['gps_estimates'] += 1
            
            self.get_logger().debug(
                f"中心像素GPS估计: ({gps_result['estimated_latitude']:.6f}, "
                f"{gps_result['estimated_longitude']:.6f}), "
                f"距离: {gps_result['lateral_distance_m']:.2f}m"
            )
            
        except Exception as e:
            self.get_logger().warning(f"中心像素GPS估计失败: {e}")

    def _create_gps_message(self, gps_result: Dict[str, Any], timestamp: float) -> NavSatFix:
        """从GPS估计结果创建ROS2 NavSatFix消息"""
        msg = NavSatFix()
        msg.header.stamp = self._to_ros_time(timestamp)
        msg.header.frame_id = "estimated_center_target"
        
        msg.latitude = gps_result['estimated_latitude']
        msg.longitude = gps_result['estimated_longitude']
        msg.altitude = 0.0  # 未估计
        
        # 设置状态以指示这是估计值，不是真实的GNSS
        msg.status.status = -1  # 估计的自定义状态
        msg.status.service = 0
        
        # 设置协方差以指示不确定性
        sigma = self.config['output']['sigma_lat_lon_m']
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [0.0] * 9
        msg.position_covariance[0] = sigma ** 2  # 纬度方差
        msg.position_covariance[4] = sigma ** 2  # 经度方差
        msg.position_covariance[8] = 1000.0      # 大的高度方差
        
        return msg



    def _to_ros_time(self, timestamp: float):
        """将时间戳转换为ROS2时间消息"""
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        
        time_msg = self.get_clock().now().to_msg()
        time_msg.sec = sec
        time_msg.nanosec = nanosec
        
        return time_msg

    # 传感器回调
    def _gps_callback(self, msg: NavSatFix):
        """处理GPS消息"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.gps_readings.append((timestamp, (msg.latitude, msg.longitude)))
        self.current_gps = (msg.latitude, msg.longitude)

    def _altitude_callback(self, msg: Float64):
        """处理高度消息"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.altitude_readings.append((timestamp, msg.data))
        self.current_altitude = msg.data

    def _heading_callback(self, msg: Float64):
        """处理航向消息"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.heading_readings.append((timestamp, msg.data))
        self.current_heading = msg.data

    def _gimbal_attitude_callback(self, msg: Vector3):
        """处理云台姿态消息"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.gimbal_attitude_readings.append((timestamp, msg))
        self.current_gimbal_attitude = msg

    def _camera_mode_callback(self, msg: String):
        """处理相机模式变化"""
        new_mode = msg.data.upper()
        if new_mode != self.current_camera_mode and new_mode in ['EO', 'IR']:
            self.get_logger().info(f"相机模式从 {self.current_camera_mode} 切换到 {new_mode}")
            self.current_camera_mode = new_mode
            
            # 更新GPS管理器内参
            try:
                self.gps_manager.set_camera_type(new_mode)
                self._setup_camera_intrinsics()  # 重新加载适当的缩放
            except Exception as e:
                self.get_logger().error(f"更新相机模式失败: {e}")

    def _image_callback(self, msg: CompressedImage):
        """处理传入的压缩图像消息"""
        try:
            # 将压缩图像转换为OpenCV格式
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().warning("解码压缩图像失败")
                return
            
            # 用时间戳存储帧以供处理
            timestamp = self.get_clock().now().nanoseconds / 1e9
            
            # 添加到帧缓冲区以供处理
            with self.frame_lock:
                self.current_frame = frame
                self.current_frame_timestamp = timestamp
                
            # 更新统计
            self.stats['frames_processed'] += 1
            
        except Exception as e:
            self.get_logger().error(f"处理图像错误: {e}")

    # 性能监控
    def _update_processing_stats(self, processing_time: float):
        """更新处理时间统计"""
        n = self.stats['gps_estimates']
        if n == 0:
            self.stats['average_processing_time'] = processing_time
        else:
            old_avg = self.stats['average_processing_time']
            self.stats['average_processing_time'] = (old_avg * (n - 1) + processing_time) / n

    def _log_performance_stats(self):
        """记录性能统计"""
        stats = self.stats.copy()
        
        # 添加组件统计
        gps_stats = self.gps_manager.get_stats()
        
        self.get_logger().info(
            f"性能统计 - "
            f"帧数: {stats['frames_processed']}, "
            f"GPS估计: {stats['gps_estimates']}, "
            f"同步失败: {stats['sync_failures']}, "
            f"平均处理时间: {stats['average_processing_time']:.3f}s, "
            f"GPS成功率: {gps_stats.get('success_rate', 0):.2f}"
        )


    def destroy_node(self):
        """节点的清理关闭"""
        self.get_logger().info("正在关闭 CenterPixelGPSNode...")
        super().destroy_node()


def main(args=None):
    """主入口点"""
    rclpy.init(args=args)
    
    try:
        node = CenterPixelGPSNode()
        
        # 使用MultiThreadedExecutor进行并行回调处理
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f"启动节点失败: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
