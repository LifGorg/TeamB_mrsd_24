"""ROS2发布器管理"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, CompressedImage
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from typing import List, Dict, Any

try:
    from dtc_network_msgs.msg import HumanDataMsg
except ImportError:
    HumanDataMsg = None


class ROSPublisherManager:
    """ROS2发布器管理器"""
    
    def __init__(self, node: Node, qos_config: Dict[str, Any]):
        """
        Args:
            node: ROS2节点
            qos_config: QoS配置字典
        """
        self.node = node
        self.qos_config = qos_config
        self.publishers = {}
        
        # 创建QoS配置
        self.mavros_qos = self._create_qos_profile(qos_config['mavros'])
        
        # 创建casualty_geolocated专用QoS (如果配置存在)
        if 'casualty_geolocated' in qos_config:
            self.casualty_qos = self._create_qos_profile(qos_config['casualty_geolocated'])
        else:
            # 默认使用RELIABLE + TRANSIENT_LOCAL以兼容domain_bridge
            self.casualty_qos = self._create_qos_profile({
                'reliability': 'reliable',
                'durability': 'transient_local',
                'history': 'keep_last',
                'depth': 10
            })
    
    def _create_qos_profile(self, config: Dict[str, Any]) -> QoSProfile:
        """创建QoS配置"""
        reliability_map = {
            'best_effort': ReliabilityPolicy.BEST_EFFORT,
            'reliable': ReliabilityPolicy.RELIABLE
        }
        durability_map = {
            'volatile': DurabilityPolicy.VOLATILE,
            'transient_local': DurabilityPolicy.TRANSIENT_LOCAL
        }
        history_map = {
            'keep_last': HistoryPolicy.KEEP_LAST,
            'keep_all': HistoryPolicy.KEEP_ALL
        }
        
        return QoSProfile(
            reliability=reliability_map.get(config['reliability'], ReliabilityPolicy.BEST_EFFORT),
            durability=durability_map.get(config['durability'], DurabilityPolicy.VOLATILE),
            history=history_map.get(config['history'], HistoryPolicy.KEEP_LAST),
            depth=config.get('depth', 10)
        )
    
    def create_drone_gps_publisher(self, topic_name: str):
        """创建无人机GPS发布器"""
        self.publishers['drone_gps'] = self.node.create_publisher(
            NavSatFix,
            topic_name,
            self.mavros_qos
        )
    
    def create_casualties_gps_publisher(self, topic_name: str):
        """创建目标GPS发布器"""
        if HumanDataMsg is None:
            self.node.get_logger().error('HumanDataMsg不可用，无法创建目标GPS发布器')
            return
        
        self.publishers['casualties_gps'] = self.node.create_publisher(
            HumanDataMsg,
            topic_name,
            10
        )
    
    def create_casualty_geolocated_publisher(self, topic_name: str):
        """创建伤员地理定位NavSatFix发布器"""
        self.publishers['casualty_geolocated'] = self.node.create_publisher(
            NavSatFix,
            topic_name,
            self.casualty_qos  # 使用专用的QoS配置
        )
    
    def publish_drone_gps(self, gps_data: NavSatFix):
        """发布无人机GPS"""
        if 'drone_gps' not in self.publishers:
            return
        
        nav_msg = NavSatFix()
        nav_msg.header = gps_data.header
        nav_msg.header.stamp = self.node.get_clock().now().to_msg()
        nav_msg.status = gps_data.status
        nav_msg.latitude = gps_data.latitude
        nav_msg.longitude = gps_data.longitude
        nav_msg.altitude = gps_data.altitude
        nav_msg.position_covariance = gps_data.position_covariance
        nav_msg.position_covariance_type = gps_data.position_covariance_type
        
        self.publishers['drone_gps'].publish(nav_msg)
    
    def publish_casualties_gps(
        self,
        latitude: float,
        longitude: float,
        frame: np.ndarray = None
    ):
        """发布目标GPS"""
        if 'casualties_gps' not in self.publishers or HumanDataMsg is None:
            return
        
        # 创建HumanDataMsg
        human_msg = HumanDataMsg()
        
        # 设置header
        human_msg.header.stamp = self.node.get_clock().now().to_msg()
        human_msg.header.frame_id = 'casualty_map'
        human_msg.stamp = self.node.get_clock().now().to_msg()
        human_msg.system = 'vlm_geolocator'
        
        # 设置GPS数据
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.node.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'casualty_map'
        gps_msg.latitude = latitude
        gps_msg.longitude = longitude
        gps_msg.altitude = 0.0
        gps_msg.status.status = 0  # STATUS_FIX
        gps_msg.status.service = 1  # SERVICE_GPS
        gps_msg.position_covariance_type = 0
        human_msg.gps_data = gps_msg
        
        # 设置图像数据
        if frame is not None:
            try:
                _, jpeg_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                
                # 创建两个CompressedImage
                for _ in range(2):
                    compressed_img = CompressedImage()
                    compressed_img.header.stamp = self.node.get_clock().now().to_msg()
                    compressed_img.header.frame_id = 'camera'
                    compressed_img.format = 'jpeg'
                    compressed_img.data = jpeg_data.tobytes()
                    human_msg.compressed_images.append(compressed_img)
            except Exception as e:
                self.node.get_logger().warn(f'图像压缩失败: {e}')
        
        human_msg.raw_images = []
        
        # 发布
        self.publishers['casualties_gps'].publish(human_msg)
        
        return human_msg
    
    def publish_casualty_geolocated(
        self,
        latitude: float,
        longitude: float,
        altitude: float = 0.0
    ):
        """发布伤员地理定位NavSatFix消息"""
        if 'casualty_geolocated' not in self.publishers:
            return
        
        # 创建NavSatFix消息
        nav_msg = NavSatFix()
        nav_msg.header.stamp = self.node.get_clock().now().to_msg()
        nav_msg.header.frame_id = 'map'
        nav_msg.latitude = latitude
        nav_msg.longitude = longitude
        nav_msg.altitude = altitude
        nav_msg.status.status = 0  # STATUS_FIX
        nav_msg.status.service = 1  # SERVICE_GPS
        nav_msg.position_covariance_type = 0
        
        # 发布
        self.publishers['casualty_geolocated'].publish(nav_msg)
        
        return nav_msg

