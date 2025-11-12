"""ROS2订阅器管理"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Bool
from rclpy.qos import QoSProfile
from typing import Callable, Dict, Any


class ROSSubscriberManager:
    """ROS2订阅器管理器"""
    
    def __init__(self, node: Node, qos_profile: QoSProfile):
        """
        Args:
            node: ROS2节点
            qos_profile: QoS配置
        """
        self.node = node
        self.qos = qos_profile
        self.subscribers = {}
    
    def subscribe_gps(self, topic_name: str, callback: Callable):
        """订阅GPS话题"""
        self.subscribers['gps'] = self.node.create_subscription(
            NavSatFix,
            topic_name,
            callback,
            self.qos
        )
    
    def subscribe_compass(self, topic_name: str, callback: Callable):
        """订阅罗盘话题"""
        self.subscribers['compass'] = self.node.create_subscription(
            Float64,
            topic_name,
            callback,
            self.qos
        )
    
    def subscribe_altitude(self, topic_name: str, callback: Callable):
        """订阅高度话题"""
        self.subscribers['altitude'] = self.node.create_subscription(
            Float64,
            topic_name,
            callback,
            self.qos
        )
    
    def subscribe_gimbal(self, topic_name: str, callback: Callable):
        """订阅云台姿态话题"""
        self.subscribers['gimbal'] = self.node.create_subscription(
            Vector3,
            topic_name,
            callback,
            10  # 默认QoS
        )
    
    def subscribe_trigger(self, topic_name: str, callback: Callable):
        """订阅触发话题"""
        self.subscribers['trigger'] = self.node.create_subscription(
            Bool,
            topic_name,
            callback,
            10
        )

