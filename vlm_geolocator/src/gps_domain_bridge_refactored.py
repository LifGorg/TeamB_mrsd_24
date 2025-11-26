#!/usr/bin/env python3
"""
Refactored GPS Domain Bridge Node

Features:
- Bridge Domain 100 to Domain 0
- Forward drone GPS and target GPS
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import threading
import time


class GPSDomainBridge(Node):
    """GPS Domain Bridge"""
    
    def __init__(self):
        super().__init__('gps_domain_bridge')
        
        # Statistics
        self.drone_gps_count = 0
        self.casualties_gps_count = 0
        self.casualty_geolocated_count = 0
        self.last_log_time = time.time()
        
        # QoS configuration
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._print_header()
    
    def _print_header(self):
        """Print startup information"""
        self.get_logger().info('='*60)
        self.get_logger().info('🌉 GPS Domain Bridge Starting')
        self.get_logger().info('='*60)
        self.get_logger().info('Source Domain: 100 (UAV)')
        self.get_logger().info('Target Domain: 0 (Downstream)')
        self.get_logger().info('Bridging topics:')
        self.get_logger().info('  - /uav_mrsd_1/drone_gps')
        self.get_logger().info('  - /uav_mrsd_1/casualties_gps')
        self.get_logger().info('  - /casualty_geolocated')
        self.get_logger().info('='*60)
    
    def setup_bridge(self, source_node, target_node):
        """Set up bridge"""
        # Domain 100 subscribers
        self.drone_gps_sub = source_node.create_subscription(
            NavSatFix,
            '/uav_mrsd_1/drone_gps',
            self._drone_gps_callback,
            self.qos
        )
        
        self.casualties_gps_sub = source_node.create_subscription(
            NavSatFix,
            '/uav_mrsd_1/casualties_gps',
            self._casualties_gps_callback,
            self.qos
        )
        
        self.casualty_geolocated_sub = source_node.create_subscription(
            NavSatFix,
            '/casualty_geolocated',
            self._casualty_geolocated_callback,
            self.qos
        )
        
        # Domain 0 publishers
        self.drone_gps_pub = target_node.create_publisher(
            NavSatFix,
            '/uav_mrsd_1/drone_gps',
            self.qos
        )
        
        self.casualties_gps_pub = target_node.create_publisher(
            NavSatFix,
            '/uav_mrsd_1/casualties_gps',
            self.qos
        )
        
        self.casualty_geolocated_pub = target_node.create_publisher(
            NavSatFix,
            '/casualty_geolocated',
            self.qos
        )
        
        self.get_logger().info('✅ Bridge setup complete')
    
    def _drone_gps_callback(self, msg):
        """Drone GPS callback"""
        self.drone_gps_pub.publish(msg)
        self.drone_gps_count += 1
        
        # Print statistics every 10 seconds
        current_time = time.time()
        if current_time - self.last_log_time >= 10.0:
            self.get_logger().info(
                f'📊 Statistics: Drone GPS: {self.drone_gps_count}, '
                f'Casualties GPS: {self.casualties_gps_count}, '
                f'Casualty Geolocated: {self.casualty_geolocated_count}'
            )
            self.last_log_time = current_time
        
        self.get_logger().debug(
            f'🛰️  Bridged drone GPS: '
            f'lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
        )
    
    def _casualties_gps_callback(self, msg):
        """Casualties GPS callback"""
        self.casualties_gps_pub.publish(msg)
        self.casualties_gps_count += 1
        
        self.get_logger().info(
            f'🎯 Bridged casualty GPS #{self.casualties_gps_count}: '
            f'lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
        )
    
    def _casualty_geolocated_callback(self, msg):
        """Casualty geolocated callback"""
        self.casualty_geolocated_pub.publish(msg)
        self.casualty_geolocated_count += 1
        
        self.get_logger().info(
            f'📍 Bridged casualty geolocated #{self.casualty_geolocated_count}: '
            f'lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
        )


def main(args=None):
    """Main function"""
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create bridge node (for logging)
    bridge = GPSDomainBridge()
    
    # Domain 100 (source)
    source_context = rclpy.Context()
    rclpy.init(context=source_context, domain_id=100)
    source_node = rclpy.create_node('gps_bridge_source_d100', context=source_context)
    
    # Domain 0 (target)
    target_context = rclpy.Context()
    rclpy.init(context=target_context, domain_id=0)
    target_node = rclpy.create_node('gps_bridge_target_d0', context=target_context)
    
    # Setup bridge
    bridge.setup_bridge(source_node, target_node)
    
    # Create executors
    source_executor = rclpy.executors.SingleThreadedExecutor(context=source_context)
    target_executor = rclpy.executors.SingleThreadedExecutor(context=target_context)
    
    source_executor.add_node(source_node)
    target_executor.add_node(target_node)
    
    # Run in separate threads
    source_thread = threading.Thread(target=source_executor.spin, daemon=True)
    target_thread = threading.Thread(target=target_executor.spin, daemon=True)
    
    bridge.get_logger().info('🚀 Starting bridge threads...')
    source_thread.start()
    target_thread.start()
    bridge.get_logger().info('✅ Bridge is running!')
    bridge.get_logger().info('Press Ctrl+C to stop')
    
    try:
        source_thread.join()
        target_thread.join()
    except KeyboardInterrupt:
        bridge.get_logger().info('🛑 Shutting down bridge...')
    finally:
        source_executor.shutdown()
        target_executor.shutdown()
        source_node.destroy_node()
        target_node.destroy_node()
        bridge.destroy_node()
        source_context.shutdown()
        target_context.shutdown()
        rclpy.shutdown()
        bridge.get_logger().info('✅ Bridge stopped')


if __name__ == '__main__':
    main()
