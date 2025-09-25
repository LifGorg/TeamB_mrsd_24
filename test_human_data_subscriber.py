#!/usr/bin/env python3
"""
简单的测试脚本，用于验证 HumanDataMsg 是否正确发布
"""

import rclpy
from rclpy.node import Node
from dtc_network_msgs.msg import HumanDataMsg

class HumanDataSubscriber(Node):
    def __init__(self):
        super().__init__('human_data_subscriber_test')
        
        self.subscription = self.create_subscription(
            HumanDataMsg,
            '/human_data',
            self.human_data_callback,
            10
        )
        
        self.msg_count = 0
        self.get_logger().info('HumanDataSubscriber 测试节点已启动，等待 HumanDataMsg...')
    
    def human_data_callback(self, msg):
        self.msg_count += 1
        
        # 提取信息
        has_gps = msg.gps_data.status.status >= 0
        num_compressed_images = len(msg.compressed_images)
        num_raw_images = len(msg.raw_images)
        system_name = msg.system
        
        # 如果有 GPS 数据，显示位置
        gps_info = "无GPS"
        if has_gps and msg.gps_data.latitude != 0:
            gps_info = f"GPS: ({msg.gps_data.latitude:.6f}, {msg.gps_data.longitude:.6f})"
        
        # 如果有压缩图像，显示大小
        image_info = "无图像"
        if num_compressed_images > 0:
            image_size = len(msg.compressed_images[0].data)
            image_info = f"压缩图像大小: {image_size/1024:.1f} KB"
        
        self.get_logger().info(
            f"[消息 #{self.msg_count}] 系统: {system_name}, "
            f"{gps_info}, {image_info}, "
            f"压缩图像数: {num_compressed_images}, 原始图像数: {num_raw_images}"
        )

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = HumanDataSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info(f'测试结束，共收到 {subscriber.msg_count} 条 HumanDataMsg 消息')
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


