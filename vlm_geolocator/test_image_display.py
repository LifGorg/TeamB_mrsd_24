#!/usr/bin/env python3
"""简单的图像显示测试脚本"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class SimpleImageViewer(Node):
    def __init__(self):
        super().__init__('simple_image_viewer')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/vlm_geolocator/debug/camera_feed/compressed',
            self.image_callback,
            10
        )
        self.frame_count = 0
        self.get_logger().info('Simple Image Viewer started, waiting for images...')
        
    def image_callback(self, msg):
        self.frame_count += 1
        try:
            # 解码 JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is not None:
                # 添加帧信息
                cv2.putText(image, f"Frame: {self.frame_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # 显示
                cv2.imshow('Compressed Feed', image)
                cv2.waitKey(1)
                
                if self.frame_count % 10 == 0:
                    self.get_logger().info(f'Received {self.frame_count} frames')
            else:
                self.get_logger().error('Failed to decode image')
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main():
    rclpy.init()
    viewer = SimpleImageViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

