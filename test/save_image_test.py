#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Image,
            '/fv_realsense/color/image_raw',
            self.image_callback,
            10)
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Counter for saved images
        self.image_count = 0
        
        self.get_logger().info('🚀 Image Saver started - listening to /fv_realsense/color/image_raw')
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Save image
            filename = f'/tmp/test_image_{self.image_count:03d}.jpg'
            cv2.imwrite(filename, cv_image)
            
            self.get_logger().info(f'💾 Saved image: {filename} (size: {cv_image.shape})')
            
            # Only save first 5 images
            self.image_count += 1
            if self.image_count >= 5:
                self.get_logger().info('✅ Saved 5 test images. Shutting down...')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'❌ Error saving image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 