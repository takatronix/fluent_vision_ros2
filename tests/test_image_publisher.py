#!/usr/bin/env python3

import rclpy
from rclcpp.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(Image, '/fv_mjpeg_image', 10)
        
        # Create timer for publishing
        self.timer = self.create_timer(0.1, self.publish_image)  # 10 FPS
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Create test image (640x480 with a face-like pattern)
        self.test_image = self.create_test_image()
        
        self.get_logger().info('🚀 Test Image Publisher started')
    
    def create_test_image(self):
        # Create a simple test image with a face-like pattern
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Background
        img[:] = (100, 100, 100)
        
        # Face circle
        cv2.circle(img, (320, 240), 80, (255, 200, 150), -1)
        
        # Eyes
        cv2.circle(img, (300, 220), 10, (255, 255, 255), -1)
        cv2.circle(img, (340, 220), 10, (255, 255, 255), -1)
        cv2.circle(img, (300, 220), 5, (0, 0, 0), -1)
        cv2.circle(img, (340, 220), 5, (0, 0, 0), -1)
        
        # Nose
        cv2.circle(img, (320, 240), 5, (255, 150, 150), -1)
        
        # Mouth
        cv2.ellipse(img, (320, 270), (30, 10), 0, 0, 180, (0, 0, 0), 3)
        
        return img
    
    def publish_image(self):
        # Convert OpenCV image to ROS message
        ros_image = self.bridge.cv2_to_imgmsg(self.test_image, "bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "test_camera"
        
        # Publish
        self.publisher.publish(ros_image)
        
        self.get_logger().debug('📤 Published test image')

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 