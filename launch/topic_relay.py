#!/usr/bin/env python3
"""
Simple topic relay script for FV Recorder passthrough
Relays source topics to player topics for consistent user interface
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import threading
import time

class TopicRelay(Node):
    def __init__(self):
        super().__init__('fv_topic_relay')
        
        self.get_logger().info('🔄 Starting FV Topic Relay...')
        
        # Topic mappings: source -> target
        self.topic_mappings = {
            '/fv/d415/color/image_raw': '/fv/recorder/d415/color/image_raw',
            '/fv/d415/depth/image_rect_raw': '/fv/recorder/d415/depth/image_rect_raw',
            '/fv/d405/color/image_raw': '/fv/recorder/d405/color/image_raw',
            '/fv/d405/depth/image_rect_raw': '/fv/recorder/d405/depth/image_rect_raw',
        }
        
        # Create subscribers and publishers
        self.relay_subscribers = {}
        self.relay_publishers = {}
        
        for source_topic, target_topic in self.topic_mappings.items():
            # Create publisher for target topic
            publisher = self.create_publisher(Image, target_topic, 10)
            self.relay_publishers[source_topic] = publisher
            
            # Create subscriber for source topic
            subscriber = self.create_subscription(
                Image,
                source_topic,
                lambda msg, src=source_topic: self.relay_callback(msg, src),
                10
            )
            self.relay_subscribers[source_topic] = subscriber
            
            self.get_logger().info(f'   📡 Relay: {source_topic} -> {target_topic}')
        
        self.get_logger().info('✅ FV Topic Relay initialized')

    def relay_callback(self, msg, source_topic):
        """Callback to relay messages from source to target topic"""
        if source_topic in self.relay_publishers:
            # Simply republish the message to target topic
            self.relay_publishers[source_topic].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    relay_node = TopicRelay()
    
    try:
        rclpy.spin(relay_node)
    except KeyboardInterrupt:
        relay_node.get_logger().info('🛑 Topic Relay shutting down...')
    finally:
        relay_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()