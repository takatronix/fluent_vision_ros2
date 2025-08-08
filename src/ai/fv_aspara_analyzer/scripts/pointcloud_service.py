#!/usr/bin/env python3
"""
点群生成サービスノード
深度画像とカラー画像から点群を生成するサービスを提供
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import struct
import cv2


class PointCloudServiceNode(Node):
    def __init__(self):
        super().__init__('pointcloud_service_node')
        
        # パラメータ
        self.declare_parameter('camera_name', 'd415')
        self.camera_name = self.get_parameter('camera_name').value
        
        # トピック名の設定
        prefix = f'/fv/{self.camera_name}'
        
        # CvBridge
        self.bridge = CvBridge()
        
        # 最新のデータを保持
        self.latest_depth = None
        self.latest_color = None
        self.latest_camera_info = None
        
        # サブスクライバー
        self.depth_sub = self.create_subscription(
            Image,
            f'{prefix}/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        self.color_sub = self.create_subscription(
            Image,
            f'{prefix}/color/image_raw',
            self.color_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            f'{prefix}/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # パブリッシャー（デバッグ用）
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            f'{prefix}/debug_pointcloud',
            10
        )
        
        self.get_logger().info(f'Point cloud service node started for camera: {self.camera_name}')
    
    def depth_callback(self, msg):
        """深度画像のコールバック"""
        self.latest_depth = msg
    
    def color_callback(self, msg):
        """カラー画像のコールバック"""
        self.latest_color = msg
    
    def camera_info_callback(self, msg):
        """カメラ情報のコールバック"""
        self.latest_camera_info = msg
    
    def generate_pointcloud(self, roi=None):
        """
        深度画像から点群を生成
        
        Args:
            roi: (x, y, width, height) or None for full image
        
        Returns:
            PointCloud2メッセージ
        """
        if self.latest_depth is None or self.latest_color is None or self.latest_camera_info is None:
            self.get_logger().warn('Data not ready')
            return None
        
        try:
            # 画像をOpenCV形式に変換
            if self.latest_depth.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth, '16UC1')
                depth_scale = 0.001  # mm to meters
            elif self.latest_depth.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth, '32FC1')
                depth_scale = 1.0  # already in meters
            else:
                self.get_logger().error(f'Unsupported depth encoding: {self.latest_depth.encoding}')
                return None
            
            color_image = self.bridge.imgmsg_to_cv2(self.latest_color, 'bgr8')
            
            # カメラパラメータ
            fx = self.latest_camera_info.k[0]
            fy = self.latest_camera_info.k[4]
            cx = self.latest_camera_info.k[2]
            cy = self.latest_camera_info.k[5]
            
            height, width = depth_image.shape
            
            # ROIの設定
            if roi:
                x_start, y_start, roi_width, roi_height = roi
                x_end = min(x_start + roi_width, width)
                y_end = min(y_start + roi_height, height)
            else:
                x_start, y_start = 0, 0
                x_end, y_end = width, height
            
            # 点群データの作成
            points = []
            colors = []
            
            for v in range(y_start, y_end):
                for u in range(x_start, x_end):
                    depth_value = depth_image[v, u]
                    
                    if depth_value == 0:
                        continue
                    
                    # 深度値をメートルに変換
                    z = float(depth_value) * depth_scale
                    
                    # 有効範囲チェック（0.2m〜2.0m）
                    if z < 0.2 or z > 2.0:
                        continue
                    
                    # 3D座標を計算
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    
                    points.append([x, y, z])
                    
                    # RGB値を取得
                    bgr = color_image[v, u]
                    colors.append([bgr[2], bgr[1], bgr[0]])  # BGR to RGB
            
            if len(points) == 0:
                self.get_logger().warn('No valid points generated')
                return None
            
            # PointCloud2メッセージを作成
            cloud_msg = self.create_pointcloud2_msg(points, colors, self.latest_depth.header)
            
            return cloud_msg
            
        except Exception as e:
            self.get_logger().error(f'Error generating point cloud: {str(e)}')
            return None
    
    def create_pointcloud2_msg(self, points, colors, header):
        """
        PointCloud2メッセージを作成
        
        Args:
            points: 3D点のリスト [[x,y,z], ...]
            colors: RGB値のリスト [[r,g,b], ...]
            header: メッセージヘッダー
        
        Returns:
            PointCloud2メッセージ
        """
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        
        # フィールド定義
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        cloud_msg.fields = fields
        
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        
        # データをパック
        data = []
        for point, color in zip(points, colors):
            x, y, z = point
            r, g, b = color
            
            # RGB値を32ビット整数にパック
            rgb = (int(r) << 16) | (int(g) << 8) | int(b)
            
            # floatとuint32をバイト列に変換
            data.append(struct.pack('fffI', x, y, z, rgb))
        
        cloud_msg.data = b''.join(data)
        
        return cloud_msg
    
    def test_pointcloud_generation(self):
        """テスト用：定期的に点群を生成してパブリッシュ"""
        cloud_msg = self.generate_pointcloud()
        if cloud_msg:
            self.pointcloud_pub.publish(cloud_msg)
            self.get_logger().info(f'Published point cloud with {cloud_msg.width} points')


def main(args=None):
    import sys
    rclpy.init(args=args)
    
    # コマンドライン引数でカメラ名を指定可能
    # 例: python3 pointcloud_service.py d415
    # 例: python3 pointcloud_service.py d405
    if len(sys.argv) > 1:
        camera_name = sys.argv[1]
        node = PointCloudServiceNode()
        node.camera_name = camera_name
        # トピック名を再設定
        prefix = f'/fv/{camera_name}'
        node.depth_sub = node.create_subscription(
            Image,
            f'{prefix}/aligned_depth_to_color/image_raw',
            node.depth_callback,
            10
        )
        node.color_sub = node.create_subscription(
            Image,
            f'{prefix}/color/image_raw',
            node.color_callback,
            10
        )
        node.camera_info_sub = node.create_subscription(
            CameraInfo,
            f'{prefix}/color/camera_info',
            node.camera_info_callback,
            10
        )
        node.get_logger().info(f'Camera name overridden to: {camera_name}')
    else:
        node = PointCloudServiceNode()
    
    # テスト用タイマー（1秒ごとに点群生成）
    timer = node.create_timer(1.0, node.test_pointcloud_generation)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()