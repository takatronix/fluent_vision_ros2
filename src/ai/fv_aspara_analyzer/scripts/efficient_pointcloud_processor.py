#!/usr/bin/env python3
"""
効率的なポイントクラウド処理ノード
深度画像から直接3D座標を計算し、検出領域のみを処理する

Author: Takashi Otsuka
Date: 2025-01-08
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from typing import Optional, Tuple, List
import time
from collections import deque
from dataclasses import dataclass, field


@dataclass
class DetectionInfo:
    """検出情報を保持するクラス"""
    id: int
    bbox: Tuple[int, int, int, int]  # x, y, width, height
    confidence: float
    timestamp: float
    smooth_bbox: Optional[Tuple[int, int, int, int]] = None
    alpha: float = 1.0  # アニメーション用の透明度


class EfficientPointCloudProcessor(Node):
    """効率的なポイントクラウド処理ノード"""
    
    def __init__(self):
        super().__init__('efficient_pointcloud_processor')
        
        # パラメータ
        self.declare_parameter('camera_name', 'd415')
        self.declare_parameter('smoothing_factor', 0.3)  # スムージング係数
        self.declare_parameter('animation_speed', 0.1)  # アニメーション速度
        self.declare_parameter('min_depth', 0.1)  # 最小深度 (m)
        self.declare_parameter('max_depth', 2.0)  # 最大深度 (m)
        
        self.camera_name = self.get_parameter('camera_name').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.animation_speed = self.get_parameter('animation_speed').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        
        # トピック名の設定
        base_topic = f'/fv/{self.camera_name}'
        
        # サブスクライバー
        self.depth_sub = self.create_subscription(
            Image,
            f'{base_topic}/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            f'{base_topic}/object_detection/detections',
            self.detection_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            f'{base_topic}/depth/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.color_sub = self.create_subscription(
            Image,
            f'{base_topic}/color/image_raw',
            self.color_callback,
            10
        )
        
        # パブリッシャー
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            f'{base_topic}/efficient/pointcloud',
            10
        )
        
        self.visualization_pub = self.create_publisher(
            Image,
            f'{base_topic}/efficient/visualization',
            10
        )
        
        # 内部状態
        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_color: Optional[np.ndarray] = None
        self.detections: deque = deque(maxlen=10)  # 最新10個の検出を保持
        self.intrinsics = None
        
        # タイマー（10Hz）
        self.timer = self.create_timer(0.1, self.process_timer_callback)
        
        self.get_logger().info(f'Efficient PointCloud Processor initialized for {self.camera_name}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """カメラ情報のコールバック"""
        self.camera_info = msg
        # カメラ内部パラメータを抽出
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        }
    
    def depth_callback(self, msg: Image):
        """深度画像のコールバック"""
        try:
            # 深度画像をnumpy配列に変換（単位：mm）
            if msg.encoding == '16UC1':
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            elif msg.encoding == '32FC1':
                depth_32f = self.bridge.imgmsg_to_cv2(msg, '32FC1')
                self.latest_depth = (depth_32f * 1000).astype(np.uint16)
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
    
    def color_callback(self, msg: Image):
        """カラー画像のコールバック"""
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert color image: {e}')
    
    def detection_callback(self, msg: Detection2DArray):
        """検出結果のコールバック"""
        current_time = time.time()
        
        for detection in msg.detections:
            if not detection.results:
                continue
            
            # バウンディングボックスを抽出
            bbox_center_x = detection.bbox.center.position.x
            bbox_center_y = detection.bbox.center.position.y
            bbox_width = detection.bbox.size_x
            bbox_height = detection.bbox.size_y
            
            # 左上座標に変換
            x = int(bbox_center_x - bbox_width / 2)
            y = int(bbox_center_y - bbox_height / 2)
            width = int(bbox_width)
            height = int(bbox_height)
            
            # 検出情報を追加
            detection_info = DetectionInfo(
                id=len(self.detections),
                bbox=(x, y, width, height),
                confidence=detection.results[0].hypothesis.score,
                timestamp=current_time
            )
            
            # スムージング処理
            if self.detections and self.detections[-1].smooth_bbox:
                prev_bbox = self.detections[-1].smooth_bbox
                smooth_x = int(prev_bbox[0] * (1 - self.smoothing_factor) + x * self.smoothing_factor)
                smooth_y = int(prev_bbox[1] * (1 - self.smoothing_factor) + y * self.smoothing_factor)
                smooth_w = int(prev_bbox[2] * (1 - self.smoothing_factor) + width * self.smoothing_factor)
                smooth_h = int(prev_bbox[3] * (1 - self.smoothing_factor) + height * self.smoothing_factor)
                detection_info.smooth_bbox = (smooth_x, smooth_y, smooth_w, smooth_h)
            else:
                detection_info.smooth_bbox = (x, y, width, height)
            
            self.detections.append(detection_info)
    
    def process_timer_callback(self):
        """定期的な処理"""
        if not all([self.latest_depth is not None, 
                   self.intrinsics is not None,
                   len(self.detections) > 0]):
            return
        
        # 最新の検出を処理
        current_time = time.time()
        active_detections = [d for d in self.detections 
                           if current_time - d.timestamp < 2.0]  # 2秒以内の検出のみ
        
        if not active_detections:
            return
        
        # 各検出領域のポイントクラウドを生成
        all_points = []
        
        for detection in active_detections:
            bbox = detection.smooth_bbox if detection.smooth_bbox else detection.bbox
            points = self.extract_pointcloud_from_bbox(bbox)
            if points is not None:
                all_points.extend(points)
        
        # ポイントクラウドをパブリッシュ
        if all_points:
            self.publish_pointcloud(all_points)
        
        # 可視化画像を生成・パブリッシュ
        if self.latest_color is not None:
            self.publish_visualization(active_detections)
    
    def extract_pointcloud_from_bbox(self, bbox: Tuple[int, int, int, int]) -> Optional[List]:
        """バウンディングボックス内の3Dポイントを抽出"""
        x, y, width, height = bbox
        
        # 深度画像のサイズを確認
        depth_height, depth_width = self.latest_depth.shape
        
        # バウンディングボックスをクリップ
        x_start = max(0, x)
        y_start = max(0, y)
        x_end = min(depth_width, x + width)
        y_end = min(depth_height, y + height)
        
        if x_start >= x_end or y_start >= y_end:
            return None
        
        # バウンディングボックス内の深度値を取得
        depth_roi = self.latest_depth[y_start:y_end, x_start:x_end]
        
        # 有効な深度値のマスクを作成（単位：mm）
        valid_mask = (depth_roi > self.min_depth * 1000) & (depth_roi < self.max_depth * 1000)
        
        # 有効なピクセルの座標を取得
        v_indices, u_indices = np.where(valid_mask)
        
        if len(v_indices) == 0:
            return None
        
        # グローバル座標に変換
        u_global = u_indices + x_start
        v_global = v_indices + y_start
        
        # 深度値を取得（メートルに変換）
        depths = depth_roi[v_indices, u_indices] / 1000.0
        
        # 3D座標を計算
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        cx = self.intrinsics['cx']
        cy = self.intrinsics['cy']
        
        x_3d = (u_global - cx) * depths / fx
        y_3d = (v_global - cy) * depths / fy
        z_3d = depths
        
        # ポイントリストを作成
        points = []
        for i in range(len(x_3d)):
            # カラー情報も追加（BGR -> RGB）
            if self.latest_color is not None:
                color = self.latest_color[v_global[i], u_global[i]]
                rgb = (color[2] << 16) | (color[1] << 8) | color[0]
            else:
                rgb = 0xFFFFFF
            
            points.append([x_3d[i], y_3d[i], z_3d[i], rgb])
        
        return points
    
    def publish_pointcloud(self, points: List):
        """ポイントクラウドをパブリッシュ"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = f'{self.camera_name}_depth_optical_frame'
        
        # PointCloud2メッセージを作成
        cloud_msg = pc2.create_cloud_xyz32(header, 
                                          [[p[0], p[1], p[2]] for p in points])
        
        self.pointcloud_pub.publish(cloud_msg)
        
        self.get_logger().debug(f'Published {len(points)} points')
    
    def publish_visualization(self, detections: List[DetectionInfo]):
        """可視化画像をパブリッシュ"""
        vis_image = self.latest_color.copy()
        
        for detection in detections:
            bbox = detection.smooth_bbox if detection.smooth_bbox else detection.bbox
            x, y, width, height = bbox
            
            # アニメーション効果（新しい検出ほど明るく）
            age = time.time() - detection.timestamp
            alpha = max(0.3, 1.0 - age * self.animation_speed)
            
            # 色を決定（信頼度に応じて）
            if detection.confidence > 0.7:
                color = (0, 255, 0)  # 緑
            elif detection.confidence > 0.5:
                color = (0, 255, 255)  # 黄色
            else:
                color = (0, 0, 255)  # 赤
            
            # バウンディングボックスを描画（アルファブレンディング）
            overlay = vis_image.copy()
            cv2.rectangle(overlay, (x, y), (x + width, y + height), color, 2)
            
            # テキストを追加
            text = f'ID:{detection.id} {detection.confidence:.2f}'
            cv2.putText(overlay, text, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # アルファブレンディング
            vis_image = cv2.addWeighted(overlay, alpha, vis_image, 1 - alpha, 0)
        
        # 画像をパブリッシュ
        try:
            msg = self.bridge.cv2_to_imgmsg(vis_image, 'bgr8')
            self.visualization_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish visualization: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = EfficientPointCloudProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()