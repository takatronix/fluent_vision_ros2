#!/usr/bin/env python3
"""
Simple ROS2 WebSocket Bridge for FluentVision
rosbridgeの代替として動作する軽量サーバー
"""

import asyncio
import json
import base64
import numpy as np
from aiohttp import web
import aiohttp_cors
import cv2

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CompressedImage
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("⚠️ ROS2が見つかりません。デモモードで動作します。")

class SimpleROS2Bridge:
    def __init__(self):
        self.clients = set()
        self.bridge = CvBridge() if ROS2_AVAILABLE else None
        self.node = None
        self.subscriptions = {}
        
    async def websocket_handler(self, request):
        """WebSocket接続ハンドラー"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self.clients.add(ws)
        
        print(f"✅ クライアント接続: {request.remote}")
        
        try:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    await self.handle_message(ws, data)
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    print(f'WebSocket error: {ws.exception()}')
        finally:
            self.clients.remove(ws)
            print(f"❌ クライアント切断: {request.remote}")
            
        return ws
    
    async def handle_message(self, ws, data):
        """メッセージ処理"""
        msg_type = data.get('type')
        
        if msg_type == 'subscribe':
            topic = data.get('topic')
            await self.subscribe_topic(topic, ws)
            
        elif msg_type == 'unsubscribe':
            topic = data.get('topic')
            await self.unsubscribe_topic(topic, ws)
            
        elif msg_type == 'list_topics':
            await self.send_topic_list(ws)
    
    async def subscribe_topic(self, topic, ws):
        """トピックの購読"""
        if ROS2_AVAILABLE and self.node:
            # 実際のROS2トピックに接続
            if topic not in self.subscriptions:
                # 画像トピックの場合
                if 'image' in topic.lower():
                    sub = self.node.create_subscription(
                        CompressedImage if 'compressed' in topic else Image,
                        topic,
                        lambda msg: asyncio.create_task(self.image_callback(topic, msg)),
                        10
                    )
                    self.subscriptions[topic] = sub
                    print(f"📷 画像トピック購読: {topic}")
            
            await ws.send_str(json.dumps({
                'type': 'subscribed',
                'topic': topic,
                'status': 'success'
            }))
        else:
            # デモモード
            await ws.send_str(json.dumps({
                'type': 'subscribed',
                'topic': topic,
                'status': 'demo_mode'
            }))
            # デモ画像を定期的に送信
            asyncio.create_task(self.send_demo_images(topic, ws))
    
    async def image_callback(self, topic, msg):
        """画像メッセージの処理"""
        try:
            # 画像をJPEGに変換
            if hasattr(msg, 'data') and isinstance(msg, CompressedImage):
                # 既に圧縮済み
                jpeg_data = msg.data
            else:
                # OpenCVで変換
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                _, jpeg = cv2.imencode('.jpg', cv_image)
                jpeg_data = jpeg.tobytes()
            
            # Base64エンコード
            b64_data = base64.b64encode(jpeg_data).decode('utf-8')
            
            # 全クライアントに送信
            message = json.dumps({
                'type': 'image',
                'topic': topic,
                'data': f'data:image/jpeg;base64,{b64_data}',
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            })
            
            for client in self.clients:
                try:
                    await client.send_str(message)
                except:
                    pass
                    
        except Exception as e:
            print(f"画像処理エラー: {e}")
    
    async def send_demo_images(self, topic, ws):
        """デモ画像の送信"""
        frame_count = 0
        while ws in self.clients:
            # テスト画像を生成
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # アニメーション
            x = int(320 + 200 * np.sin(frame_count * 0.05))
            y = int(240 + 100 * np.cos(frame_count * 0.05))
            cv2.circle(img, (x, y), 50, (0, 255, 0), -1)
            
            # テキスト
            cv2.putText(img, f"Demo: {topic}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(img, f"Frame: {frame_count}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # JPEGエンコード
            _, jpeg = cv2.imencode('.jpg', img)
            b64_data = base64.b64encode(jpeg.tobytes()).decode('utf-8')
            
            # 送信
            await ws.send_str(json.dumps({
                'type': 'image',
                'topic': topic,
                'data': f'data:image/jpeg;base64,{b64_data}',
                'timestamp': frame_count / 30.0
            }))
            
            frame_count += 1
            await asyncio.sleep(0.033)  # 30fps
    
    async def send_topic_list(self, ws):
        """トピックリストの送信"""
        if ROS2_AVAILABLE and self.node:
            topics = self.node.get_topic_names_and_types()
            topic_list = [{'name': t[0], 'type': t[1][0]} for t in topics]
        else:
            # デモトピック
            topic_list = [
                {'name': '/camera/image_raw', 'type': 'sensor_msgs/msg/Image'},
                {'name': '/camera/image_raw/compressed', 'type': 'sensor_msgs/msg/CompressedImage'},
                {'name': '/demo/stream1', 'type': 'sensor_msgs/msg/Image'},
                {'name': '/demo/stream2', 'type': 'sensor_msgs/msg/Image'},
            ]
        
        await ws.send_str(json.dumps({
            'type': 'topics',
            'data': topic_list
        }))
    
    async def index_handler(self, request):
        """テストページ"""
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>ROS2 WebSocket Bridge Test</title>
            <style>
                body { font-family: Arial; padding: 20px; background: #1a1a1a; color: white; }
                .status { padding: 10px; margin: 10px 0; border-radius: 5px; }
                .connected { background: #22c55e; }
                .disconnected { background: #ef4444; }
                #images { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
                img { width: 100%; border: 1px solid #444; }
            </style>
        </head>
        <body>
            <h1>🚀 Simple ROS2 WebSocket Bridge</h1>
            <div id="status" class="status disconnected">Disconnected</div>
            <button onclick="connect()">Connect</button>
            <button onclick="subscribe('/camera/image_raw/compressed')">Subscribe Camera</button>
            <button onclick="listTopics()">List Topics</button>
            <div id="topics"></div>
            <div id="images"></div>
            
            <script>
                let ws = null;
                
                function connect() {
                    ws = new WebSocket('ws://localhost:9090');
                    
                    ws.onopen = () => {
                        document.getElementById('status').className = 'status connected';
                        document.getElementById('status').textContent = 'Connected';
                    };
                    
                    ws.onclose = () => {
                        document.getElementById('status').className = 'status disconnected';
                        document.getElementById('status').textContent = 'Disconnected';
                    };
                    
                    ws.onmessage = (event) => {
                        const msg = JSON.parse(event.data);
                        
                        if (msg.type === 'image') {
                            let img = document.getElementById(msg.topic);
                            if (!img) {
                                img = document.createElement('img');
                                img.id = msg.topic;
                                document.getElementById('images').appendChild(img);
                            }
                            img.src = msg.data;
                        } else if (msg.type === 'topics') {
                            document.getElementById('topics').innerHTML = 
                                '<h3>Topics:</h3>' + 
                                msg.data.map(t => `<div>${t.name} (${t.type})</div>`).join('');
                        }
                    };
                }
                
                function subscribe(topic) {
                    if (ws && ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify({type: 'subscribe', topic: topic}));
                    }
                }
                
                function listTopics() {
                    if (ws && ws.readyState === WebSocket.OPEN) {
                        ws.send(JSON.stringify({type: 'list_topics'}));
                    }
                }
                
                // 自動接続
                connect();
            </script>
        </body>
        </html>
        """
        return web.Response(text=html, content_type='text/html')
    
    async def init_ros2(self):
        """ROS2の初期化"""
        if ROS2_AVAILABLE:
            rclpy.init()
            self.node = Node('fluentvision_bridge')
            print("✅ ROS2ノード初期化完了")
        else:
            print("⚠️ ROS2なしでデモモード動作中")
    
    def setup_routes(self, app):
        """ルート設定"""
        app.router.add_get('/', self.index_handler)
        app.router.add_get('/ws', self.websocket_handler)
        
        # CORS設定
        cors = aiohttp_cors.setup(app, defaults={
            "*": aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                expose_headers="*",
                allow_headers="*",
                allow_methods="*"
            )
        })
        
        for route in list(app.router.routes()):
            cors.add(route)

async def main():
    bridge = SimpleROS2Bridge()
    await bridge.init_ros2()
    
    app = web.Application()
    bridge.setup_routes(app)
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', 9090)
    
    print("=" * 50)
    print("🚀 Simple ROS2 WebSocket Bridge")
    print("=" * 50)
    print("WebSocket: ws://localhost:9090")
    print("Test Page: http://localhost:9090")
    print("=" * 50)
    
    await site.start()
    
    # ROS2スピン（利用可能な場合）
    if ROS2_AVAILABLE and bridge.node:
        try:
            while True:
                rclpy.spin_once(bridge.node, timeout_sec=0.1)
                await asyncio.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            bridge.node.destroy_node()
            rclpy.shutdown()
    else:
        # デモモードで永続実行
        await asyncio.Event().wait()

if __name__ == '__main__':
    # 必要なパッケージをインストール
    import subprocess
    import sys
    
    required = ['aiohttp', 'aiohttp-cors', 'opencv-python', 'numpy']
    for package in required:
        try:
            __import__(package.replace('-', '_'))
        except ImportError:
            print(f"Installing {package}...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])
    
    asyncio.run(main())