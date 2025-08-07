#!/usr/bin/env python3
"""
FluentVision テスト用ストリーミングサーバー
MJPEGストリームをシミュレート
"""

import cv2
import numpy as np
from flask import Flask, Response
import time
import threading

app = Flask(__name__)

class TestStreamGenerator:
    def __init__(self):
        self.frame_count = 0
        
    def generate_test_frame(self, stream_id=1):
        """テスト用の画像フレームを生成"""
        # 640x480のカラフルなテスト画像を作成
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 背景のグラデーション
        for i in range(480):
            color = int(255 * (i / 480))
            img[i, :] = [color // 2, color // 3, color]
        
        # 動くボックス
        box_x = int(320 + 200 * np.sin(self.frame_count * 0.05))
        box_y = int(240 + 100 * np.cos(self.frame_count * 0.05))
        cv2.rectangle(img, (box_x-50, box_y-50), (box_x+50, box_y+50), 
                     (0, 255, 0), -1)
        
        # ストリーム情報を表示
        cv2.putText(img, f"Stream {stream_id}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"Frame: {self.frame_count}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, time.strftime("%H:%M:%S"), (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        self.frame_count += 1
        
        # JPEGエンコード
        ret, jpeg = cv2.imencode('.jpg', img)
        return jpeg.tobytes()
    
    def generate_stream(self, stream_id=1):
        """MJPEGストリームを生成"""
        while True:
            frame = self.generate_test_frame(stream_id)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)  # 約30fps

# 各ストリーム用のジェネレーター
generators = {
    1: TestStreamGenerator(),
    2: TestStreamGenerator(),
    3: TestStreamGenerator(),
    4: TestStreamGenerator()
}

@app.route('/')
def index():
    return '''
    <html>
    <head>
        <title>FluentVision Test Streams</title>
        <style>
            body {
                font-family: Arial, sans-serif;
                background: #1a1a1a;
                color: white;
                padding: 20px;
            }
            h1 { color: #6366f1; }
            .stream-list {
                display: grid;
                grid-template-columns: repeat(2, 1fr);
                gap: 20px;
                max-width: 800px;
                margin: 20px auto;
            }
            .stream-item {
                background: #2a2a2a;
                padding: 15px;
                border-radius: 8px;
                border: 1px solid #444;
            }
            .stream-item img {
                width: 100%;
                height: auto;
                border-radius: 4px;
                margin-top: 10px;
            }
            code {
                background: #333;
                padding: 4px 8px;
                border-radius: 4px;
                font-size: 12px;
            }
        </style>
    </head>
    <body>
        <h1>🎥 FluentVision Test Streams</h1>
        <p>テスト用MJPEGストリーミングサーバー</p>
        
        <div class="stream-list">
            <div class="stream-item">
                <h3>Stream 1 - Robot Camera</h3>
                <code>http://localhost:5000/stream/1</code>
                <img src="/stream/1" />
            </div>
            <div class="stream-item">
                <h3>Stream 2 - Sensor View</h3>
                <code>http://localhost:5000/stream/2</code>
                <img src="/stream/2" />
            </div>
            <div class="stream-item">
                <h3>Stream 3 - Navigation</h3>
                <code>http://localhost:5000/stream/3</code>
                <img src="/stream/3" />
            </div>
            <div class="stream-item">
                <h3>Stream 4 - Overview</h3>
                <code>http://localhost:5000/stream/4</code>
                <img src="/stream/4" />
            </div>
        </div>
        
        <p style="text-align: center; margin-top: 40px;">
            FluentVisionアプリで上記のURLを使用してストリームを追加できます
        </p>
    </body>
    </html>
    '''

@app.route('/stream/<int:stream_id>')
def stream(stream_id):
    """MJPEGストリームエンドポイント"""
    if stream_id not in generators:
        stream_id = 1
    return Response(generators[stream_id].generate_stream(stream_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def status():
    """ステータスAPI"""
    return {
        'server': 'FluentVision Test Server',
        'streams': [1, 2, 3, 4],
        'status': 'running'
    }

if __name__ == '__main__':
    print("=" * 50)
    print("FluentVision Test Stream Server")
    print("=" * 50)
    print("Server: http://localhost:5000")
    print("Stream URLs:")
    print("  - http://localhost:5000/stream/1")
    print("  - http://localhost:5000/stream/2")
    print("  - http://localhost:5000/stream/3")
    print("  - http://localhost:5000/stream/4")
    print("=" * 50)
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)